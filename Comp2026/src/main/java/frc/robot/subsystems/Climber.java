//
// Climber Subystem - lifts the robot to hang onto the tower rung
//
package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Ports;
import frc.robot.lib.math.Conversions;
import frc.robot.lib.phoenix.CTREConfigs6;
import frc.robot.lib.phoenix.PhoenixUtil6;

/****************************************************************************
 * 
 * Climber subsystem class - left side climber
 */
public class Climber extends SubsystemBase
{
  // Constants
  private static final String  kSubsystemName       = "Climber";
  private static final double  kGearRatio           = 16.0;    // Gear reduction
  private static final double  kClimberLengthMeters = 0.5;     // Simulation
  private static final double  kCarriageMassKg      = 2.0;     // Simulation
  private static final double  kDrumDiameterInches  = 1.375;   // Drum diameter in inches
  private static final double  kDrumRadiusMeters    = Units.inchesToMeters(kDrumDiameterInches) / 2;
  private static final double  kRolloutRatio        = kDrumDiameterInches * Math.PI / kGearRatio; // inches per shaft rotation
  private static final Voltage kCalibrateSpeedVolts = Volts.of(-1.0);           // Motor voltage during calibration
  private static final Voltage kManualSpeedVolts    = Volts.of(3.0);  // Motor voltage during manual operation (joystick)
  private static final double  kCalibrateStallAmps  = 10.0;    // Motor supply current amps during calibration stall
  private static final double  kCalibrateStallTime  = 0.100;   // Seconds of stall before calibrating
  private static final double  kCalibrationTimeout  = 1.5;     // Max calibration time

  private static final double  kToleranceInches     = 0.5;     // Climber PID tolerance in inches
  private static final double  kMMDebounceTime      = 0.060;   // Seconds to debounce a final position check
  private static final double  kMMMoveTimeout       = 2.5;     // Seconds allowed for a Motion Magic movement

  // Climber lengths - Motion Magic config parameters
  private static final double  kLengthClimbed       = 0.0;     // By definition - Climber fully climbed
  private static final double  kLengthStowed        = 0.0;     // Climber fully down/back
  private static final double  kLengthFull          = 10.5;

  private static final double  kLengthMin           = 0.0;     // Climber minimum allowable length
  private static final double  kLengthMax           = 10.5;    // Climber maximum allowable length (2" beyond high length)

  /** Climber manual move parameters */
  private enum ClimberMode
  {
    INIT,   // Initialize climber
    UP,     // Climber move upward
    STOP,   // Climber stop
    DOWN    // Climber move downward
  }

  // Device objects
  private TalonFX                     m_climbMotor        = new TalonFX(Ports.kCANID_ClimberRight);

  // Alerts
  private final Alert                 m_alert             =
      new Alert(String.format("%s: Motor init failed!", getSubsystem( )), AlertType.kError);

  // Simulation objects
  private final TalonFXSimState       m_climberSim        = m_climbMotor.getSimState( );
  private final ElevatorSim           m_elevSim           = new ElevatorSim(DCMotor.getKrakenX60(1), kGearRatio, kCarriageMassKg,
      kDrumRadiusMeters, -kLengthMax, kLengthMax, false, 0.0);

  // Mechanism2d
  private final Mechanism2d           m_climberMech       = new Mechanism2d(1.0, 1.0);
  private final MechanismLigament2d   m_mechLigament      = m_climberMech.getRoot("Linear", 0.5, 0.5)
      .append(new MechanismLigament2d(kSubsystemName, kClimberLengthMeters, 90.0, 6, new Color8Bit(Color.kRed)));

  // CTRE Status signals for sensors
  private final StatusSignal<Angle>   m_position;    // Default 4Hz (250ms)
  private final StatusSignal<Current> m_supplyCur;   // Default 4Hz (250ms)

  // Declare module variables
  private String                      kSubsystemFullName  = kSubsystemName;
  private String                      kNTPrefix           = "missing";
  private String                      kNTTableName        = "missing";
  private boolean                     m_climberValid;             // Health indicator for motor 
  private double                      m_curLength         = 0.0;  // Current length in inches
  private double                      m_targetLength      = 0.0;  // Target length in inches

  // Calibration variables
  private Timer                       m_calibrateTimer    = new Timer( );
  private Debouncer                   m_stalled           = new Debouncer(kCalibrateStallTime, DebounceType.kRising);
  private boolean                     m_calibrated        = false;

  // Manual mode config parameters
  private VoltageOut                  m_requestVolts      = new VoltageOut(Volts.of(0));
  private ClimberMode                 m_mode              = ClimberMode.INIT;       // Manual movement mode with joysticks

  // Motion Magic mode config parameters
  private MotionMagicVoltage          m_mmRequestVolts    = new MotionMagicVoltage(0).withSlot(0).withEnableFOC(false);
  private Debouncer                   m_mmWithinTolerance = new Debouncer(kMMDebounceTime, DebounceType.kRising);
  private Timer                       m_mmMoveTimer       = new Timer( );           // Movement timer
  private Voltage                     m_mmArbFeedForward  = Volts.of(0);  // Arbitrary feedforward added to counteract gravity
  private int                         m_mmHardStopCounter = 0;
  private boolean                     m_mmMoveIsFinished  = true;                   // Movement has completed (within tolerance)

  // Network tables publisher objects
  private BooleanPublisher            m_calibratedPub;
  private DoublePublisher             m_curLengthPub;
  private DoublePublisher             m_targetLengthPub;

  /****************************************************************************
   * 
   * Constructor
   */
  public Climber(String instanceName, String prefix, boolean opposing)
  {
    kSubsystemFullName = kSubsystemName + instanceName;
    setName(kSubsystemFullName);
    setSubsystem(kSubsystemFullName);

    kNTPrefix = prefix;
    kNTTableName = "climber" + instanceName;

    // m_climbMotor = (instanceName == "Left") ? new TalonFX(Ports.kCANID_ClimberLeft) : new TalonFX(Ports.kCANID_ClimberRight);

    // Initialize climber motor objects
    m_climberValid = PhoenixUtil6.getInstance( ).talonFXInitialize6(m_climbMotor, kSubsystemFullName,
        CTREConfigs6.climberFXConfig(true, Units.degreesToRotations(kLengthMin), Units.degreesToRotations(kLengthMax)));

    m_alert.set(!m_climberValid);

    // Initialize status signal objects
    m_position = m_climbMotor.getRotorPosition( );
    m_supplyCur = m_climbMotor.getSupplyCurrent( );

    // Initialize the climber status signals
    if (m_climberValid)
    {
      setClimberPosition(m_curLength);

      // Status signals
      BaseStatusSignal.setUpdateFrequencyForAll(50, m_position);

      DataLogManager.log(String.format("%s: Update (Hz) position: %.1f supplyCur: %.1f", getSubsystem( ),
          m_position.getAppliedUpdateFrequency( ), m_supplyCur.getAppliedUpdateFrequency( )));
    }

    DataLogManager.log(String.format("%s: Initial position L %.1f inches", getSubsystem( ), m_curLength));

    // Simulation object initialization
    m_climberSim.Orientation = ChassisReference.CounterClockwise_Positive;

    initDashboard( );
    initialize( );
  }

  /****************************************************************************
   * 
   * Periodic actions that run every scheduler loop time (20 msec)
   */
  @Override
  public void periodic( )
  {
    // This method will be called once per scheduler run

    if (m_climberValid)
    {
      BaseStatusSignal.refreshAll(m_position);
      m_curLength = Conversions.rotationsToWinchRollout(m_position.getValue( ).in(Rotations), kRolloutRatio);
      if (m_curLength < 0)
        setClimberPosition(0.0);
    }

    // Update network table publishers
    m_calibratedPub.set((m_calibrated));
    m_curLengthPub.set(m_curLength);
    m_targetLengthPub.set(m_targetLength);
  }

  /****************************************************************************
   * 
   * Periodic actions that run every scheduler loop time (20 msec) during simulation
   */
  @Override
  public void simulationPeriodic( )
  {
    // This method will be called once per scheduler run during simulation

    // Set input motor voltage from the motor setting
    m_climberSim.setSupplyVoltage(RobotController.getInputVoltage( ));
    m_elevSim.setInput(m_climberSim.getMotorVoltage( ));

    // update for 20 msec loop
    m_elevSim.update(0.020);

    // Finally, we set our simulated encoder's readings and simulated battery voltage
    m_climberSim.setRawRotorPosition(
        Conversions.rolloutToWinchRotations(Units.metersToInches(m_elevSim.getPositionMeters( )), kRolloutRatio));
    m_climberSim.setRotorVelocity(
        Conversions.rolloutToWinchRotations(Units.metersToInches(m_elevSim.getVelocityMetersPerSecond( )), kRolloutRatio));

    // SimBattery estimates loaded battery voltages
    RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(m_elevSim.getCurrentDrawAmps( )));

    m_mechLigament.setLength(0.1 + Units.inchesToMeters(m_curLength));
  }

  /****************************************************************************
   * 
   * Initialize dashboard widgets
   */
  private void initDashboard( )
  {
    // Get the default instance of NetworkTables that was created automatically when the robot program starts
    NetworkTableInstance inst = NetworkTableInstance.getDefault( );
    NetworkTable table = inst.getTable(kNTTableName);

    // Initialize network tables publishers
    m_calibratedPub = table.getBooleanTopic(kNTPrefix + "calibrated").publish( );
    m_curLengthPub = table.getDoubleTopic(kNTPrefix + "currentInches").publish( );
    m_targetLengthPub = table.getDoubleTopic(kNTPrefix + "targetInches").publish( );

    SmartDashboard.putData(kNTPrefix + "Mech", m_climberMech);

    // Add commands
    SmartDashboard.putData(kSubsystemFullName + "Extend", getMoveToPositionCommand(this::getClimberFullyExtended));
    SmartDashboard.putData(kSubsystemFullName + "Lift", getMoveToPositionCommand(this::getClimberClimbed));
    SmartDashboard.putData(kSubsystemFullName + "Calibrate", getCalibrateCommand( ));
  }

  // Put methods for controlling this subsystem here. Call these from Commands.

  /****************************************************************************
   * 
   * Initialize subsystem during robot mode changes
   */
  public void initialize( )
  {
    setVoltage(Volts.of(0.0));
    m_calibrated = false;

    m_curLength = 0.0; // Allow calibration routine to run for up to this length
    m_targetLength = 0.0;
    DataLogManager.log(String.format("%s: Subsystem initialized! Target Inches: %.1f", getSubsystem( ), m_targetLength));
  }

  /****************************************************************************
   * 
   * Write out hardware faults and reset sticky faults
   */
  public void printFaults( )
  {
    if (m_climberValid)
    {
      PhoenixUtil6.getInstance( ).talonFXPrintFaults(m_climbMotor, getSubsystem( ));
      m_climbMotor.clearStickyFaults( );
    }
    else
    {
      DataLogManager.log(String.format("%s: m_climberValid is FALSE!", getSubsystem( )));
    }
  }

  ////////////////////////////////////////////////////////////////////////////
  ///////////////////////// MANUAL MOVEMENT //////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////

  /****************************************************************************
   * 
   * Move motors proportional to a joystick axis value
   * 
   * @param getAxis
   *          double supplier that returns desired joystick axis
   */
  private void moveWithJoystick(DoubleSupplier getAxis)
  {
    double axisValue = getAxis.getAsDouble( );
    boolean rangeLimited = false;
    ClimberMode newMode = ClimberMode.STOP;

    axisValue = MathUtil.applyDeadband(axisValue, Constants.kStickDeadband);

    if ((axisValue < 0.0) && (m_curLength > kLengthMin))
    {
      newMode = ClimberMode.DOWN;
    }
    else if ((axisValue > 0.0) && (m_curLength < kLengthMax))
    {
      newMode = ClimberMode.UP;
    }
    else
    {
      rangeLimited = true;
      axisValue = 0.0;
    }

    if (newMode != m_mode)
    {
      m_mode = newMode;
      DataLogManager.log(String.format("%s: Manual move mode %s now L %.1f inches %s", getSubsystem( ), m_mode, m_curLength,
          ((rangeLimited) ? " - RANGE LIMITED" : "")));
    }

    m_targetLength = m_curLength;

    setVoltage(kManualSpeedVolts.times(axisValue));
  }

  ////////////////////////////////////////////////////////////////////////////
  ///////////////////////// MOTION MAGIC /////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////

  /****************************************************************************
   * 
   * Initialize a Motion Magic movement
   * 
   * @param newLength
   *          distance to move
   * @param holdPosition
   *          hold previous position if true
   */
  private void moveToPositionInit(double newLength, boolean holdPosition)
  {
    m_mmMoveTimer.restart( );
    m_mmHardStopCounter = 0;

    if (!(m_calibrated))
    {
      DataLogManager.log(String.format("%s: MM Position move target %.1f in - NOT CALIBRATED!", getSubsystem( ), m_targetLength));
      return;
    }

    if (holdPosition)
    {
      newLength = m_curLength;
    }

    newLength = MathUtil.clamp(newLength, 0.25, kLengthMax);

    // Decide if a new position request
    if (holdPosition || newLength != m_targetLength || !MathUtil.isNear(newLength, m_curLength, kToleranceInches))
    {
      // Validate the position request
      if (isMoveValid(newLength))
      {
        m_targetLength = newLength;
        m_mmMoveIsFinished = false;
        m_mmWithinTolerance.calculate(false); // Reset the debounce filter

        setMMPosition(m_targetLength);

        DataLogManager.log(String.format("%s: MM Position move: %.1f -> %.1f inches (%.3f -> %.3f rot)", getSubsystem( ),
            m_curLength, m_targetLength, Conversions.rolloutToWinchRotations(m_curLength, kRolloutRatio),
            Conversions.rolloutToWinchRotations(m_targetLength, kRolloutRatio)));
      }
      else
        DataLogManager.log(String.format("%s: MM Position move target %.1f inches is OUT OF RANGE! [%.1f, %.1f rot]",
            getSubsystem( ), m_targetLength, kLengthMin, kLengthMax));
    }
    else
    {
      m_mmMoveIsFinished = true;
      DataLogManager.log(String.format("%s: MM Position already achieved - target %s inches", getSubsystem( ), m_targetLength));
    }
  }

  /****************************************************************************
   * 
   * Continuously update Motion Magic setpoint
   */
  private void moveToPositionExecute( )
  {}

  /****************************************************************************
   * 
   * Detect Motion Magic finished state
   * 
   * @param holdPosition
   *          hold the current position
   * @return true when movement has completed
   */
  private boolean moveToPositionIsFinished(boolean holdPosition)
  {
    boolean timedOut = m_mmMoveTimer.hasElapsed(kMMMoveTimeout);
    double error = m_targetLength - m_curLength;
    boolean hittingHardStop = (m_targetLength <= 0.0) && (m_curLength <= 1.0) && (m_mmHardStopCounter++ >= 10);

    setMMPosition(m_targetLength);

    if (holdPosition)
      return false;

    if (m_mmWithinTolerance.calculate(Math.abs(error) < kToleranceInches) || timedOut || hittingHardStop)
    {
      if (hittingHardStop)
        DataLogManager.log(String.format("%s - hittingHardStop: %s", getSubsystem( ), hittingHardStop));
      if (!m_mmMoveIsFinished)
        DataLogManager
            .log(String.format("%s: MM Position move finished - Current inches: %.1f (difference %.1f) - Time: %.3f sec %s",
                getSubsystem( ), m_curLength, error, m_mmMoveTimer.get( ), (timedOut) ? "- Warning: TIMED OUT!" : ""));

      m_mmMoveIsFinished = true;
    }

    return m_mmMoveIsFinished;
  }

  /****************************************************************************
   * 
   * Wrap up a Motion Magic movement
   */
  private void moveToPositionEnd( )
  {
    m_mmMoveTimer.stop( );
  }

  ////////////////////////////////////////////////////////////////////////////
  ///////////////////////// CALIBRATION //////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////

  /****************************************************************************
   * 
   * Initialize calibration movement
   */
  private void calibrateInit( )
  {
    // Reset the calibration state, time, debounce filters, and motor setting
    DataLogManager.log(String.format("%s: Start up (%s, %s)", getSubsystem( ), kCalibrateSpeedVolts.toString( ),
        kCalibrateSpeedVolts.toString( )));
    m_calibrated = false;
    m_calibrateTimer.restart( );
    m_stalled.calculate(false);
    setVoltage(kCalibrateSpeedVolts);
  }

  /****************************************************************************
   * 
   * Move climber down during calibration
   */
  private void calibrateExecute( )
  {}

  /****************************************************************************
   * 
   * Check for climber full down during calibration
   * 
   * @return true when command has completed
   */
  private boolean calibrateIsFinished( )
  {
    boolean calibrated = m_stalled.calculate(m_supplyCur.getValue( ).in(Amps) > kCalibrateStallAmps);

    if (calibrated && !m_calibrated)
      DataLogManager.log(String.format("%s: Stalled %s", getSubsystem( ), calibrated));

    m_calibrated = calibrated;

    setVoltage((m_calibrated) ? Volts.of(0.0) : kCalibrateSpeedVolts);

    return (m_calibrated) || m_calibrateTimer.hasElapsed(kCalibrationTimeout);
  }

  /****************************************************************************
   * 
   * Wrap up calibration sequence
   */
  private void calibrateEnd( )
  {
    DataLogManager.log(String.format("%s: End - elapsed %.3f sec", getSubsystem( ), m_calibrateTimer.get( )));
    m_calibrateTimer.stop( );
    setClimberPosition(0.0);
    setVoltage(Volts.of(0.0));
    m_targetLength = m_curLength;
    m_calibrated = true;
  }

  ////////////////////////////////////////////////////////////////////////////
  ///////////////////////// PRIVATE HELPERS ///////////////////////////////
  ////////////////////////////////////////////////////////////////////////////

  /****************************************************************************
   * 
   * Set climber encoder position
   * 
   * @param inches
   *          height to set
   */
  private void setClimberPosition(double inches)
  {
    m_curLength = inches;
    if (m_climberValid)
    {
      double rotations = Conversions.rolloutToWinchRotations(inches, kRolloutRatio);
      m_climbMotor.setPosition(rotations);
    }
  }

  /****************************************************************************
   * 
   * Set climber motors to a known voltage
   * 
   * @param volts
   *          voltage to apply to climber motor (0.0 is stopped)
   */
  private void setVoltage(Voltage volts)
  {
    if (m_climberValid)
    {
      m_climbMotor.setControl(m_requestVolts.withOutput(volts));
    }
  }

  /****************************************************************************
   * 
   * Set Motion Magic setpoint based on passed length
   * 
   * @param targetInches
   *          distance to move
   */
  private void setMMPosition(double targetInches)
  {
    if (m_climberValid)
    {
      // y = mx + b, where 0 degrees is 0.0 climber and 90 degrees is 1/4 winch turn (the climber constant)
      double position = Conversions.rolloutToWinchRotations(targetInches, kRolloutRatio);
      m_climbMotor.setControl(m_mmRequestVolts.withPosition(position).withFeedForward(m_mmArbFeedForward));
    }
  }

  /****************************************************************************
   * 
   * Validate requested climber move
   * 
   * @param inches
   *          distance requested
   * @return true if move is within range
   */
  private boolean isMoveValid(double inches)
  {
    return (inches >= kLengthMin) && (inches <= kLengthMax);
  }

  ////////////////////////////////////////////////////////////////////////////
  ///////////////////////// PUBLIC HELPERS ///////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////

  /****************************************************************************
   * 
   * Return current cliimber position
   * 
   * @return current climber position in inches
   */
  public double getClimberPosition( )
  {
    return m_curLength;
  }

  /****************************************************************************
   * 
   * Return climber length for climbed state
   * 
   * @return climbed state length
   */
  public double getClimberClimbed( )
  {
    return kLengthClimbed;
  }

  /****************************************************************************
   * 
   * Return climber length for fully extended state
   * 
   * @return fully extended state length
   */
  public double getClimberFullyExtended( )
  {
    return kLengthFull;
  }

  /****************************************************************************
   * 
   * Return climber length for fully extended state
   * 
   * @return fully extended state length
   */
  public double getClimberStowed( )
  {
    return kLengthStowed;
  }

  ////////////////////////////////////////////////////////////////////////////
  ///////////////////////// COMMAND FACTORIES ////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////

  /****************************************************************************
   * 
   * Create calibration command
   * 
   * @return continuous command that runs climber motors
   */
  public Command getCalibrateCommand( )
  {
    return new FunctionalCommand(       // Command with all phases declared
        ( ) -> calibrateInit( ),        // Init method
        ( ) -> calibrateExecute( ),     // Execute method
        interrupted -> calibrateEnd( ), // End method
        ( ) -> calibrateIsFinished( ),  // IsFinished method
        this                            // Subsytem required
    )                                   //
        .withName(kSubsystemName + "Calibrate");
  }

  /****************************************************************************
   * 
   * Create joystick manual move command
   * 
   * @param axis
   *          double supplier that provides the joystick axis value
   * @return continuous command that runs climber motors
   */
  public Command getJoystickCommand(DoubleSupplier axis)
  {
    return new RunCommand(              // Command that runs continuously
        ( ) -> moveWithJoystick(axis),  // Lambda method to call
        this                            // Subsystem required
    ).withName(kSubsystemName + "MoveWithJoystick");
  }

  /****************************************************************************
   * 
   * Create motion magic base command
   * 
   * @param position
   *          double supplier that provides the target distance
   * @param holdPosition
   *          boolen to indicate whether the command ever finishes
   * @return continuous command that runs climber motors
   */
  private Command getMMPositionCommand(DoubleSupplier position, boolean holdPosition)
  {
    return new FunctionalCommand(                                         // Command with all phases declared
        ( ) -> moveToPositionInit(position.getAsDouble( ), holdPosition), // Init method
        ( ) -> moveToPositionExecute( ),                                  // Execute method
        interrupted -> moveToPositionEnd( ),                              // End method
        ( ) -> moveToPositionIsFinished(holdPosition),                    // IsFinished method
        this                                                              // Subsytem required
    );
  }

  /****************************************************************************
   * 
   * Create motion magic move to position command
   * 
   * @param position
   *          double supplier that provides the target distance value
   * @return continuous command that runs climber motors
   */
  public Command getMoveToPositionCommand(DoubleSupplier position)
  {
    return getMMPositionCommand(position, false).withName(kSubsystemName + "MMMoveToPosition");
  }

  /****************************************************************************
   * 
   * Create motion magic hold position command
   * 
   * @param position
   *          double supplier that provides the target distance value
   * @return continuous command that runs climber motors
   */
  public Command getHoldPositionCommand(DoubleSupplier position)
  {
    return getMMPositionCommand(position, true).withName(kSubsystemName + "MMHoldPosition");
  }
}
