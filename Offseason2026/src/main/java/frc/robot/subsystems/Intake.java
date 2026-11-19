//
// Intake Subystem - takes in Notes and delivers them to the other subsystems
//
// The intake is composed of two motorized mechanisms: Rack and pinion and a roller roller.
// The Rack and pinion uses an the internal TalonFX encoder for measuring rotation.
//
package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
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
import edu.wpi.first.wpilibj.RobotState;
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
import frc.robot.Constants.INConsts.INRollerMode;
import frc.robot.Constants.Ports;
import frc.robot.lib.math.Conversions;
import frc.robot.lib.phoenix.CTREConfigs6;
import frc.robot.lib.phoenix.PhoenixUtil6;

/****************************************************************************
 * 
 * Intake subsystem to control the intake roller and Rack mechanism and provide command factories
 */
public class Intake extends SubsystemBase
{
  // Constants
  private static final String  kSubsystemName      = "Intake";
  private static final double  kRackGearRatio      = (40.0 / 8.0) * (50.0 / 22.0) * (18.0 / 18.0);    // Gear reduction
  private static final double  kCarriageMassKg     = 2.0;                                             // Simulation
  private static final double  kDrumDiameterInches = 1.0;                                             // Drum diameter in inches (10T 10DP Gear diameter in inches)
  private static final double  kDrumRadiusMeters   = Units.inchesToMeters(kDrumDiameterInches) / 2.0;
  private static final double  kRolloutRatio       = kDrumDiameterInches * Math.PI / kRackGearRatio;  // inches per shaft rotation

  // Intake roller constants
  private static final double  kRollerSpeedAcquire = 0.9;    // Motor direction for positive input
  private static final double  kRollerSpeedExpel   = -0.4;

  // Intake Rack constants
  private static final Voltage kRackManualVolts    = Volts.of(3.5); // Motor voltage during manual operation (joystick)

  /** Rack manual move parameters */

  private enum RackMode
  {
    INIT,    // Initialize Rack
    INBOARD, // Rack moving into the robot
    STOPPED, // Rack stop and hold position
    OUTBOARD // Rack moving into the robot
  }

  /** Rack Motion Magic movement parameters */

  private static final double       kToleranceInches    = 0.5;      // PID tolerance in inches
  private static final double       kMMDebounceTime     = 0.060;    // Seconds to debounce a final position check
  private static final double       kMMMoveTimeout      = 1.5;      // Seconds allowed for a Motion Magic movement

  // Rack lengths - Motion Magic move parameters    
  private static final double       kRackLengthStowed   = 0.0;      // Min length in inches
  private static final double       kRackLengthDeployed = 10.0;     // Max length in inches

  private static final double       kRackLengthMin      = kRackLengthStowed - 0.5;
  private static final double       kRackLengthMax      = kRackLengthDeployed + 0.5;

  // Declare device objects
  private final TalonFX             m_rollerMotorL      = new TalonFX(Ports.kCANID_IntakeRollerL);
  private final TalonFX             m_rollerMotorR      = new TalonFX(Ports.kCANID_IntakeRollerR);
  private final TalonFX             m_rackMotor         = new TalonFX(Ports.kCANID_IntakeRack);

  // Alerts
  private final Alert               m_rollerAlertL      =
      new Alert(String.format("%s: Roller motor left init failed!", getSubsystem( )), AlertType.kError);
  private final Alert               m_rollerAlertR      =
      new Alert(String.format("%s: Roller motor right init failed!", getSubsystem( )), AlertType.kError);
  private final Alert               m_rackAlert         =
      new Alert(String.format("%s: Rack motor init failed!", getSubsystem( )), AlertType.kError);

  // Simulation objects
  private final TalonFXSimState     m_rollerMotorLSim   = m_rollerMotorL.getSimState( );
  private final TalonFXSimState     m_rackMotorSim      = m_rackMotor.getSimState( );

  private final ElevatorSim         m_rackSim           = new ElevatorSim(DCMotor.getKrakenX60(2), kRackGearRatio,
      kCarriageMassKg, kDrumRadiusMeters, Units.inchesToMeters(kRackLengthMin), Units.inchesToMeters(kRackLengthMax), false, 0.0);

  // Mechanism2d
  private final Mechanism2d         m_rackMech          = new Mechanism2d(1.0, 1.0);
  private final MechanismLigament2d m_mechLigament      = m_rackMech.getRoot("Rack", 0.5, 0.5)
      .append(new MechanismLigament2d(kSubsystemName, 0.5, 0.0, 6, new Color8Bit(Color.kPurple)));

  // Status signals
  private final StatusSignal<Angle> m_rackPosition;     // Default 50Hz (20ms)

  // Declare module variables

  // Roller variables
  private boolean                   m_rollerLValid;     // Health indicator for motor 
  private boolean                   m_rollerRValid;     // Health indicator for motor

  // Rack variables
  private boolean                   m_rackValid;                // Health indicator for motor
  private double                    m_currentInches     = 0.0;  // Current length in inches
  private double                    m_targetInches      = 0.0;  // Target length in inches

  // Manual mode config parameters
  private VoltageOut                m_requestVolts      = new VoltageOut(Volts.of(0)).withEnableFOC(false);
  private RackMode                  m_rackMode          = RackMode.INIT;    // Manual movement mode with joysticks

  // Motion Magic config parameters    // Manual movement mode with joysticks
  private MotionMagicVoltage        m_mmRequestVolts    = new MotionMagicVoltage(0).withSlot(0).withEnableFOC(false);
  private Debouncer                 m_mmWithinTolerance = new Debouncer(kMMDebounceTime, DebounceType.kRising);
  private Timer                     m_mmMoveTimer       = new Timer( );       // Movement timer
  private boolean                   m_mmMoveIsFinished;                       // Movement has completed (within tolerance)

  // Network tables publisher objects
  private DoublePublisher           m_rackInchesPub;
  private DoublePublisher           m_targetInchesPub;

  /****************************************************************************
   * 
   * Constructor
   */
  public Intake( )
  {
    setName(kSubsystemName);
    setSubsystem(kSubsystemName);

    // Roller motor init
    m_rollerLValid = PhoenixUtil6.getInstance( ).talonFXInitialize6(m_rollerMotorL, kSubsystemName + "Roller_L",
        CTREConfigs6.intakeRollerFXConfig(false));
    m_rollerRValid = PhoenixUtil6.getInstance( ).talonFXInitialize6(m_rollerMotorR, kSubsystemName + "Roller_R",
        CTREConfigs6.intakeRollerFXConfig(true));
    if (m_rollerLValid && m_rollerRValid)
      m_rollerMotorR.setControl(new Follower(m_rollerMotorL.getDeviceID( ), MotorAlignmentValue.Opposed));

    // Initialize Rack motor
    m_rackValid = PhoenixUtil6.getInstance( ).talonFXInitialize6(m_rackMotor, kSubsystemName + "Rack",
        CTREConfigs6.intakeRackFXConfig(kRackGearRatio));

    m_rollerAlertL.set(!m_rollerLValid);
    m_rollerAlertR.set(!m_rollerRValid);
    m_rackAlert.set(!m_rackValid);

    // Initialize status signal objects
    m_rackPosition = m_rackMotor.getPosition( );

    // Initialize the intake status signals
    if (m_rackValid)
    {
      m_rackMotor.setPosition(Conversions.rolloutToWinchRotations(m_currentInches, kRolloutRatio));
    }

    // Simulation object initialization
    m_rackMotorSim.Orientation = ChassisReference.Clockwise_Positive;

    // Status signals
    m_rackPosition.setUpdateFrequency(50);

    StatusSignal<Current> m_rackSupplyCur = m_rackMotor.getSupplyCurrent( );                      // Default 4Hz (250ms)
    BaseStatusSignal.setUpdateFrequencyForAll(10, m_rackSupplyCur);  // Default 4Hz (250ms)

    DataLogManager.log(String.format("%s: Update (Hz) RackLength: %.1f RackSupplyCur: %.1f", getSubsystem( ),
        m_rackPosition.getAppliedUpdateFrequency( ), m_rackSupplyCur.getAppliedUpdateFrequency( )));

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

    if (m_rackValid)
    {
      BaseStatusSignal.refreshAll(m_rackPosition);
      m_currentInches =
          (m_rackValid) ? Conversions.rotationsToWinchRollout(m_rackPosition.getValue( ).in(Rotations), kRolloutRatio) : 0.0;
    }

    if (RobotState.isDisabled( ))
    {
      if (m_currentInches < 0.0)
      {
        m_currentInches = 0.0;
        m_rackMotor.setPosition(Conversions.rolloutToWinchRotations(m_currentInches, kRolloutRatio));
      }
    }

    // Update network table publishers

    m_rackInchesPub.set(m_currentInches);
    m_targetInchesPub.set(m_targetInches);
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
    m_rollerMotorLSim.setSupplyVoltage(RobotController.getInputVoltage( ));
    m_rackMotorSim.setSupplyVoltage(RobotController.getInputVoltage( ));
    m_rackSim.setInput(m_rackMotorSim.getMotorVoltage( ));

    // update for 20 msec loop
    m_rackSim.update(0.020);

    // Finally, we set our simulated encoder's readings and simulated battery voltage
    m_rackMotorSim.setRawRotorPosition(
        Conversions.rolloutToWinchRotations(Units.metersToInches(m_rackSim.getPositionMeters( )), kRolloutRatio));
    m_rackMotorSim.setRotorVelocity(
        Conversions.rolloutToWinchRotations(Units.metersToInches(m_rackSim.getVelocityMetersPerSecond( )), kRolloutRatio));

    m_rollerMotorLSim
        .setRawRotorPosition((Constants.kKrakenX60 / Constants.kSecsPerMin / Constants.kSampleFreq) * m_rollerMotorL.get( ));
    m_rollerMotorLSim.setRotorVelocity((Constants.kKrakenX60 / Constants.kSecsPerMin) * m_rollerMotorL.get( ));

    // SimBattery estimates loaded battery voltages
    RoboRioSim
        .setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(m_rackMotor.getSupplyCurrent( ).getValue( ).in(Amps)));

    m_mechLigament.setLength(0.05 + m_currentInches / kRackLengthDeployed / 2.0);
  }

  /****************************************************************************
   * 
   * Initialize dashboard widgets
   */
  private void initDashboard( )
  {
    // Get the default instance of NetworkTables that was created automatically when the robot program starts
    NetworkTableInstance inst = NetworkTableInstance.getDefault( );
    NetworkTable table = inst.getTable("intake");

    // Initialize network tables publishers
    m_rackInchesPub = table.getDoubleTopic("rackInches").publish( );
    m_targetInchesPub = table.getDoubleTopic("targetInches").publish( );

    SmartDashboard.putData("INRackMech", m_rackMech);

    // Add commands
    SmartDashboard.putData("IntakeStop", getMoveToLengthCommand(INRollerMode.STOP, this::getCurrentLength));
    SmartDashboard.putData("IntakeAcquire", getMoveToLengthCommand(INRollerMode.ACQUIRE, this::getCurrentLength));
    SmartDashboard.putData("IntakeExpel", getMoveToLengthCommand(INRollerMode.EXPEL, this::getCurrentLength));
    SmartDashboard.putData("IntakeHold", getMoveToLengthCommand(INRollerMode.HOLD, this::getCurrentLength));

    SmartDashboard.putData("IntakeDeploy", getMoveToLengthCommand(INRollerMode.HOLD, this::getDeployedLength));
    SmartDashboard.putData("IntakeRetract", getMoveToLengthCommand(INRollerMode.HOLD, this::getStowedLength));
    SmartDashboard.putData("IntakeIndexing", getIndexingCommand( ));
  }

  // Put methods for controlling this subsystem here. Call these from Commands.

  /****************************************************************************
   * 
   * Initialize subsystem during robot mode changes
   */
  public void initialize( )
  {
    setRollerMode(INRollerMode.STOP);
    setRackStopped( );

    m_targetInches = m_currentInches;
    DataLogManager.log(String.format("%s: Subsystem initialized! target inches: %.1f", getSubsystem( ), m_targetInches));
  }

  /****************************************************************************
   * 
   * Write out hardware faults and reset sticky faults
   */
  public void printFaults( )
  {
    PhoenixUtil6.getInstance( ).talonFXPrintFaults(m_rollerMotorL, kSubsystemName + "Roller L");
    PhoenixUtil6.getInstance( ).talonFXPrintFaults(m_rollerMotorR, kSubsystemName + "Roller R");
    PhoenixUtil6.getInstance( ).talonFXPrintFaults(m_rackMotor, kSubsystemName + "Rack");

    m_rollerMotorL.clearStickyFaults( );
    m_rollerMotorR.clearStickyFaults( );
    m_rackMotor.clearStickyFaults( );
  }

  ////////////////////////////////////////////////////////////////////////////
  ///////////////////////// MANUAL MOVEMENT //////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////

  // private final static Voltage kManualKG = Volts.of(-0.45);

  /****************************************************************************
   * 
   * Move motors proportional to a joystick axis value
   * 
   * @param getAxis
   *          double supplier that returns desired joystick axis
   */
  private void moveRackWithJoystick(DoubleSupplier getAxis)
  {
    double axisValue = getAxis.getAsDouble( );
    boolean rangeLimited = false;
    RackMode newMode = RackMode.INIT;

    axisValue = MathUtil.applyDeadband(axisValue, Constants.kStickDeadband);

    if ((axisValue < 0.0) && (m_currentInches > kRackLengthMin))
    {
      newMode = RackMode.INBOARD;
    }
    else if ((axisValue > 0.0) && (m_currentInches < kRackLengthMax))
    {
      newMode = RackMode.OUTBOARD;
    }
    else
    {
      rangeLimited = true;
      axisValue = 0.0;
    }

    DataLogManager.log(String.format("%s: Manual move %.2f %s", getSubsystem( ), axisValue, kRackManualVolts.times(axisValue)));

    if (newMode != m_rackMode)
    {
      m_rackMode = newMode;
      DataLogManager.log(String.format("%s: Manual move mode %s %.1f deg %s", getSubsystem( ), m_rackMode, getCurrentLength( ),
          ((rangeLimited) ? " - RANGE LIMITED" : "")));
    }

    m_targetInches = m_currentInches;

    if (m_rackValid)
    {
      m_rackMotor.setControl(m_requestVolts.withOutput(kRackManualVolts.times(axisValue)));
    }
  }

  ////////////////////////////////////////////////////////////////////////////
  ///////////////////////// MOTION MAGIC /////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////

  /****************************************************************************
   * 
   * Initialize a Motion Magic movement and control rollers
   * 
   * @param mode
   *          roller mode to apply
   * @param newLength
   *          rotation to move
   * @param holdLength
   *          hold previous length if true
   */
  public void moveToLengthInit(INRollerMode mode, double newLength, boolean holdLength)
  {
    setRollerMode(mode);
    m_mmMoveTimer.restart( );

    if (holdLength)
    {
      newLength = getCurrentLength( );
    }

    // Decide if a new length request
    if (holdLength || newLength != m_targetInches || !MathUtil.isNear(newLength, m_currentInches, kToleranceInches))
    {
      // Validate the length request
      if (isMoveValid(newLength))
      {
        m_targetInches = newLength;
        m_mmMoveIsFinished = false;
        m_mmWithinTolerance.calculate(false); // Reset the debounce filter

        double targetRotations = Conversions.rolloutToWinchRotations(newLength, kRolloutRatio);

        if (m_rackValid)
        {
          m_rackMotor.setControl(m_mmRequestVolts.withPosition(targetRotations));
        }
        DataLogManager
            .log(String.format("%s: MM Length move: %.1f -> %.1f inches (%.3f -> %.3f rot)", getSubsystem( ), m_currentInches,
                m_targetInches, Conversions.rolloutToWinchRotations(m_currentInches, kRolloutRatio), targetRotations));
      }
      else
      {
        DataLogManager.log(String.format("%s: MM Length move target %.1f inches is OUT OF RANGE! [%.1f, %.1f deg]",
            getSubsystem( ), m_targetInches, kRackLengthMin, kRackLengthMax));
      }
    }
    else
    {
      m_mmMoveIsFinished = true;
      DataLogManager.log(String.format("%s: MM Length already achieved -target %s inches", getSubsystem( ), m_targetInches));
    }
  }

  /****************************************************************************
   * 
   * Continuously update Motion Magic setpoint
   */
  public void moveToLengthExecute( )
  {}

  /****************************************************************************
   * 
   * Detect Motion Magic finished state
   * 
   * @param holdLength
   *          hold the current length
   * @return true when movement has completed
   */
  public boolean moveToLengthIsFinished(boolean holdLength)
  {
    boolean timedOut = m_mmMoveTimer.hasElapsed(kMMMoveTimeout);
    double error = m_targetInches - m_currentInches;

    if (m_rackValid)
    {
      m_rackMotor.setControl(m_mmRequestVolts.withPosition(Conversions.rolloutToWinchRotations(m_targetInches, kRolloutRatio)));
    }

    if (holdLength)
      return false;

    if (m_mmWithinTolerance.calculate(Math.abs(error) < kToleranceInches) || timedOut)
    {
      if (!m_mmMoveIsFinished)
        DataLogManager
            .log(String.format("%s: MM Length move finished - Current inches: %.1f (difference %.1f) - Time: %.3f sec %s",
                getSubsystem( ), m_currentInches, error, m_mmMoveTimer.get( ), (timedOut) ? "- Warning: TIMED OUT!" : ""));

      m_mmMoveIsFinished = true;
    }

    return m_mmMoveIsFinished;
  }

  /****************************************************************************
   * 
   * Wrap up a Motion Magic movement
   */
  public void moveToLengthEnd( )
  {
    m_mmMoveTimer.stop( );
  }

  ////////////////////////////////////////////////////////////////////////////
  ///////////////////////// PRIVATE HELPERS //////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////

  /****************************************************************************
   * 
   * Set roller speed based on requested mode
   * 
   * @param mode
   *          requested speed
   */
  private void setRollerMode(INRollerMode mode)
  {
    double output = 0.0;

    if (mode == INRollerMode.HOLD)
    {
      DataLogManager.log(String.format("%s: Roller mode is unchanged - %s (%.3f)", getSubsystem( ), mode, m_rollerMotorL.get( )));
    }
    else
    {
      switch (mode)
      {
        default :
          DataLogManager.log(String.format("%s: Roller mode is invalid: %s", getSubsystem( ), mode));
        case STOP :
          output = 0.0;
          break;
        case ACQUIRE :
          output = kRollerSpeedAcquire;
          break;
        case EXPEL :
          output = kRollerSpeedExpel;
          break;
      }
      DataLogManager.log(String.format("%s: Roller mode is now - %s", getSubsystem( ), mode));
      m_rollerMotorL.set(output);
    }
  }

  /****************************************************************************
   * 
   * Set Rack motor to stopped
   */
  private void setRackStopped( )
  {
    DataLogManager.log(String.format("%s: Rack motor now STOPPED", getSubsystem( )));
    if (m_rackValid)
    {
      m_rackMotor.setControl(m_requestVolts.withOutput(0.0));
    }
  }

  /****************************************************************************
   * 
   * Validate requested move
   * 
   * @param inches
   *          length requested
   * @return true if move is within range
   */
  private boolean isMoveValid(double inches)
  {
    return (inches >= kRackLengthMin) && (inches <= kRackLengthMax);
  }

  ////////////////////////////////////////////////////////////////////////////
  ///////////////////////// PUBLIC HELPERS ///////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////

  /****************************************************************************
   * 
   * Return current length
   * 
   * @return current Rack length
   */
  public double getCurrentLength( )
  {
    return m_currentInches;
  }

  /****************************************************************************
   * 
   * Return intake length for stowed state - fully retracted into stowed position
   * 
   * @return stowed state length
   */
  public double getStowedLength( )
  {
    return kRackLengthStowed;
  }

  /****************************************************************************
   * 
   * Return intake length for deployed state - ready for intaking fuel
   * 
   * @return deployed state length
   */
  public double getDeployedLength( )
  {
    return kRackLengthDeployed;
  }

  ////////////////////////////////////////////////////////////////////////////
  ///////////////////////// COMMAND FACTORIES ////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////

  /****************************************************************************
   * 
   * Create joystick manual move command
   * 
   * @param axis
   *          double supplier that provides the joystick axis value
   * @return continuous command that runs Rack motor
   */
  public Command getJoystickCommand(DoubleSupplier axis)
  {
    return new RunCommand(                    // Command that runs continuously
        ( ) -> moveRackWithJoystick(axis),  // Lambda method to call
        this                                  // Subsystem required
    ).withName(kSubsystemName + "MoveWithJoystick");
  }

  /****************************************************************************
   * 
   * Create motion magic base command
   * 
   * @param mode
   *          roller mode to apply
   * @param length
   *          double supplier that provides the target length
   * @param holdLength
   *          boolen to indicate whether the command ever finishes
   * @return continuous command that runs intake motors
   */
  private Command getMMLengthCommand(INRollerMode mode, DoubleSupplier length, boolean holdLength)
  {
    return new FunctionalCommand(                                       // Command with all phases declared
        ( ) -> moveToLengthInit(mode, length.getAsDouble( ), holdLength),  // Init method
        ( ) -> moveToLengthExecute( ),                                   // Execute method
        interrupted -> moveToLengthEnd( ),                               // End method
        ( ) -> moveToLengthIsFinished(holdLength),                        // IsFinished method
        this                                                            // Subsystem required
    );
  }

  /****************************************************************************
   * 
   * Create motion magic move to length command
   * 
   * @param mode
   *          roller mode to apply
   * @param length
   *          double supplier that provides the target distance value
   * @return continuous command that runs intake motors
   */
  public Command getMoveToLengthCommand(INRollerMode mode, DoubleSupplier length)
  {
    return getMMLengthCommand(mode, length, false).withName(kSubsystemName + "MMMoveToLength");
  }

  /****************************************************************************
   * 
   * Create motion magic hold length command
   * 
   * @param mode
   *          roller mode to apply
   * @param length
   *          double supplier that provides the target distance value
   * @return continuous command that runs intake motors
   */
  public Command getHoldLengthCommand(INRollerMode mode, DoubleSupplier length)
  {
    return getMMLengthCommand(mode, length, true).withName(kSubsystemName + "MMHoldLength");
  }

  /****************************************************************************
   * 
   * Create indexing sequence for pushing fuel to launcher
   * 
   * @return command that runs moves intake arm while intaking
   */
  public Command getIndexingCommand( )
  {
    return getMoveToLengthCommand(INRollerMode.ACQUIRE, this::getCurrentLength).withName(kSubsystemName + "IndexingCommand");
  }
}
