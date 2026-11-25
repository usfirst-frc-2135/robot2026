//
// Intake Subsystem - takes in Notes and delivers them to the other subsystems
//
// The intake is composed of two motorized mechanisms: a rotary joint and a roller.
// The rotary joint uses an external CANcoder for measuring rotation.
//
package first.robot.subsystems;

import static org.wpilib.units.Units.Rotations;
import static org.wpilib.units.Units.Volts;

import java.util.function.DoubleSupplier;

import org.wpilib.command2.Command;
import org.wpilib.command2.FunctionalCommand;
import org.wpilib.command2.RepeatCommand;
import org.wpilib.command2.RunCommand;
import org.wpilib.command2.SequentialCommandGroup;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.command2.WaitCommand;
import org.wpilib.hardware.bus.CANPort;
import org.wpilib.math.filter.Debouncer;
import org.wpilib.math.filter.Debouncer.DebounceType;
import org.wpilib.math.system.DCMotor;
import org.wpilib.math.util.MathUtil;
import org.wpilib.math.util.Units;
import org.wpilib.networktables.DoublePublisher;
import org.wpilib.networktables.NetworkTable;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.simulation.BatterySim;
import org.wpilib.simulation.RoboRioSim;
import org.wpilib.simulation.SingleJointedArmSim;
import org.wpilib.smartdashboard.Mechanism2d;
import org.wpilib.smartdashboard.MechanismLigament2d;
import org.wpilib.system.DataLogManager;
import org.wpilib.system.RobotController;
import org.wpilib.system.Timer;
import org.wpilib.tunable.Tunables;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.Current;
import org.wpilib.units.measure.Voltage;
import org.wpilib.util.Alert;
import org.wpilib.util.Alert.Level;
import org.wpilib.util.Color;
import org.wpilib.util.Color8Bit;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;

import first.robot.Constants;
import first.robot.Robot;
import first.robot.Constants.Ports;
import first.robot.Constants.INConsts.INRollerMode;
import first.robot.lib.math.Conversions;
import first.robot.lib.phoenix.CTREConfigs6;
import first.robot.lib.phoenix.PhoenixUtil6;

/****************************************************************************
 * 
 * Intake subsystem to control the intake roller and rotary mechanism and provide command factories
 */
public class Intake extends SubsystemBase
{
  // Constants
  private static final String  kSubsystemName      = "Intake";

  // Intake roller constants
  private static final double  kRollerSpeedAcquire = 0.9;    // Motor direction for positive input
  private static final double  kRollerSpeedExpel   = -0.4;

  // Intake rotary constants
  private static final double  kRotaryGearRatio    = 36.0;   // Simulation
  private static final double  kRotaryLengthMeters = 0.239;  // Simulation
  private static final double  kRotaryWeightKg     = 3.0;    // Simulation
  private static final Voltage kRotaryManualVolts  = Volts.of(3.5); // Motor voltage during manual operation (joystick)

  /** Rotary manual move parameters */

  private enum RotaryMode
  {
    INIT,    // Initialize rotary
    INBOARD, // Rotary moving into the robot
    STOPPED, // Rotary stop and hold position
    OUTBOARD // Rotary moving into the robot
  }

  /** Rotary Motion Magic movement parameters */

  private static final double       kToleranceDegrees     = 3.0;      // PID tolerance in degrees
  private static final double       kMMDebounceTime       = 0.030;    // Seconds to debounce a final position check
  private static final double       kMMMoveTimeout        = 1.5;      // Seconds allowed for a Motion Magic movement

  // Rotary angles - Motion Magic move parameters    
  //    Measured hardstops and pre-defined positions (0 degrees is horizontal to the floor):
  //               hstop   retracted  deployed  hstop
  //      Comp     -124.7  -120.7     5.1       6.1
  //      Practice -130.4  -126.4     4.2       5.2
  private static final double       kRotaryAngleStowed    = Robot.isComp( ) ? -124.7 : -126.4; // Four degrees from hardstops
  private static final double       kRotaryAngle_75       = -75.0;
  private static final double       kRotaryAngle_45       = -45.0;
  private static final double       kRotaryAngle_30       = -30.0;
  private static final double       kRotaryAngleProtected = -15.0;
  private static final double       kRotaryAngleDeployed  = Robot.isComp( ) ? 5.1 : 4.2;       // One degree from hardstops

  private static final double       kRotaryAngleMin       = kRotaryAngleStowed - 3.0;
  private static final double       kRotaryAngleMax       = kRotaryAngleDeployed + 3.0;

  // Declare device objects
  private final TalonFX             m_rollerMotor         = new TalonFX(Ports.kCANID_IntakeRoller, new CANBus(CANPort.CAN_S0));
  private final TalonFX             m_rotaryMotor         = new TalonFX(Ports.kCANID_IntakeRotary, new CANBus(CANPort.CAN_S0));
  private final CANcoder            m_CANcoder            = new CANcoder(Ports.kCANID_IntakeCANcoder, new CANBus(CANPort.CAN_S0));

  // Alerts
  private final Alert               m_rollerAlert         =
      new Alert("intakeRoller", String.format("%s: Roller motor init failed!", getName( )), Level.HIGH);
  private final Alert               m_rotaryAlert         =
      new Alert("intakeRotary", String.format("%s: Rotary motor init failed!", getName( )), Level.HIGH);
  private final Alert               m_canCoderAlert       =
      new Alert("intakeCanCoder", String.format("%s: CANcoder init failed!", getName( )), Level.HIGH);

  // Simulation objects
  private final TalonFXSimState     m_rollerMotorSim      = m_rollerMotor.getSimState( );
  private final TalonFXSimState     m_rotarySim           = m_rotaryMotor.getSimState( );
  private final CANcoderSimState    m_CANcoderSim         = m_CANcoder.getSimState( );
  private final SingleJointedArmSim m_armSim              = new SingleJointedArmSim(DCMotor.getKrakenX60(1), kRotaryGearRatio,
      SingleJointedArmSim.estimateMOI(kRotaryLengthMeters, kRotaryWeightKg), kRotaryLengthMeters, -Math.PI, Math.PI, true, 0.0);

  // Mechanism2d
  private final Mechanism2d         m_rotaryMech          = new Mechanism2d(1.0, 1.0);
  private final MechanismLigament2d m_mechLigament        = m_rotaryMech.getRoot("Rotary", 0.5, 0.5)
      .append(new MechanismLigament2d(kSubsystemName, 0.5, 0.0, 6, new Color8Bit(Color.PURPLE)));

  // Status signals
  private final StatusSignal<Angle> m_rotaryAngle;  // Default 50Hz (20ms)
  private final StatusSignal<Angle> m_ccAngle;      // Default 100Hz (10ms)

  // Declare module variables

  // Roller variables
  private boolean                   m_rollerValid;        // Health indicator for motor 

  // Rotary variables
  private boolean                   m_rotaryValid;                // Health indicator for motor 
  private boolean                   m_canCoderValid;              // Health indicator for CANcoder 
  private double                    m_currentDegrees      = 0.0;  // Current angle in degrees
  private double                    m_goalDegrees         = 0.0;  // Goal angle in degrees
  private double                    m_ccDegrees           = 0.0;  // CANcoder angle in degrees

  // Manual mode config parameters
  private VoltageOut                m_requestVolts        = new VoltageOut(Volts.of(0)).withEnableFOC(true);
  private RotaryMode                m_rotaryMode          = RotaryMode.INIT;    // Manual movement mode with joysticks

  // Motion Magic config parameters
  private MotionMagicVoltage        m_mmRequestVolts      = new MotionMagicVoltage(0).withSlot(0).withEnableFOC(false);
  private Debouncer                 m_mmWithinTolerance   = new Debouncer(kMMDebounceTime, DebounceType.RISING);
  private Timer                     m_mmMoveTimer         = new Timer( );       // Movement timer
  private boolean                   m_mmMoveIsFinished;                         // Movement has completed (within tolerance)

  // Network tables publisher objects
  private DoublePublisher           m_rotDegreesPub;
  private DoublePublisher           m_ccDegreesPub;
  private DoublePublisher           m_goalDegreesPub;

  /****************************************************************************
   * 
   * Constructor
   */
  public Intake( )
  {
    setName(kSubsystemName);

    // Roller motor init
    m_rollerValid = PhoenixUtil6.getInstance( ).talonFXInitialize6(m_rollerMotor, kSubsystemName + "Roller",
        CTREConfigs6.intakeRollerFXConfig( ));

    // Initialize rotary motor and CANcoder objects
    m_rotaryValid = PhoenixUtil6.getInstance( ).talonFXInitialize6(m_rotaryMotor, kSubsystemName + "Rotary",
        CTREConfigs6.intakeRotaryFXConfig(Units.degreesToRotations(kRotaryAngleMin), Units.degreesToRotations(kRotaryAngleMax),
            Ports.kCANID_IntakeCANcoder, kRotaryGearRatio));
    m_canCoderValid = PhoenixUtil6.getInstance( ).canCoderInitialize6(m_CANcoder, kSubsystemName + "Rotary",
        CTREConfigs6.intakeRotaryCCConfig( ));

    m_rollerAlert.set(!m_rollerValid);
    m_rotaryAlert.set(!m_rotaryValid);
    m_canCoderAlert.set(!m_canCoderValid);

    // Initialize status signal objects
    m_rotaryAngle = m_rotaryMotor.getPosition( );
    m_ccAngle = m_CANcoder.getAbsolutePosition( );

    // Initialize the intake status signals
    Double ccRotations = (m_canCoderValid) ? m_ccAngle.refresh( ).getValue( ).in(Rotations) : 0.0;
    m_currentDegrees = Units.rotationsToDegrees(ccRotations);
    DataLogManager.log(String.format("%s: CANcoder initial degrees %.1f", getName( ), m_currentDegrees));
    if (m_rotaryValid)
      m_rotaryMotor.setPosition(ccRotations);

    // Simulation object initialization
    m_rotarySim.Orientation = ChassisReference.Clockwise_Positive;
    m_CANcoderSim.Orientation = ChassisReference.CounterClockwise_Positive;

    // Status signals
    m_rotaryAngle.setUpdateFrequency(100);
    m_ccAngle.setUpdateFrequency(200);

    StatusSignal<Current> m_rotarySupplyCur = m_rotaryMotor.getSupplyCurrent( );                      // Default 4Hz (250ms)
    BaseStatusSignal.setUpdateFrequencyForAll(10, m_rotarySupplyCur);  // Default 4Hz (250ms)

    DataLogManager.log(String.format("%s: Update (Hz) rotaryAngle: %.1f rotarySupplyCur: %.1f canCoderAngle: %.1f", getName( ),
        m_rotaryAngle.getAppliedUpdateFrequency( ), m_rotarySupplyCur.getAppliedUpdateFrequency( ),
        m_ccAngle.getAppliedUpdateFrequency( )));

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

    BaseStatusSignal.refreshAll(m_rotaryAngle, m_ccAngle);
    m_currentDegrees = Units.rotationsToDegrees((m_rotaryValid) ? m_rotaryAngle.getValue( ).in(Rotations) : 0.0);
    m_ccDegrees = Units.rotationsToDegrees((m_canCoderValid) ? m_ccAngle.getValue( ).in(Rotations) : 0.0);

    // Update network table publishers

    m_ccDegreesPub.set(m_ccDegrees);
    m_rotDegreesPub.set(m_currentDegrees);
    m_goalDegreesPub.set(m_goalDegrees);
  }

  /****************************************************************************
   * 
   * Periodic actions that run every scheduler loop time (20 msec) during simulation
   */
  @Override
  public void simulationPeriodic( )
  {
    // This method will be called once per scheduler run during simulation
    // NOTE: Intake rotary uses a physics model (arm + CANcoder), while intake
    // roller simulation is intentionally lightweight because it is a simple,
    // lightly loaded roller.

    // Set input motor voltage from the motor setting
    m_rotarySim.setSupplyVoltage(RobotController.getInputVoltage( ));
    m_CANcoderSim.setSupplyVoltage(RobotController.getInputVoltage( ));
    m_armSim.setInputVoltage(m_rotarySim.getMotorVoltage( ));
    m_rollerMotorSim.setSupplyVoltage(RobotController.getInputVoltage( ));

    // update for 20 msec loop
    m_armSim.update(0.020);

    // Finally, we set our simulated encoder's readings and simulated battery voltage
    m_rotarySim.setRawRotorPosition(Conversions.radiansToInputRotations(m_armSim.getAngle( ), kRotaryGearRatio));
    m_rotarySim.setRotorVelocity(Conversions.radiansToInputRotations(m_armSim.getVelocity( ), kRotaryGearRatio));

    m_CANcoderSim.setRawPosition(Units.radiansToRotations(m_armSim.getAngle( )));
    m_CANcoderSim.setVelocity(Units.radiansToRotations(m_armSim.getVelocity( )));

    // Roller sim intentionally mirrors commanded output to sensor state.
    m_rollerMotorSim.setRawRotorPosition(
        (Constants.kKrakenX60 / Constants.kSecsPerMin / Constants.kSampleFreq) * m_rollerMotor.getThrottle( ));
    m_rollerMotorSim.setRotorVelocity((Constants.kKrakenX60 / Constants.kSecsPerMin) * m_rollerMotor.getThrottle( ));

    // SimBattery estimates loaded battery voltages
    RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(m_armSim.getCurrentDraw( )));

    m_mechLigament.setAngle(-m_currentDegrees);
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
    m_ccDegreesPub = table.getDoubleTopic("ccDegrees").publish( );
    m_rotDegreesPub = table.getDoubleTopic("rotDegrees").publish( );
    m_goalDegreesPub = table.getDoubleTopic("targetDegrees").publish( );

    // Tunables.publish("INRotaryMech", m_rotaryMech);

    // Add commands
    Tunables.publish("IntakeStop", getMoveToAngleCommand(INRollerMode.STOP, this::getCurrentAngle));
    Tunables.publish("IntakeAcquire", getMoveToAngleCommand(INRollerMode.ACQUIRE, this::getCurrentAngle));
    Tunables.publish("IntakeExpel", getMoveToAngleCommand(INRollerMode.EXPEL, this::getCurrentAngle));
    Tunables.publish("IntakeUnchanged", getMoveToAngleCommand(INRollerMode.UNCHANGED, this::getCurrentAngle));

    Tunables.publish("IntakeDeploy", getMoveToAngleCommand(INRollerMode.UNCHANGED, this::getDeployedAngle));
    Tunables.publish("IntakeRetract", getMoveToAngleCommand(INRollerMode.UNCHANGED, this::getStowedAngle));
    Tunables.publish("IntakeIndexing", getIndexingCommand( ));
  }

  // Put methods for controlling this subsystem here. Call these from Commands.

  /****************************************************************************
   * 
   * Initialize subsystem during robot mode changes
   */
  public void initialize( )
  {
    setRollerMode(INRollerMode.STOP);
    setRotaryStopped( );

    m_goalDegrees = m_currentDegrees;
    DataLogManager.log(String.format("%s: Subsystem initialized! Goal Degrees: %.1f", getName( ), m_goalDegrees));
  }

  /****************************************************************************
   * 
   * Write out hardware faults and reset sticky faults
   */
  public void printFaults( )
  {
    if (m_rollerValid)
    {
      PhoenixUtil6.getInstance( ).talonFXPrintFaults(m_rollerMotor, kSubsystemName + "Roller");
      m_rollerMotor.clearStickyFaults( );
    }

    if (m_rotaryValid)
    {
      PhoenixUtil6.getInstance( ).talonFXPrintFaults(m_rotaryMotor, kSubsystemName + "Rotary");
      m_rotaryMotor.clearStickyFaults( );
    }

    if (m_canCoderValid)
    {
      PhoenixUtil6.getInstance( ).canCoderPrintFaults(m_CANcoder, kSubsystemName + "CANcoder");
      m_CANcoder.clearStickyFaults( );
    }
  }

  private final static Voltage kManualKG = Volts.of(-0.45);

  //////////////////////////////////////////////////////////////////////////// //////////////////////
  /// MANUAL MOVEMENT //////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////

  /****************************************************************************
   * 
   * Move motors proportional to a joystick axis value
   * 
   * @param getAxis
   *          double supplier that returns desired joystick axis
   */
  private void moveRotaryWithJoystick(DoubleSupplier getAxis)
  {
    double axisValue = getAxis.getAsDouble( );
    boolean rangeLimited = false;
    RotaryMode newMode = RotaryMode.INIT;

    axisValue = MathUtil.applyDeadband(axisValue, Constants.kStickDeadband);

    if ((axisValue < 0.0) && (m_currentDegrees > kRotaryAngleMin))
    {
      newMode = RotaryMode.INBOARD;
    }
    else if ((axisValue > 0.0) && (m_currentDegrees < kRotaryAngleMax))
    {
      newMode = RotaryMode.OUTBOARD;
    }
    else
    {
      rangeLimited = true;
      axisValue = 0.0;
    }

    DataLogManager.log(String.format("%s: Manual move %.2f %s", getName( ), axisValue, kRotaryManualVolts.times(axisValue)));

    if (newMode != m_rotaryMode)
    {
      m_rotaryMode = newMode;
      DataLogManager.log(String.format("%s: Manual move mode %s %.1f deg %s", getName( ), m_rotaryMode, getCurrentAngle( ),
          ((rangeLimited) ? " - RANGE LIMITED" : "")));
    }

    m_goalDegrees = m_currentDegrees;

    if (m_rotaryValid)
    {
      m_rotaryMotor.setControl(m_requestVolts.withOutput(
          kRotaryManualVolts.times(axisValue).plus(kManualKG.times(Math.cos(Units.degreesToRadians(m_currentDegrees))))));
    }
  }

  //////////////////////////////////////////////////////////////////////////// //////////////////////
  /// MOTION MAGIC /////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////

  /****************************************************************************
   * 
   * Initialize a Motion Magic movement and control rollers
   * 
   * @param mode
   *          roller mode to apply
   * @param newAngle
   *          rotation to move
   * @param holdAngle
   *          hold previous angle if true
   */
  public void moveToAngleInit(INRollerMode mode, double newAngle, boolean holdAngle)
  {
    setRollerMode(mode);
    m_mmMoveTimer.restart( );

    if (holdAngle)
    {
      newAngle = getCurrentAngle( );
    }

    // Decide if a new angle request
    if (holdAngle || newAngle != m_goalDegrees || !MathUtil.isNear(newAngle, m_currentDegrees, kToleranceDegrees))
    {
      // Validate the angle request
      if (isMoveValid(newAngle))
      {
        m_goalDegrees = newAngle;
        m_mmMoveIsFinished = false;
        m_mmWithinTolerance.calculate(false); // Reset the debounce filter

        double goalRotations = Units.degreesToRotations(m_goalDegrees);
        if (m_rotaryValid)
        {
          m_rotaryMotor.setControl(m_mmRequestVolts.withPosition(goalRotations));
        }
        DataLogManager.log(String.format("%s: MM Angle move: %.1f -> %.1f degrees (%.3f -> %.3f rot)", getName( ),
            m_currentDegrees, m_goalDegrees, Units.degreesToRotations(m_currentDegrees), goalRotations));
      }
      else
      {
        DataLogManager.log(String.format("%s: MM Angle move goal %.1f degrees is OUT OF RANGE! [%.1f, %.1f deg]", getName( ),
            newAngle, kRotaryAngleMin, kRotaryAngleMax));
      }
    }
    else
    {
      m_mmMoveIsFinished = true;
      DataLogManager.log(String.format("%s: MM Angle already achieved -goal %s degrees", getName( ), m_goalDegrees));
    }
  }

  /****************************************************************************
   * 
   * Continuously update Motion Magic setpoint
   */
  public void moveToAngleExecute( )
  {}

  /****************************************************************************
   * 
   * Detect Motion Magic finished state
   * 
   * @param holdAngle
   *          hold the current angle
   * @return true when movement has completed
   */
  public boolean moveToAngleIsFinished(boolean holdAngle)
  {
    boolean timedOut = m_mmMoveTimer.hasElapsed(kMMMoveTimeout);
    double error = m_goalDegrees - m_currentDegrees;

    if (m_rotaryValid)
    {
      m_rotaryMotor.setControl(m_mmRequestVolts.withPosition(Units.degreesToRotations(m_goalDegrees)));
    }

    if (holdAngle)
      return false;

    if (m_mmWithinTolerance.calculate(Math.abs(error) < kToleranceDegrees) || timedOut)
    {
      if (!m_mmMoveIsFinished)
        DataLogManager
            .log(String.format("%s: MM Angle move finished - Current degrees: %.1f (difference %.1f) - Time: %.3f sec %s",
                getName( ), m_currentDegrees, error, m_mmMoveTimer.get( ), (timedOut) ? "- Warning: TIMED OUT!" : ""));

      m_mmMoveIsFinished = true;
    }

    return m_mmMoveIsFinished;
  }

  /****************************************************************************
   * 
   * Wrap up a Motion Magic movement
   */
  public void moveToAngleEnd( )
  {
    m_mmMoveTimer.stop( );
  }

  //////////////////////////////////////////////////////////////////////////// //////////////////////
  /// PRIVATE HELPERS //////////////////////////////////
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
    if (!m_rollerValid)
    {
      DataLogManager.log(String.format("%s: setRollerMode(%s) skipped; roller is invalid", getName( ), mode));
      return;
    }

    double output = 0.0;

    if (mode == INRollerMode.UNCHANGED)
    {
      DataLogManager
          .log(String.format("%s: Roller mode is unchanged - %s (%.3f)", getName( ), mode, m_rollerMotor.getThrottle( )));
    }
    else
    {
      switch (mode)
      {
        default :
          DataLogManager.log(String.format("%s: Roller mode is invalid: %s", getName( ), mode));
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
      DataLogManager.log(String.format("%s: Roller mode is now - %s", getName( ), mode));
      m_rollerMotor.setThrottle(output);
    }
  }

  /****************************************************************************
   * 
   * Set rotary motor to stopped
   */
  private void setRotaryStopped( )
  {
    DataLogManager.log(String.format("%s: Rotary motor now STOPPED", getName( )));
    if (m_rotaryValid)
    {
      m_rotaryMotor.setControl(m_requestVolts.withOutput(Volts.of(0.0)));
    }
  }

  /****************************************************************************
   * 
   * Validate requested move
   * 
   * @param degrees
   *          angle requested
   * @return true if move is within range
   */
  private boolean isMoveValid(double degrees)
  {
    return (degrees >= kRotaryAngleMin) && (degrees <= kRotaryAngleMax);
  }

  //////////////////////////////////////////////////////////////////////////// //////////////////////
  /// PUBLIC HELPERS ///////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////

  /****************************************************************************
   * 
   * Return current angle
   * 
   * @return current rotary angle
   */
  public double getCurrentAngle( )
  {
    return m_currentDegrees;
  }

  /****************************************************************************
   * 
   * Return intake angle for stowed state - fully retracted into stowed position
   * 
   * @return stowed state angle
   */
  public double getStowedAngle( )
  {
    return kRotaryAngleStowed;
  }

  /****************************************************************************
   * 
   * Return intake angle for indexing states
   * 
   * @return indexing state angles
   */
  public double getIndexingHigh( )
  {
    return kRotaryAngle_75;
  }

  public double getIndexingMedium( )
  {
    return kRotaryAngle_45;
  }

  public double getIndexingLow( )
  {
    return kRotaryAngle_30;
  }

  /****************************************************************************
   * 
   * Return intake angle for protected state - slightly raised to protect hopper and intake
   * 
   * @return protected state angle
   */
  public double getProtectedAngle( )
  {
    return kRotaryAngleProtected;
  }

  /****************************************************************************
   * 
   * Return intake angle for deployed state - ready for intaking fuel
   * 
   * @return deployed state angle
   */
  public double getDeployedAngle( )
  {
    return kRotaryAngleDeployed;
  }

  //////////////////////////////////////////////////////////////////////////// //////////////////////
  /// COMMAND FACTORIES ////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////

  /****************************************************************************
   * 
   * Create joystick manual move command
   * 
   * @param axis
   *          double supplier that provides the joystick axis value
   * @return continuous command that runs rotary motor
   */
  public Command getJoystickCommand(DoubleSupplier axis)
  {
    return new RunCommand(                    // Command that runs continuously
        ( ) -> moveRotaryWithJoystick(axis),  // Lambda method to call
        this                                  // Subsystem required
    ).withName(kSubsystemName + "MoveWithJoystick");
  }

  /****************************************************************************
   * 
   * Create motion magic base command
   * 
   * @param mode
   *          roller mode to apply
   * @param angle
   *          double supplier that provides the goal angle
   * @param holdAngle
   *          boolean indicating whether the command should run continuously
   * @return continuous command that runs intake motors
   */
  private Command getMMAngleCommand(INRollerMode mode, DoubleSupplier angle, boolean holdAngle)
  {
    return new FunctionalCommand(                                       // Command with all phases declared
        ( ) -> moveToAngleInit(mode, angle.getAsDouble( ), holdAngle),  // Init method
        ( ) -> moveToAngleExecute( ),                                   // Execute method
        interrupted -> moveToAngleEnd( ),                               // End method
        ( ) -> moveToAngleIsFinished(holdAngle),                        // IsFinished method
        this                                                            // Subsystem required
    );
  }

  /****************************************************************************
   * 
   * Create motion magic move to angle command
   * 
   * @param mode
   *          roller mode to apply
   * @param angle
   *          double supplier that provides the goal angle value
   * @return continuous command that runs intake motors
   */
  public Command getMoveToAngleCommand(INRollerMode mode, DoubleSupplier angle)
  {
    return getMMAngleCommand(mode, angle, false).withName(kSubsystemName + "MMMoveToAngle");
  }

  /****************************************************************************
   * 
   * Create motion magic hold angle command
   * 
   * @param mode
   *          roller mode to apply
   * @param angle
   *          double supplier that provides the goal angle value
   * @return continuous command that runs intake motors
   */
  public Command getHoldAngleCommand(INRollerMode mode, DoubleSupplier angle)
  {
    return getMMAngleCommand(mode, angle, true).withName(kSubsystemName + "MMHoldAngle");
  }

  /****************************************************************************
   * 
   * Create indexing sequence for pushing fuel to launcher
   * 
   * @return command that moves the intake arm while intaking
   */
  public Command getIndexingCommand( )
  {
    return new SequentialCommandGroup( //
        getMoveToAngleCommand(INRollerMode.ACQUIRE, this::getProtectedAngle),       //
        new WaitCommand(0.15),                                              //
        getMoveToAngleCommand(INRollerMode.UNCHANGED, this::getIndexingLow),             //
        new RepeatCommand(                                                          //
            new SequentialCommandGroup(                                             //
                getMoveToAngleCommand(INRollerMode.UNCHANGED, this::getIndexingMedium),  //
                new WaitCommand(0.15),                                      //
                getMoveToAngleCommand(INRollerMode.UNCHANGED, this::getIndexingHigh),    //
                new WaitCommand(0.15)                                       //
            // getMoveToAngleCommand(INRollerMode.UNCHANGED, this::getStowedAngle)       //
            )                                                                       //
        )).withName(kSubsystemName + "IndexingCommand");
  }
}
