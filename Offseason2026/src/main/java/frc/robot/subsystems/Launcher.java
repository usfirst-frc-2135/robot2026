//
// Launcher Subystem - scores Notes into the Hub
//
package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Ports;
import frc.robot.lib.math.Conversions;
import frc.robot.lib.phoenix.CTREConfigs6;
import frc.robot.lib.phoenix.PhoenixUtil6;

/****************************************************************************
 * 
 * Launcher subsystem to control the launcher flywheel mechanisms and provide command factories
 */
public class Launcher extends SubsystemBase
{
  // Constants
  private static final String kSubsystemName     = "Launcher";

  private static final double kMOI               = 0.005;     // Simulation - Moment of Inertia
  private static final double kLauncherAutoRPM   = 3170.0;    // RPM to score for autonomous
  private static final double kLauncherTeleopRPM = 3150.0;    // RPM to score for autonomous
  private static final double kLauncherPassRPM   = 3300.0;    // RPM to pass 
  private static final double kLauncherPrimedRPM = 2400.0;    // RPM to pass 
  private static final double kRPMStepSize       = 50.0;      // RPM step size for increment/decrement
  private static final double kToleranceRPM      = 50.0;      // Tolerance band around requested RPM

  private static final double kLauncherGearRatio = (24.0 / 24.0);
  private static final double kHoodGearRatio     = (60.0 / 24.0) * (36.0 / 18.);

  /** Launcher (RPM) modes */
  private enum LauncherMode
  {
    STOP,       // Launcher is stopped
    SCORE,      // Launcher RPM needed to score fuel
    PASS,        // Launcher RPM needed for passing
    PRIMED
  }

  private static final double  kHoodLengthMeters = 0.10;  // Simulation
  private static final double  kHoodWeightKg     = 1.0;    // Simulation
  private static final Voltage kHoodManualVolts  = Volts.of(1.5); // Motor voltage during manual operation (joystick)

  /** Hood manual move parameters */

  private enum HoodMode
  {
    INIT,    // Initialize Hood
    INBOARD, // Hood moving into the robot
    STOPPED, // Hood stop and hold position
    OUTBOARD // Hood moving into the robot
  }

  /** Hood Motion Magic movement parameters */

  private static final double                 kToleranceDegrees      = 0.5;      // PID tolerance in inches
  private static final double                 kMMDebounceTime        = 0.060;    // Seconds to debounce a final position check
  private static final double                 kMMMoveTimeout         = 1.5;      // Seconds allowed for a Motion Magic movement

  // Devices  objects
  private final TalonFX                       m_leftMotorLower       = new TalonFX(Ports.kCANID_LauncherLeftLw);
  private final TalonFX                       m_leftMotorUpper       = new TalonFX(Ports.kCANID_LauncherLeftUp);
  private final TalonFX                       m_rightMotor           = new TalonFX(Ports.kCANID_LauncherRight);
  private final TalonFX                       m_hoodMotor            = new TalonFX(Ports.kCANID_LauncherHood);

  // Hood Angles - Motion Magic move parameters    
  private static final double                 kHoodAngleStowed       = 0.0;
  private static final double                 kHoodAngleDeployed     = 30.0;

  private static final double                 kHoodAngleMin          = kHoodAngleStowed - 0.5;
  private static final double                 kHoodAngleMax          = kHoodAngleDeployed + 0.5;

  // Alerts
  private final Alert                         m_leftLowerAlert       =
      new Alert(String.format("%s: Left motor Lower init failed!", getSubsystem( )), AlertType.kError);
  private final Alert                         m_leftUpperAlert       =
      new Alert(String.format("%s: Left motor Upper init failed!", getSubsystem( )), AlertType.kError);
  private final Alert                         m_rightAlert           =
      new Alert(String.format("%s: Right motor init failed!", getSubsystem( )), AlertType.kError);
  private final Alert                         m_hoodAlert            =
      new Alert(String.format("%s: Hood motor init failed!", getSubsystem( )), AlertType.kError);

  // Simulation objects
  private final TalonFXSimState               m_leftMotorLowerSim    = m_leftMotorLower.getSimState( );
  private final TalonFXSimState               m_hoodMotorSim         = m_hoodMotor.getSimState( );

  private final FlywheelSim                   m_leftFlywheelSim      = new FlywheelSim(
      LinearSystemId.createFlywheelSystem(DCMotor.getKrakenX60Foc(1), kMOI, kLauncherGearRatio), DCMotor.getKrakenX60Foc(1), 0.0);
  private final SingleJointedArmSim           m_armSim               =
      new SingleJointedArmSim(DCMotor.getKrakenX60(1), kHoodGearRatio,
          SingleJointedArmSim.estimateMOI(kHoodLengthMeters, kHoodWeightKg), kHoodLengthMeters, -Math.PI, Math.PI, false, 0.0);

  // Mechanism2d
  private final Mechanism2d                   m_hoodMech             = new Mechanism2d(1.0, 1.0);
  private final MechanismLigament2d           m_mechLigament         = m_hoodMech.getRoot("Hood", 0.5, 0.5)
      .append(new MechanismLigament2d(kSubsystemName, 0.5, 0.0, 6, new Color8Bit(Color.kPurple)));

  // CTRE Status signals for sensors
  private final StatusSignal<AngularVelocity> m_leftVelocity;   // Default 4Hz (250ms)
  private final StatusSignal<Angle>           m_hoodAngle;      // Default 4Hz (250ms)

  // Declare module variables
  private boolean                             m_launcherLLowerValid;
  private boolean                             m_launcherLUpperValid;
  private boolean                             m_launcherRightValid;

  private boolean                             m_hoodMotorValid;

  private boolean                             m_isAtRequestedRPM     = false; // Indicates launcher RPM is close to requested
  private boolean                             m_isAtRequestedRPMPrev = false;

  private double                              m_leftRPM;              // Current left motor RPM (before gear ratio)
  private double                              m_motorRPM;             // Requested motor RPM (before gear ratio)
  private double                              m_launcherRPM;          // Requested final launcher RPM 

  private double                              m_currentDegrees       = 0.0; // Current hood angle
  private double                              m_targetDegrees        = 0.0;  // Desired hood angle

  private VelocityVoltage                     m_requestVelocity      = new VelocityVoltage(0.0).withEnableFOC(true);
  private VoltageOut                          m_requestVolts         = new VoltageOut(0.0);
  private LinearFilter                        m_leftVelocityFilter   = LinearFilter.singlePoleIIR(0.080, 0.020);

  // Manual mode config parameters
  private VoltageOut                          m_hoodRequestVolts     = new VoltageOut(Volts.of(0)).withEnableFOC(false);
  private HoodMode                            m_hoodMode             = HoodMode.INIT;    // Manual movement mode with joysticks

  // Motion Magic config parameters    // Manual movement mode with joysticks
  private MotionMagicVoltage                  m_mmRequestVolts       = new MotionMagicVoltage(0).withSlot(0).withEnableFOC(false);
  private Debouncer                           m_mmWithinTolerance    = new Debouncer(kMMDebounceTime, DebounceType.kRising);
  private Timer                               m_mmMoveTimer          = new Timer( );       // Movement timer
  private boolean                             m_mmMoveIsFinished;                         // Movement has completed (within tolerance)

  // Network tables publisher objects
  private DoublePublisher                     m_motorRPMPub;
  private DoublePublisher                     m_launcherRPMPub;
  private BooleanPublisher                    m_atRequestedRPMPub;
  private DoubleEntry                         m_ScoreRPMEntry;

  // Network tables publisher objects
  private DoublePublisher                     m_hoodAnglePub;
  private DoublePublisher                     m_targetDegreesPub;

  /****************************************************************************
   * 
   * Constructor
   */
  public Launcher( )
  {
    setName("Launcher");
    setSubsystem("Launcher");

    m_launcherLLowerValid = PhoenixUtil6.getInstance( ).talonFXInitialize6(m_leftMotorLower, kSubsystemName + "Left_Lw",
        CTREConfigs6.launcherFXConfig(true));
    m_launcherLUpperValid = PhoenixUtil6.getInstance( ).talonFXInitialize6(m_leftMotorUpper, kSubsystemName + "Left_Up",
        CTREConfigs6.launcherFXConfig(true));
    m_launcherRightValid = PhoenixUtil6.getInstance( ).talonFXInitialize6(m_rightMotor, kSubsystemName + "Right",
        CTREConfigs6.launcherFXConfig(false));

    m_leftMotorUpper.setControl(new Follower(m_leftMotorLower.getDeviceID( ), MotorAlignmentValue.Aligned));
    m_rightMotor.setControl(new Follower(m_leftMotorLower.getDeviceID( ), MotorAlignmentValue.Opposed));

    m_launcherLLowerValid = m_launcherLLowerValid && m_launcherLUpperValid && m_launcherRightValid;

    m_hoodMotorValid = PhoenixUtil6.getInstance( ).talonFXInitialize6(m_hoodMotor, kSubsystemName + "Hood",
        CTREConfigs6.hoodFXConfig(kHoodGearRatio));

    m_leftLowerAlert.set(!m_launcherLLowerValid);
    m_leftUpperAlert.set(!m_launcherLUpperValid);
    m_rightAlert.set(!m_launcherRightValid);
    m_hoodAlert.set(!m_hoodMotorValid);

    // Simulation object initialization
    m_hoodMotorSim.Orientation = ChassisReference.CounterClockwise_Positive;

    // Initialize status signal objects
    m_leftVelocity = m_leftMotorLower.getRotorVelocity( );
    m_hoodAngle = m_hoodMotor.getPosition( );

    StatusSignal<Current> m_leftSupplyCur = m_leftMotorLower.getSupplyCurrent( ); // Default 4Hz (250ms)
    BaseStatusSignal.setUpdateFrequencyForAll(50, m_leftVelocity);

    DataLogManager.log(String.format("%s: Update (Hz) leftVelocity: %.1f leftSupplyCur: %.1f", getSubsystem( ),
        m_leftVelocity.getAppliedUpdateFrequency( ), m_leftSupplyCur.getAppliedUpdateFrequency( )));

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

    if (m_hoodMotorValid)
    {
      BaseStatusSignal.refreshAll(m_hoodAngle);
    }
    m_currentDegrees = (m_hoodMotorValid) ? m_hoodAngle.getValue( ).in(Degrees) : 0.0;

    // Update network table publishers
    if (m_launcherLLowerValid)
    {
      // Calculate Motor RPM and update network tables publishers
      BaseStatusSignal.refreshAll(m_leftVelocity);
      m_leftRPM = m_leftVelocityFilter.calculate((m_leftVelocity.getValue( ).in(RotationsPerSecond) * 60.0));

      m_isAtRequestedRPM = (m_leftRPM > kToleranceRPM) && MathUtil.isNear(m_motorRPM, m_leftRPM, kToleranceRPM);
      m_atRequestedRPMPub.set(m_isAtRequestedRPM);

      if (m_isAtRequestedRPM != m_isAtRequestedRPMPrev)
      {
        DataLogManager
            .log(String.format("%s: At requested RPM now: %.1f (motor: %.1f)", getSubsystem( ), m_launcherRPM, m_motorRPM));
        m_isAtRequestedRPMPrev = m_isAtRequestedRPM;
      }
    }

    // Update network table publishers

    m_hoodAnglePub.set(m_currentDegrees);
    m_targetDegreesPub.set(m_targetDegrees);
  }

  /****************************************************************************
   * 
   * Periodic actions that run every scheduler loop time (20 msec) during simulation
   */
  @Override
  public void simulationPeriodic( )
  {
    // This method will be called once per scheduler run during simulation

    // Set input flywheel voltage from the motor setting
    m_leftMotorLowerSim.setSupplyVoltage(RobotController.getInputVoltage( ));
    m_leftFlywheelSim.setInput(m_leftMotorLowerSim.getMotorVoltage( ));
    m_hoodMotorSim.setSupplyVoltage(RobotController.getInputVoltage( ));
    m_armSim.setInputVoltage(m_hoodMotorSim.getMotorVoltage( ));

    // update for 20 msec loop
    m_leftFlywheelSim.update(0.020);
    m_armSim.update(0.020);

    // Finally, we set our simulated encoder's readings and simulated battery voltage
    m_leftMotorLowerSim.setRotorVelocity(m_leftFlywheelSim.getAngularVelocityRPM( ) / 60.0);

    m_hoodMotorSim.setRawRotorPosition(Conversions.radiansToInputRotations(m_armSim.getAngleRads( ), kHoodGearRatio));
    m_hoodMotorSim.setRotorVelocity(Conversions.radiansToInputRotations(m_armSim.getVelocityRadPerSec( ), kHoodGearRatio));

    // SimBattery estimates loaded battery voltages
    RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(m_leftFlywheelSim.getCurrentDrawAmps( )));
    // RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(m_armSim.getCurrentDrawAmps( )));

    m_mechLigament.setAngle(m_currentDegrees);
  }

  /****************************************************************************
   * 
   * Initialize dashboard widgets
   */
  private void initDashboard( )
  {
    // Get the default instance of NetworkTables that was created automatically when the robot program starts
    NetworkTableInstance inst = NetworkTableInstance.getDefault( );
    NetworkTable table = inst.getTable("launcher");

    // Initialize network tables publishers
    m_hoodAnglePub = table.getDoubleTopic("hoodDegrees").publish( );
    m_targetDegreesPub = table.getDoubleTopic("targetDegrees").publish( );

    SmartDashboard.putData("LNHoodMech", m_hoodMech);

    // Initialize network tables publishers
    m_atRequestedRPMPub = table.getBooleanTopic("atRequestedRPM").publish( );
    m_motorRPMPub = table.getDoubleTopic("motorRPM").publish( );
    m_launcherRPMPub = table.getDoubleTopic("launcherRPM").publish( );
    m_ScoreRPMEntry = table.getDoubleTopic("scoreRPM").getEntry(0.0);
    m_ScoreRPMEntry.set(kLauncherAutoRPM);

    // Add commands
    SmartDashboard.putData("LauncherScore", getLauncherScoreCommand( ));
    SmartDashboard.putData("LauncherPass", getLauncherPassCommand( ));
    SmartDashboard.putData("LauncherStop", getLauncherStopCommand( ));
    SmartDashboard.putData("HoodDeploy", getMoveToAngleCommand(this::getDeployedAngle));
    SmartDashboard.putData("HoodStow", getMoveToAngleCommand(this::getStowedAngle));
  }

  // Put methods for controlling this subsystem here. Call these from Commands.

  /****************************************************************************
   * 
   * Initialize subsystem during robot mode changes
   */
  public void initialize( )
  {
    DataLogManager.log(String.format("%s: Subsystem initialized!", getSubsystem( )));
    setLauncherMode(LauncherMode.STOP);
  }

  /****************************************************************************
   * 
   * Write out hardware faults and reset sticky faults
   */
  public void printFaults( )
  {
    PhoenixUtil6.getInstance( ).talonFXPrintFaults(m_leftMotorLower, "LauncherLeftLw");
    PhoenixUtil6.getInstance( ).talonFXPrintFaults(m_leftMotorUpper, "LauncherLeftUp");
    PhoenixUtil6.getInstance( ).talonFXPrintFaults(m_rightMotor, "LauncherRight");
    PhoenixUtil6.getInstance( ).talonFXPrintFaults(m_hoodMotor, "LauncherHood");

    m_leftMotorLower.clearStickyFaults( );
    m_leftMotorUpper.clearStickyFaults( );
    m_rightMotor.clearStickyFaults( );
    m_hoodMotor.clearStickyFaults( );
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
  private void moveHoodWithJoystick(DoubleSupplier getAxis)
  {
    double axisValue = getAxis.getAsDouble( );
    boolean rangeLimited = false;
    HoodMode newMode = HoodMode.INIT;

    axisValue = MathUtil.applyDeadband(axisValue, Constants.kStickDeadband);

    if ((axisValue < 0.0) && (m_currentDegrees > kHoodAngleMin))
    {
      newMode = HoodMode.INBOARD;
    }
    else if ((axisValue > 0.0) && (m_currentDegrees < kHoodAngleMax))
    {
      newMode = HoodMode.OUTBOARD;
    }
    else
    {
      rangeLimited = true;
      axisValue = 0.0;
    }

    DataLogManager.log(String.format("%s: Manual move %.2f %s", getSubsystem( ), axisValue, kHoodManualVolts.times(axisValue)));

    if (newMode != m_hoodMode)
    {
      m_hoodMode = newMode;
      DataLogManager.log(String.format("%s: Manual move mode %s %.1f deg %s", getSubsystem( ), m_hoodMode, getCurrentAngle( ),
          ((rangeLimited) ? " - RANGE LIMITED" : "")));
    }

    m_targetDegrees = m_currentDegrees;

    if (m_hoodMotorValid)
    {
      // m_hoodMotor.setControl(m_hoodRequestVolts.withOutput(
      //     kHoodManualVolts.times(axisValue).plus(kManualKG.times(Math.cos(Units.degreesToRadians(m_currentDegrees))))));
      m_hoodMotor.setControl(m_hoodRequestVolts.withOutput(kHoodManualVolts.times(axisValue)));
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
   * @param newAngle
   *          rotation to move
   * @param holdAngle
   *          hold previous angle if true
   */
  public void moveToAngleInit(double newAngle, boolean holdAngle)
  {
    m_mmMoveTimer.restart( );

    if (holdAngle)
    {
      newAngle = getCurrentAngle( );
    }

    // Decide if a new angle request
    if (holdAngle || newAngle != m_targetDegrees || !MathUtil.isNear(newAngle, m_currentDegrees, kToleranceDegrees))
    {
      // Validate the angle request
      if (isMoveValid(newAngle))
      {
        m_targetDegrees = newAngle;
        m_mmMoveIsFinished = false;
        m_mmWithinTolerance.calculate(false); // Reset the debounce filter

        double targetRotations = Units.degreesToRotations(m_targetDegrees);
        if (m_hoodMotorValid)
        {
          m_hoodMotor.setControl(m_mmRequestVolts.withPosition(targetRotations));
        }
        DataLogManager.log(String.format("%s: MM Angle move: %.1f -> %.1f degrees (%.3f -> %.3f rot)", getSubsystem( ),
            m_currentDegrees, m_targetDegrees, Units.degreesToRotations(m_currentDegrees), targetRotations));
      }
      else
      {
        DataLogManager.log(String.format("%s: MM Angle move target %.1f degrees is OUT OF RANGE! [%.1f, %.1f deg]",
            getSubsystem( ), m_targetDegrees, kHoodAngleMin, kHoodAngleMax));
      }
    }
    else
    {
      m_mmMoveIsFinished = true;
      DataLogManager.log(String.format("%s: MM Angle already achieved -target %s degrees", getSubsystem( ), m_targetDegrees));
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
    double error = m_targetDegrees - m_currentDegrees;

    if (m_hoodMotorValid)
    {
      m_hoodMotor.setControl(m_mmRequestVolts.withPosition(Units.degreesToRotations(m_targetDegrees)));
    }

    if (holdAngle)
      return false;

    if (m_mmWithinTolerance.calculate(Math.abs(error) < kToleranceDegrees) || timedOut)
    {
      if (!m_mmMoveIsFinished)
        DataLogManager
            .log(String.format("%s: MM Angle move finished - Current degrees: %.1f (difference %.1f) - Time: %.3f sec %s",
                getSubsystem( ), m_currentDegrees, error, m_mmMoveTimer.get( ), (timedOut) ? "- Warning: TIMED OUT!" : ""));

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

  /****************************************************************************
   * 
   * Validate requested move
   * 
   * @param inches
   *          angle requested
   * @return true if move is within range
   */
  private boolean isMoveValid(double inches)
  {
    return (inches >= kHoodAngleMin) && (inches <= kHoodAngleMax);
  }

  ////////////////////////////////////////////////////////////////////////////
  ///////////////////////// PRIVATE HELPERS //////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////

  /****************************************************************************
   * 
   * Set launcher RPM based on requested mode
   * 
   * @param mode
   *          requested RPM
   */
  private void setLauncherMode(LauncherMode mode)
  {
    DataLogManager.log(String.format("%s: Set launcher mode to %s", getSubsystem( ), mode));

    // Select the launcher RPM for the requested mode - NEVER NEGATIVE when running!
    switch (mode)
    {
      default :
        DataLogManager.log(String.format("%s: Launcher mode is invalid: %s", getSubsystem( ), mode));
      case STOP :
        m_launcherRPM = 0.0;
        break;
      case SCORE :
        m_launcherRPM = m_ScoreRPMEntry.get(0.0);
        break;
      case PASS :
        m_launcherRPM = kLauncherPassRPM;
        break;
      case PRIMED :
        m_launcherRPM = kLauncherPrimedRPM;
        break;
    }

    m_launcherRPMPub.set(m_launcherRPM);

    m_motorRPM = Conversions.rotationsToInputRotations(m_launcherRPM, kLauncherGearRatio);
    m_motorRPMPub.set(m_motorRPM);

    double rotPerSecond = m_motorRPM / 60.0;
    if (m_launcherLLowerValid)
    {
      if (m_motorRPM > 100.0)            // Only allow positive velocities
        setLauncherVelocity(rotPerSecond);
      else                                  // When set to near zero, let motors coast
        setLauncherStopped( );
    }

    DataLogManager.log(String.format("%s: Launcher rpm is %.1f motor %.1f", getSubsystem( ), m_launcherRPM, m_motorRPM));
  }

  /****************************************************************************
   * 
   * Set launcher motors to requested velocity
   * 
   * @param rps
   *          rotations per second
   */
  private void setLauncherVelocity(double rps)
  {
    m_leftMotorLower.setControl(m_requestVelocity.withVelocity(rps));
  }

  /****************************************************************************
   * 
   * Set launcher motors to stopped
   */
  private void setLauncherStopped( )
  {
    m_leftMotorLower.setControl(m_requestVolts.withOutput(0.0));
  }

  ////////////////////////////////////////////////////////////////////////////
  ///////////////////////// PUBLIC HELPERS ///////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////

  /****************************************************************************
   *
   * Return launcher check against requested RPM
   * 
   * @return true if launcher is at requested RPM
   */
  public boolean isAtRequestedRPM( )
  {
    return m_isAtRequestedRPM;
  }

  /****************************************************************************
   * 
   * Initialize widget with launcher RPM for autonomous
   */
  public void initAutonomousRPM( )
  {
    m_ScoreRPMEntry.set(kLauncherAutoRPM);
  }

  /****************************************************************************
   * 
   * Initialize widget with launcher RPM for teleop
   */
  public void initTeleopRPM( )
  {
    m_ScoreRPMEntry.set(kLauncherTeleopRPM);
  }

  /****************************************************************************
   * 
   * Decrement launcher RPM by a fixed step size
   */
  public void decrementTeleopRPM( )
  {
    m_ScoreRPMEntry.set(m_ScoreRPMEntry.get( ) - kRPMStepSize);
  }

  /****************************************************************************
   * 
   * Increment launcher RPM by a fixed step size
   */
  public void incrementTeleopRPM( )
  {
    m_ScoreRPMEntry.set(m_ScoreRPMEntry.get( ) + kRPMStepSize);
  }

  /****************************************************************************
   * 
   * Return current angle
   * 
   * @return current hood angle
   */
  public double getCurrentAngle( )
  {
    return m_currentDegrees;
  }

  /****************************************************************************
   * 
   * Return hood angle for stowed state - fully retracted into stowed position
   * 
   * @return stowed state angle
   */
  public double getStowedAngle( )
  {
    return kHoodAngleStowed;
  }

  /****************************************************************************
   * 
   * Return hood angle for deployed state - ready for intaking fuel
   * 
   * @return deployed state angle
   */
  public double getDeployedAngle( )
  {
    return kHoodAngleDeployed;
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
        ( ) -> moveHoodWithJoystick(axis),    // Lambda method to call
        this                                  // Subsystem required
    ).withName(kSubsystemName + "MoveWithJoystick");
  }

  /****************************************************************************
   * 
   * Create motion magic base command
   * 
   * @param mode
   *          hood mode to apply
   * @param angle
   *          double supplier that provides the target angle
   * @param holdAngle
   *          boolen to indicate whether the command ever finishes
   * @return continuous command that runs intake motors
   */
  private Command getMMAngleCommand(DoubleSupplier angle, boolean holdAngle)
  {
    return new FunctionalCommand(                                       // Command with all phases declared
        ( ) -> moveToAngleInit(angle.getAsDouble( ), holdAngle),  // Init method
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
   *          double supplier that provides the target distance value
   * @return continuous command that runs intake motors
   */
  public Command getMoveToAngleCommand(DoubleSupplier angle)
  {
    return getMMAngleCommand(angle, false).withName(kSubsystemName + "MMMoveToAngle");
  }

  /****************************************************************************
   * 
   * Create motion magic hold angle command
   * 
   * @param mode
   *          roller mode to apply
   * @param angle
   *          double supplier that provides the target distance value
   * @return continuous command that runs intake motors
   */
  public Command getHoldAngleCommand(DoubleSupplier angle)
  {
    return getMMAngleCommand(angle, true).withName(kSubsystemName + "MMHoldAngle");
  }

  /****************************************************************************
   * 
   * Create launcher command based on passed mode
   * 
   * @param mode
   *          launcher mode that detemines RPM
   * @return instant command that changes launcher motors
   */
  private Command getLauncherCommand(LauncherMode mode)
  {
    return new InstantCommand(        // Command that runs exactly once
        ( ) -> setLauncherMode(mode),  // Method to call
        this                          // Subsystem requirement
    );
  }

  /****************************************************************************
   * 
   * Create launcher mode command for passing
   * 
   * @return instant command that runs launcher motors for scoring
   */
  public Command getLauncherPassCommand( )
  {
    return getLauncherCommand(LauncherMode.PASS).withName("LauncherPass");
  }

  /****************************************************************************
   * 
   * Create launcher mode command for scoring
   * 
   * @return instant command that runs launcher motors for scoring
   */
  public Command getLauncherScoreCommand( )
  {
    return getLauncherCommand(LauncherMode.SCORE).withName("LauncherScore");
  }

  /****************************************************************************
   * 
   * Create launcher mode command to stop motors
   * 
   * @return instant command that stops launcher motors
   */
  public Command getLauncherStopCommand( )
  {
    return getLauncherCommand(LauncherMode.STOP).withName("LauncherStop");
  }

  public Command getLauncherPrimedCommand( )
  {
    return getLauncherCommand(LauncherMode.PRIMED).withName("LauncherPrimed");
  }

}
