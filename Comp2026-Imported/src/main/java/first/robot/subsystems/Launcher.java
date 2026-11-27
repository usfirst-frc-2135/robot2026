//
// Launcher Subystem - scores Notes into the Hub
//
package first.robot.subsystems;

import static org.wpilib.units.Units.RotationsPerSecond;

import org.wpilib.command2.Command;
import org.wpilib.command2.InstantCommand;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.hardware.bus.CANPort;
import org.wpilib.math.filter.LinearFilter;
import org.wpilib.math.system.DCMotor;
import org.wpilib.math.system.Models;
import org.wpilib.math.util.MathUtil;
import org.wpilib.networktables.BooleanPublisher;
import org.wpilib.networktables.DoubleEntry;
import org.wpilib.networktables.DoublePublisher;
import org.wpilib.networktables.NetworkTable;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.simulation.BatterySim;
import org.wpilib.simulation.FlywheelSim;
import org.wpilib.simulation.RoboRioSim;
import org.wpilib.system.DataLogManager;
import org.wpilib.system.RobotController;
import org.wpilib.tunable.Tunables;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Current;
import org.wpilib.util.Alert;
import org.wpilib.util.Alert.Level;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import first.robot.Constants;
import first.robot.Constants.Ports;
import first.robot.lib.math.Conversions;
import first.robot.lib.phoenix.CTREConfigs;
import first.robot.lib.phoenix.PhoenixUtil;

/****************************************************************************
 * 
 * Launcher subsystem to control the launcher flywheel mechanisms and provide command factories
 */
public class Launcher extends SubsystemBase
{
  // Constants
  private static final String kSubsystemName     = "Launcher";

  private static final double kMOI               = 0.005;     // Simulation - Moment of Inertia
  private static final double kScoreRPMMin       = 0.0;       // Minimum allowed score RPM from driver adjustments
  private static final double kScoreRPMMax       = 4000.0;    // Maximum allowed score RPM from driver adjustments
  private static final double kLauncherAutoRPM   = 3170.0;    // RPM to score for autonomous
  private static final double kLauncherTeleopRPM = 3150.0;    // RPM to score for teleop
  private static final double kLauncherPassRPM   = 3300.0;    // RPM to pass 
  private static final double kLauncherPrimedRPM = 2400.0;    // RPM to pass 
  private static final double kRPMStepSize       = 50.0;      // RPM step size for increment/decrement
  private static final double kToleranceRPM      = 50.0;      // Tolerance band around requested RPM

  private static final double kLauncherGearRatio = (18.0 / 24.0);

  /** Launcher (RPM) modes */
  private enum LauncherMode
  {
    STOP,       // Launcher is stopped
    SCORE,      // Launcher RPM needed to score fuel
    PASS,        // Launcher RPM needed for passing
    PRIMED
  }

  // Devices  objects
  private final TalonFX                       m_leftMotor            =
      new TalonFX(Ports.kCANID_LauncherLeft, new CANBus(CANPort.CAN_S0));
  private final TalonFX                       m_rightMotor           =
      new TalonFX(Ports.kCANID_LauncherRight, new CANBus(CANPort.CAN_S0));

  // Alerts
  private final Alert                         m_leftAlert            =
      new Alert("launcherLeft", String.format("%s: Left motor init failed!", getName( )), Level.HIGH);
  private final Alert                         m_rightAlert           =
      new Alert("launcherRight", String.format("%s: Right motor init failed!", getName( )), Level.HIGH);

  // Simulation objects
  private final TalonFXSimState               m_leftMotorSim         = new TalonFXSimState(m_leftMotor);
  private final TalonFXSimState               m_rightMotorSim        = new TalonFXSimState(m_rightMotor);
  private final FlywheelSim                   m_leftFlywheelSim      =
      new FlywheelSim(Models.flywheelFromPhysicalConstants(DCMotor.getKrakenX60Foc(1), kMOI, kLauncherGearRatio),
          DCMotor.getKrakenX60Foc(1), 0.0);
  private final FlywheelSim                   m_rightFlywheelSim     =
      new FlywheelSim(Models.flywheelFromPhysicalConstants(DCMotor.getKrakenX60Foc(1), kMOI, kLauncherGearRatio),
          DCMotor.getKrakenX60Foc(1), 0.0);

  // CTRE Status signals for sensors
  private final StatusSignal<AngularVelocity> m_leftVelocity;   // Default 4Hz (250ms)
  private final StatusSignal<AngularVelocity> m_rightVelocity;   // Default 4Hz (250ms)

  // Declare module variables
  private boolean                             m_launcherValid;
  private boolean                             m_isAtRequestedRPM     = false; // Indicates launcher RPM is close to requested
  private boolean                             m_isAtRequestedRPMPrev = false;

  private double                              m_leftRPM;              // Current left motor RPM (before gear ratio)
  private double                              m_rightRPM;             // Current right motor RPM (before gear ratio)
  private double                              m_motorRPM;             // Requested motor RPM (before gear ratio)
  private double                              m_launcherRPM;          // Requested final launcher RPM 

  private VelocityVoltage                     m_requestVelocity      = new VelocityVoltage(0.0).withEnableFOC(true);
  private VoltageOut                          m_requestVolts         = new VoltageOut(0.0);
  private LinearFilter                        m_leftVelocityFilter   = LinearFilter.singlePoleIIR(0.080, 0.020);
  private LinearFilter                        m_rightVelocityFilter  = LinearFilter.singlePoleIIR(0.080, 0.020);

  // Network tables publisher objects
  private DoublePublisher                     m_motorRPMPub;
  private DoublePublisher                     m_launcherRPMPub;
  private BooleanPublisher                    m_atRequestedRPMPub;
  private DoubleEntry                         m_ScoreRPMEntry;

  /****************************************************************************
   * 
   * Constructor
   */
  public Launcher( )
  {
    setName("Launcher");

    boolean leftValid =
        PhoenixUtil.getInstance( ).talonFXInitialize(m_leftMotor, kSubsystemName + "Left", CTREConfigs.launcherFXConfig(false));
    boolean rightValid =
        PhoenixUtil.getInstance( ).talonFXInitialize(m_rightMotor, kSubsystemName + "Right", CTREConfigs.launcherFXConfig(true));
    //m_rightMotor.setControl(new Follower(Ports.kCANID_LauncherLeft, MotorAlignmentValue.Opposed));
    m_launcherValid = leftValid && rightValid;

    m_leftAlert.set(!leftValid);
    m_rightAlert.set(!rightValid);

    // Initialize status signal objects
    m_leftVelocity = m_leftMotor.getRotorVelocity( );
    m_rightVelocity = m_rightMotor.getRotorVelocity( );

    BaseStatusSignal.setUpdateFrequencyForAll(50, m_leftVelocity, m_rightVelocity);

    StatusSignal<Current> m_leftSupplyCur = m_leftMotor.getSupplyCurrent( ); // Default 4Hz (250ms)
    StatusSignal<Current> m_rightSupplyCur = m_rightMotor.getSupplyCurrent( ); // Default 4Hz (250ms)

    DataLogManager
        .log(String.format("%s: Update (Hz) leftVelocity: %.1f rightVelocity: %.1f leftSupplyCur: %.1f rightSupplyCur: %.1f",
            getName( ), m_leftVelocity.getAppliedUpdateFrequency( ), m_rightVelocity.getAppliedUpdateFrequency( ),
            m_leftSupplyCur.getAppliedUpdateFrequency( ), m_rightSupplyCur.getAppliedUpdateFrequency( )));

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

    // Update network table publishers
    if (m_launcherValid)
    {
      // Calculate Motor RPM and update network tables publishers
      BaseStatusSignal.refreshAll(m_leftVelocity, m_rightVelocity);
      m_leftRPM = m_leftVelocityFilter.calculate((m_leftVelocity.getValue( ).in(RotationsPerSecond) * Constants.kSecsPerMin));
      m_rightRPM = m_rightVelocityFilter.calculate((m_rightVelocity.getValue( ).in(RotationsPerSecond) * Constants.kSecsPerMin));

      m_isAtRequestedRPM = ((m_leftRPM > kToleranceRPM) && MathUtil.isNear(m_motorRPM, m_leftRPM, kToleranceRPM))
          && ((m_rightRPM > kToleranceRPM) && MathUtil.isNear(m_motorRPM, m_rightRPM, kToleranceRPM));
      m_atRequestedRPMPub.set(m_isAtRequestedRPM);

      if (m_isAtRequestedRPM != m_isAtRequestedRPMPrev)
      {
        DataLogManager.log(String.format("%s: At requested RPM now: %.1f (motor: %.1f)", getName( ), m_launcherRPM, m_motorRPM));
        m_isAtRequestedRPMPrev = m_isAtRequestedRPM;
      }
    }
    else
    {
      m_isAtRequestedRPM = false;
      m_isAtRequestedRPMPrev = false;
      m_atRequestedRPMPub.set(false);
    }
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
    m_leftMotorSim.setSupplyVoltage(RobotController.getInputVoltage( ));
    m_rightMotorSim.setSupplyVoltage(RobotController.getInputVoltage( ));
    m_leftFlywheelSim.setInput(m_leftMotorSim.getMotorVoltage( ));
    m_rightFlywheelSim.setInput(m_rightMotorSim.getMotorVoltage( ));

    // update for 20 msec loop
    m_leftFlywheelSim.update(0.020);
    m_rightFlywheelSim.update(0.020);

    // Finally, we set our simulated encoder's readings and simulated battery voltage
    m_leftMotorSim.setRotorVelocity(m_leftFlywheelSim.getAngularVelocity( ) / Constants.kSecsPerMin);
    m_rightMotorSim.setRotorVelocity(m_rightFlywheelSim.getAngularVelocity( ) / Constants.kSecsPerMin);

    // SimBattery estimates loaded battery voltage from total launcher current draw
    double totalCurrentDrawAmps = m_leftFlywheelSim.getCurrentDraw( ) + m_rightFlywheelSim.getCurrentDraw( );
    RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(totalCurrentDrawAmps));
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
    m_atRequestedRPMPub = table.getBooleanTopic("atRequestedRPM").publish( );
    m_motorRPMPub = table.getDoubleTopic("motorRPM").publish( );
    m_launcherRPMPub = table.getDoubleTopic("launcherRPM").publish( );
    m_ScoreRPMEntry = table.getDoubleTopic("scoreRPM").getEntry(0.0);
    m_ScoreRPMEntry.set(kLauncherAutoRPM);

    // Add commands
    Tunables.publish("LauncherScore", getLauncherScoreCommand( ));
    Tunables.publish("LauncherPass", getLauncherPassCommand( ));
    Tunables.publish("LauncherStop", getLauncherStopCommand( ));
  }

  // Put methods for controlling this subsystem here. Call these from Commands.

  /****************************************************************************
   * 
   * Initialize subsystem during robot mode changes
   */
  public void initialize( )
  {
    DataLogManager.log(String.format("%s: Subsystem initialized!", getName( )));
    setLauncherMode(LauncherMode.STOP);
  }

  /****************************************************************************
   * 
   * Write out hardware faults and reset sticky faults
   */
  public void printFaults( )
  {
    if (!m_launcherValid)
    {
      DataLogManager.log(String.format("%s: printFaults() skipped; launcher is invalid", getName( )));
      return;
    }

    PhoenixUtil.getInstance( ).talonFXPrintFaults(m_leftMotor, "LauncherLeft");
    PhoenixUtil.getInstance( ).talonFXPrintFaults(m_rightMotor, "LauncherRight");
    m_leftMotor.clearStickyFaults( );
    m_rightMotor.clearStickyFaults( );
  }

  /***************************************************************************
   * PRIVATE HELPERS
   ***************************************************************************/

  /****************************************************************************
   * 
   * Set launcher RPM based on requested mode
   * 
   * @param mode
   *          requested RPM
   */
  private void setLauncherMode(LauncherMode mode)
  {
    DataLogManager.log(String.format("%s: Set launcher mode to %s", getName( ), mode));

    // Select the launcher RPM for the requested mode - NEVER NEGATIVE when running!
    switch (mode)
    {
      default :
        DataLogManager.log(String.format("%s: Launcher mode is invalid: %s", getName( ), mode));
      case STOP :
        m_launcherRPM = 0.0;
        break;
      case SCORE :
        m_launcherRPM = Math.clamp(m_ScoreRPMEntry.get(0.0), kScoreRPMMin, kScoreRPMMax);
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

    double rotPerSecond = m_motorRPM / Constants.kSecsPerMin;
    if (m_launcherValid)
    {
      if (m_motorRPM > 100.0)            // Only allow positive velocities
        setLauncherVelocity(rotPerSecond);
      else                                  // When set to near zero, let motors coast
        setLauncherStopped( );
    }

    DataLogManager.log(String.format("%s: Launcher rpm is %.1f motor %.1f", getName( ), m_launcherRPM, m_motorRPM));
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
    if (!m_launcherValid)
      return;

    m_leftMotor.setControl(m_requestVelocity.withVelocity(rps));
    m_rightMotor.setControl(m_requestVelocity.withVelocity(rps));
  }

  /****************************************************************************
   * 
   * Set launcher motors to stopped
   */
  private void setLauncherStopped( )
  {
    if (!m_launcherValid)
      return;

    m_leftMotor.setControl(m_requestVolts.withOutput(0.0));
    m_rightMotor.setControl(m_requestVolts.withOutput(0.0));
  }

  /***************************************************************************
   * PUBLIC HELPERS
   ***************************************************************************/

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
    double requestedRPM = m_ScoreRPMEntry.get( ) - kRPMStepSize;
    m_ScoreRPMEntry.set(Math.clamp(requestedRPM, kScoreRPMMin, kScoreRPMMax));
  }

  /****************************************************************************
   * 
   * Increment launcher RPM by a fixed step size
   */
  public void incrementTeleopRPM( )
  {
    double requestedRPM = m_ScoreRPMEntry.get( ) + kRPMStepSize;
    m_ScoreRPMEntry.set(Math.clamp(requestedRPM, kScoreRPMMin, kScoreRPMMax));
  }

  /***************************************************************************
   * COMMAND FACTORIES
   ***************************************************************************/

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
