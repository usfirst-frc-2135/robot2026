//
// Launcher Subystem - scores Notes into the Hub
//
package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
  private static final double kLauncherScoreRPM  = 3100.0;    // RPM to score 
  private static final double kLauncherPassRPM   = 3300.0;    // RPM to pass 
  private static final double kToleranceRPM      = 60.0;      // Tolerance band around requested RPM

  private static final double kLauncherGearRatio = (18.0 / 24.0);

  /** Launcher (RPM) modes */
  private enum LauncherMode
  {
    STOP,       // Launcher is stopped
    SCORE,      // Launcher RPM needed to score fuel
    PASS        // Launcher RPM needed for passing
  }

  // Devices  objects
  private final TalonFX                       m_leftMotor            = new TalonFX(Ports.kCANID_LauncherLeft);
  private final TalonFX                       m_rightMotor           = new TalonFX(Ports.kCANID_LauncherRight);
  private final Servo                         m_hoodLeft             = new Servo(0);
  private final Servo                         m_hoodRight            = new Servo(1);

  // Alerts
  private final Alert                         m_leftAlert            =
      new Alert(String.format("%s: Left motor init failed!", getSubsystem( )), AlertType.kError);
  private final Alert                         m_rightAlert           =
      new Alert(String.format("%s: Right motor init failed!", getSubsystem( )), AlertType.kError);

  // Simulation objects
  private final TalonFXSimState               m_leftMotorSim         = new TalonFXSimState(m_leftMotor);
  private final TalonFXSimState               m_rightMotorSim        = new TalonFXSimState(m_rightMotor);
  private final FlywheelSim                   m_leftFlywheelSim      = new FlywheelSim(
      LinearSystemId.createFlywheelSystem(DCMotor.getKrakenX60Foc(1), kMOI, kLauncherGearRatio), DCMotor.getKrakenX60Foc(1), 0.0);
  private final FlywheelSim                   m_rightFlywheelSim     = new FlywheelSim(
      LinearSystemId.createFlywheelSystem(DCMotor.getKrakenX60Foc(1), kMOI, kLauncherGearRatio), DCMotor.getKrakenX60Foc(1), 0.0);

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
  private LinearFilter                        m_leftVelocityFilter   = LinearFilter.singlePoleIIR(0.060, 0.020);
  private LinearFilter                        m_rightVelocityFilter  = LinearFilter.singlePoleIIR(0.060, 0.020);

  // Network tables publisher objects
  private DoublePublisher                     m_leftSpeedPub;
  private DoublePublisher                     m_rightSpeedPub;

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
    setSubsystem("Launcher");

    boolean leftValid =
        PhoenixUtil6.getInstance( ).talonFXInitialize6(m_leftMotor, kSubsystemName + "Left", CTREConfigs6.launcherFXConfig( ));
    boolean rightValid =
        PhoenixUtil6.getInstance( ).talonFXInitialize6(m_rightMotor, kSubsystemName + "Right", CTREConfigs6.launcherFXConfig( ));
    m_rightMotor.setControl(new Follower(Ports.kCANID_LauncherLeft, MotorAlignmentValue.Opposed));
    m_launcherValid = leftValid && rightValid;

    m_leftAlert.set(!leftValid);
    m_rightAlert.set(!rightValid);

    // Initialize status signal objects
    m_leftVelocity = m_leftMotor.getRotorVelocity( );
    m_rightVelocity = m_rightMotor.getRotorVelocity( );

    BaseStatusSignal.setUpdateFrequencyForAll(50, m_leftVelocity, m_rightVelocity);

    StatusSignal<Current> m_leftSupplyCur = m_leftMotor.getSupplyCurrent( ); // Default 4Hz (250ms)
    StatusSignal<Current> m_leftStatorCur = m_leftMotor.getStatorCurrent( ); // Default 4Hz (250ms)
    StatusSignal<Current> m_rightSupplyCur = m_rightMotor.getSupplyCurrent( ); // Default 4Hz (250ms)
    StatusSignal<Current> m_rightStatorCur = m_rightMotor.getStatorCurrent( ); // Default 4Hz (250ms)

    m_hoodLeft.setBoundsMicroseconds(2000, 1800, 1500, 1200, 1000);
    m_hoodRight.setBoundsMicroseconds(2000, 1800, 1500, 1200, 1000);

    DataLogManager.log(String.format(
        "%s: Update (Hz) leftVelocity: %.1f rightVelocity: %.1f leftSupplyCur: %.1f leftStatorCur: %.1f rightSupplyCur: %.1f rightStatorCur: %.1f",
        getSubsystem( ), m_leftVelocity.getAppliedUpdateFrequency( ), m_rightVelocity.getAppliedUpdateFrequency( ),
        m_leftSupplyCur.getAppliedUpdateFrequency( ), m_leftStatorCur.getAppliedUpdateFrequency( ),
        m_rightSupplyCur.getAppliedUpdateFrequency( ), m_rightStatorCur.getAppliedUpdateFrequency( )));

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
      m_leftRPM = m_leftVelocityFilter.calculate((m_leftVelocity.getValue( ).in(RotationsPerSecond) * 60.0));
      m_rightRPM = m_rightVelocityFilter.calculate((m_rightVelocity.getValue( ).in(RotationsPerSecond) * 60.0));
      m_leftSpeedPub.set(m_leftRPM);
      m_rightSpeedPub.set(m_rightRPM);

      m_isAtRequestedRPM = ((m_leftRPM > kToleranceRPM) && MathUtil.isNear(m_motorRPM, m_leftRPM, kToleranceRPM))
          && ((m_rightRPM > kToleranceRPM) && MathUtil.isNear(m_motorRPM, m_rightRPM, kToleranceRPM));
      m_atRequestedRPMPub.set(m_isAtRequestedRPM);

      if (m_isAtRequestedRPM != m_isAtRequestedRPMPrev)
      {
        DataLogManager
            .log(String.format("%s: At requested RPM now: %.1f (motor: %.1f)", getSubsystem( ), m_launcherRPM, m_motorRPM));
        m_isAtRequestedRPMPrev = m_isAtRequestedRPM;
      }
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
    m_leftMotorSim.setRotorVelocity(m_leftFlywheelSim.getAngularVelocityRPM( ) / 60.0);
    m_rightMotorSim.setRotorVelocity(m_rightFlywheelSim.getAngularVelocityRPM( ) / 60.0);

    // SimBattery estimates loaded battery voltages
    RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(m_leftFlywheelSim.getCurrentDrawAmps( )));
    RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(m_rightFlywheelSim.getCurrentDrawAmps( )));
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
    m_leftSpeedPub = table.getDoubleTopic("leftSpeed").publish( );
    m_rightSpeedPub = table.getDoubleTopic("rightSpeed").publish( );

    m_atRequestedRPMPub = table.getBooleanTopic("atRequestedRPM").publish( );
    m_motorRPMPub = table.getDoubleTopic("motorRPM").publish( );
    m_launcherRPMPub = table.getDoubleTopic("launcherRPM").publish( );
    m_ScoreRPMEntry = table.getDoubleTopic("scoreRPM").getEntry(0.0);
    m_ScoreRPMEntry.set(kLauncherScoreRPM);

    // Add commands
    SmartDashboard.putData("LauncherScore", getLauncherScoreCommand( ));
    SmartDashboard.putData("LauncherPass", getLauncherPassCommand( ));
    SmartDashboard.putData("LauncherStop", getLauncherStopCommand( ));
    SmartDashboard.putData("HoodIn", Commands.runOnce(( ) ->
    {
      m_hoodLeft.setSpeed(1.0);
      m_hoodRight.setSpeed(1.0);
    }));

    SmartDashboard.putData("HoodOut", Commands.runOnce(( ) ->
    {
      m_hoodLeft.setSpeed(-1.0);
      m_hoodRight.setSpeed(-1.0);
    }));
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
    Commands.runOnce(( ) ->
    {
      m_hoodLeft.setSpeed(-1.0);
      m_hoodRight.setSpeed(-1.0);
    });

  }

  /****************************************************************************
   * 
   * Write out hardware faults and reset sticky faults
   */
  public void printFaults( )
  {
    PhoenixUtil6.getInstance( ).talonFXPrintFaults(m_leftMotor, "LauncherLeft");
    PhoenixUtil6.getInstance( ).talonFXPrintFaults(m_rightMotor, "LauncherRight");
    m_leftMotor.clearStickyFaults( );
    m_rightMotor.clearStickyFaults( );
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
    }

    m_launcherRPMPub.set(m_launcherRPM);

    m_motorRPM = Conversions.rotationsToInputRotations(m_launcherRPM, kLauncherGearRatio);
    m_motorRPMPub.set(m_motorRPM);

    double rotPerSecond = m_motorRPM / 60.0;
    if (m_launcherValid)
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
    m_leftMotor.setControl(m_requestVelocity.withVelocity(rps));
  }

  /****************************************************************************
   * 
   * Set launcher motors to stopped
   */
  private void setLauncherStopped( )
  {
    m_leftMotor.setControl(m_requestVolts.withOutput(0.0));
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

  ////////////////////////////////////////////////////////////////////////////
  ///////////////////////// COMMAND FACTORIES ////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////

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

}
