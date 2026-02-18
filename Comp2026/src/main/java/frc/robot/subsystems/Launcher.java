//
// Launcher Subystem - scores Notes into the Speaker
//
package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
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
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
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

  private static final double kMOI               = 0.001;     // Simulation - Moment of Inertia
  private static final double kFlywheelScoreRPM  = 3300.0;    // RPM to score
  private static final double kFlywheelPassRPM   = 3000.0;    // RPM to pass
  private static final double kToleranceRPM      = 150.0;     // Tolerance band around target RPM

  private static final double kFlywheelGearRatio = (18.0 / 18.0);

  /** Launcher (speed) modes */
  private enum LauncherMode
  {
    REVERSE,    // Launcher runs in reverse direction to handle jams
    STOP,       // Launcher is stopped
    SCORE,      // Launcher ramped to an initial speed before launching
    PASS        // Launcher slowed to passing speed
  }

  // Devices  objects
  private final TalonFX                       m_leftMotor             = new TalonFX(Ports.kCANID_LauncherLeft);
  private final TalonFX                       m_rightMotor            = new TalonFX(Ports.kCANID_LauncherRight);

  // Alerts
  private final Alert                         m_leftAlert             =
      new Alert(String.format("%s: Left motor init failed!", getSubsystem( )), AlertType.kError);
  private final Alert                         m_rightAlert            =
      new Alert(String.format("%s: Right motor init failed!", getSubsystem( )), AlertType.kError);

  // Simulation objects
  private final TalonFXSimState               m_leftMotorSim          = new TalonFXSimState(m_leftMotor);
  private final TalonFXSimState               m_rightMotorSim         = new TalonFXSimState(m_rightMotor);
  private final FlywheelSim                   m_leftFlywheelSim       = new FlywheelSim(
      LinearSystemId.createFlywheelSystem(DCMotor.getKrakenX60(1), kMOI, kFlywheelGearRatio), DCMotor.getKrakenX60(1), 0.0);
  private final FlywheelSim                   m_rightFlywheelSim      = new FlywheelSim(
      LinearSystemId.createFlywheelSystem(DCMotor.getKrakenX60(1), kMOI, kFlywheelGearRatio), DCMotor.getKrakenX60(1), 0.0);

  // CTRE Status signals for sensors
  private final StatusSignal<AngularVelocity> m_leftVelocity;   // Default 4Hz (250ms)
  private final StatusSignal<AngularVelocity> m_rightVelocity;   // Default 4Hz (250ms)

  // Declare module variables
  private boolean                             m_launcherValid;
  private boolean                             m_isAttargetRPM         = false; // Indicates flywheel RPM is close to target
  private boolean                             m_isAttargetRPMPrevious = false;

  private double                              m_targetRPM;            // Requested target flywheel RPM
  private double                              m_leftRPM;             // Current left RPM
  private double                              m_rightRPM;             // Current right RPM

  private VelocityVoltage                     m_requestVelocity       = new VelocityVoltage(0.0);
  private VoltageOut                          m_requestVolts          = new VoltageOut(0.0);
  private LinearFilter                        m_leftFlywheelFilter    = LinearFilter.singlePoleIIR(0.060, 0.020);
  private LinearFilter                        m_rightFlywheelFilter   = LinearFilter.singlePoleIIR(0.060, 0.020);

  // Network tables publisher objects
  private DoublePublisher                     m_leftSpeedPub;
  private DoublePublisher                     m_rightSpeedPub;

  private BooleanPublisher                    m_atDesiredRPMPub;
  private DoublePublisher                     m_targetRPMPub;
  private DoubleEntry                         m_flywheelScoreEntry;

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
      // Calculate flywheel RPM and update network tables publishers
      BaseStatusSignal.refreshAll(m_leftVelocity, m_rightVelocity);
      m_leftRPM = m_leftFlywheelFilter.calculate((m_leftVelocity.getValue( ).in(RotationsPerSecond) * 60.0));
      m_rightRPM = m_rightFlywheelFilter.calculate((m_rightVelocity.getValue( ).in(RotationsPerSecond) * 60.0));
      m_leftSpeedPub.set(m_leftRPM);
      m_rightSpeedPub.set(m_rightRPM);

      m_isAttargetRPM = ((m_leftRPM > kToleranceRPM) && MathUtil.isNear(m_targetRPM, m_leftRPM, kToleranceRPM))
          && ((m_rightRPM > kToleranceRPM) && MathUtil.isNear(m_targetRPM, m_rightRPM, kToleranceRPM));
      m_atDesiredRPMPub.set(m_isAttargetRPM);

      if (m_isAttargetRPM != m_isAttargetRPMPrevious)
      {
        DataLogManager.log(String.format("%s: At desired speed now: %.1f", getSubsystem( ), m_targetRPM));
        m_isAttargetRPMPrevious = m_isAttargetRPM;
      }
    }

    m_targetRPMPub.set(m_targetRPM);
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

    m_atDesiredRPMPub = table.getBooleanTopic("atDesiredRPM").publish( );
    m_targetRPMPub = table.getDoubleTopic("targetRPM").publish( );
    m_flywheelScoreEntry = table.getDoubleTopic("flywheelRPM").getEntry(0.0);
    m_flywheelScoreEntry.set(kFlywheelScoreRPM);

    // Add commands
    SmartDashboard.putData("ShRunScore", getLauncherScoreCommand( ));
    SmartDashboard.putData("ShRunPass", getLauncherPassCommand( ));
    SmartDashboard.putData("ShRunStop", getLauncherStopCommand( ));
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
    PhoenixUtil6.getInstance( ).talonFXPrintFaults(m_leftMotor, "LauncherLower");
    PhoenixUtil6.getInstance( ).talonFXPrintFaults(m_rightMotor, "LauncherUpper");
    m_leftMotor.clearStickyFaults( );
    m_rightMotor.clearStickyFaults( );
  }

  ////////////////////////////////////////////////////////////////////////////
  ///////////////////////// PRIVATE HELPERS //////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////

  /****************************************************************************
   * 
   * Set launcher speed based on requested mode
   * 
   * @param mode
   *          requested speed
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
        m_targetRPM = 0.0;
        break;
      case SCORE :
        m_targetRPM = m_flywheelScoreEntry.get(0.0);
        break;
      case PASS :
        m_targetRPM = kFlywheelPassRPM;
        break;
    }

    double rotPerSecond = m_targetRPM / 60.0;
    if (m_launcherValid)
    {
      if (m_targetRPM > 100.0)
        setLauncherVelocity(rotPerSecond);
      else
        setLauncherStopped( );
    }
    DataLogManager.log(String.format("%s: Target rpm is %.1f rps %.1f", getSubsystem( ), m_targetRPM, rotPerSecond));
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
    m_leftMotor.setControl(m_requestVelocity.withVelocity(Conversions.rotationsToInputRotations(rps, kFlywheelGearRatio)));
    m_rightMotor.setControl(m_requestVelocity.withVelocity(Conversions.rotationsToInputRotations(rps, kFlywheelGearRatio)));
  }

  /****************************************************************************
   * 
   * Set launcher motors to stopped
   */
  private void setLauncherStopped( )
  {
    m_leftMotor.setControl(m_requestVolts);
    m_rightMotor.setControl(m_requestVolts);
  }

  ////////////////////////////////////////////////////////////////////////////
  ///////////////////////// PUBLIC HELPERS ///////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////

  /****************************************************************************
   * 
   * Return launcher speed check against target RPM
   * 
   * @return true if launcher is at target RPM
   */
  public boolean isAtTargetRPM( )
  {
    return m_isAttargetRPM;
  }

  ////////////////////////////////////////////////////////////////////////////
  ///////////////////////// COMMAND FACTORIES ////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////

  /****************************************************************************
   * 
   * Create launcher command based on passed mode
   * 
   * @param mode
   *          launcher mode that detemines speed
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
