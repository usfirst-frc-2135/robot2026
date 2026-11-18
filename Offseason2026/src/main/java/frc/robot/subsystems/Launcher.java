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
  private static final double kLauncherAutoRPM   = 3170.0;    // RPM to score for autonomous
  private static final double kLauncherTeleopRPM = 3150.0;    // RPM to score for autonomous
  private static final double kLauncherPassRPM   = 3300.0;    // RPM to pass 
  private static final double kLauncherPrimedRPM = 2400.0;    // RPM to pass 
  private static final double kRPMStepSize       = 50.0;      // RPM step size for increment/decrement
  private static final double kToleranceRPM      = 50.0;      // Tolerance band around requested RPM
  private static final double kHoodFullDown      = -1.0;      // Hood actuator servo all the way down
  private static final double kHoodFullUp        = 1.0;       // Hood actuator servo all the way up

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

  // Devices  objects
  private final TalonFX                       m_leftMotorLower       = new TalonFX(Ports.kCANID_LauncherLeftLw);
  private final TalonFX                       m_leftMotorUpper       = new TalonFX(Ports.kCANID_LauncherLeftUp);
  private final TalonFX                       m_rightMotor           = new TalonFX(Ports.kCANID_LauncherRight);
  private final TalonFX                       m_hoodMotor            = new TalonFX(Ports.kCANID_LauncherHood);

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
  private final TalonFXSimState               m_leftMotorLowerSim    = new TalonFXSimState(m_leftMotorLower);
  // private final TalonFXSimState               m_hoodMotorSim         = new TalonFXSimState(m_hoodMotor);

  private final FlywheelSim                   m_leftFlywheelSim      = new FlywheelSim(
      LinearSystemId.createFlywheelSystem(DCMotor.getKrakenX60Foc(1), kMOI, kLauncherGearRatio), DCMotor.getKrakenX60Foc(1), 0.0);

  // CTRE Status signals for sensors
  private final StatusSignal<AngularVelocity> m_leftVelocity;   // Default 4Hz (250ms)

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

  private VelocityVoltage                     m_requestVelocity      = new VelocityVoltage(0.0).withEnableFOC(true);
  private VoltageOut                          m_requestVolts         = new VoltageOut(0.0);
  private LinearFilter                        m_leftVelocityFilter   = LinearFilter.singlePoleIIR(0.080, 0.020);

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

    // Initialize status signal objects
    m_leftVelocity = m_leftMotorLower.getRotorVelocity( );

    BaseStatusSignal.setUpdateFrequencyForAll(50, m_leftVelocity);

    StatusSignal<Current> m_leftSupplyCur = m_leftMotorLower.getSupplyCurrent( ); // Default 4Hz (250ms)

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

    // update for 20 msec loop
    m_leftFlywheelSim.update(0.020);

    // Finally, we set our simulated encoder's readings and simulated battery voltage
    m_leftMotorLowerSim.setRotorVelocity(m_leftFlywheelSim.getAngularVelocityRPM( ) / 60.0);

    // SimBattery estimates loaded battery voltages
    RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(m_leftFlywheelSim.getCurrentDrawAmps( )));
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
    SmartDashboard.putData("LauncherScore", getLauncherScoreCommand( ));
    SmartDashboard.putData("LauncherPass", getLauncherPassCommand( ));
    SmartDashboard.putData("LauncherStop", getLauncherStopCommand( ));
    SmartDashboard.putData("HoodUp", Commands.runOnce(( ) -> setHoodAngle(kHoodFullUp)));
    SmartDashboard.putData("HoodDown", Commands.runOnce(( ) -> setHoodAngle(kHoodFullDown)));
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
    setHoodAngle(kHoodFullDown);
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

  /****************************************************************************
   * 
   * Set hood actuator position
   */
  private void setHoodAngle(double position)
  {

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

  public Command getLauncherPrimedCommand( )
  {
    return getLauncherCommand(LauncherMode.PRIMED).withName("LauncherPrimed");
  }

}
