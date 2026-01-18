package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
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
import frc.robot.lib.math.Conversions;
import frc.robot.lib.phoenix.CTREConfigs6;
import frc.robot.lib.phoenix.PhoenixUtil6;

public class Shooter extends SubsystemBase
{
  private static final String kSubsystemName     = "Shooter";
  private static final double kMOI               = 0.0001; // Simulation - Moment of Inertia
  private static final double kFlywheelScoreRPM  = 3300.0; // RPM to score
  private static final double kFlywheelPassRPM   = 3000.0; // RPM to pass
  private static final double kToleranceRPM      = 150.0; // Tolerance band around target RPM
  private static final double kFlywheelGearRatio = (18.0 / 18.0);

  /** Shooter (speed) modes */
  private enum ShooterMode
  {
    REVERSE,    // Shooter runs in reverse direction to handle jams
    STOP,       // Shooter is stopped
    SCORE       // Shooter ramped to an initial speed before shooting
  }

  // Devices objects
  private final TalonFX                       m_lowerMotor            = new TalonFX(12);

  // Alerts
  private final Alert                         m_lowerAlert            =
      new Alert(String.format("%s: Lower motor init failed!", getSubsystem( )), AlertType.kError);

  // Simulation objects
  private final TalonFXSimState               m_lowerMotorSim         = new TalonFXSimState(m_lowerMotor);
  private final FlywheelSim                   m_lowerFlywheelSim      = new FlywheelSim(
      LinearSystemId.createFlywheelSystem(DCMotor.getFalcon500(1), kMOI, kFlywheelGearRatio), DCMotor.getFalcon500(1), 0.0);

  // CTRE Status signals for sensors
  private final StatusSignal<AngularVelocity> m_lowerVelocity; // Default 4Hz (250ms)

  // Declare module variables
  private boolean                             m_lowerValid;
  private double                              m_lowerRPM;                       // Current lower RPM
  private double                              m_targetRPM             = 0;      // Requested target flywheel RPM
  private boolean                             m_isAtTargetRPM         = false;  // Indicates flywheel RPM is close to target
  private boolean                             m_isAtTargetRPMPrevious = false;
  private VelocityVoltage                     m_requestVelocity       = new VelocityVoltage(0.0);
  private VoltageOut                          m_requestVolts          = new VoltageOut(0.0);
  private LinearFilter                        m_lowerFlywheelFilter   = LinearFilter.singlePoleIIR(0.060, 0.020);

  // Network tables publisher objects
  private DoublePublisher                     m_lowerRPMPub;
  private DoublePublisher                     m_targetRPMPub;
  private BooleanPublisher                    m_isAtTargetRPMPub;
  private DoubleEntry                         m_flywheelScoreEntry;

  /****************************************************************************
   * 
   * Constructor
   */
  public Shooter( )
  {
    setName("Shooter");
    setSubsystem("Shooter");

    m_lowerValid =
        PhoenixUtil6.getInstance( ).talonFXInitialize6(m_lowerMotor, kSubsystemName + "Lower", CTREConfigs6.shooterFXConfig( ));
    m_lowerAlert.set(!m_lowerValid);

    // Initialize status signal objects
    m_lowerVelocity = m_lowerMotor.getRotorVelocity( );

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
    if (m_lowerValid)
    {
      // Calculate flywheel RPM and update network tables publishers
      BaseStatusSignal.refreshAll(m_lowerVelocity);
      m_lowerRPM = m_lowerFlywheelFilter.calculate((m_lowerVelocity.getValue( ).in(RotationsPerSecond) * 60.0));
      m_lowerRPMPub.set(m_lowerRPM);

      m_isAtTargetRPM = ((m_lowerRPM > kToleranceRPM) && MathUtil.isNear(m_targetRPM, m_lowerRPM, kToleranceRPM));
      m_isAtTargetRPMPub.set(m_isAtTargetRPM);

      if (m_isAtTargetRPM != m_isAtTargetRPMPrevious)
      {
        DataLogManager.log(String.format("%s: At desired RPM now: %.1f", getSubsystem( ), m_targetRPM));
        m_isAtTargetRPMPrevious = m_isAtTargetRPM;
      }
    }
    DataLogManager
        .log(String.format("%s: Angular Velocity %s", getSubsystem( ), m_lowerFlywheelSim.getAngularVelocity( ).toString( )));

    m_targetRPMPub.set(m_targetRPM);
  }

  /****************************************************************************
   * 
   * Periodic actions that run every scheduler loop time (20 msec) during
   * simulation
   */
  @Override
  public void simulationPeriodic( )
  {
    // This method will be called once per scheduler run during simulation

    // Set input flywheel voltage from the motor setting
    m_lowerMotorSim.setSupplyVoltage(RobotController.getInputVoltage( ));
    m_lowerFlywheelSim.setInput(m_lowerMotorSim.getMotorVoltage( ));

    // update for 20 msec loop
    m_lowerFlywheelSim.update(0.020);

    // Finally, we set our simulated encoder's readings and simulated battery voltage
    m_lowerMotorSim.setRotorVelocity(m_lowerFlywheelSim.getAngularVelocity( ));

    // SimBattery estimates loaded battery voltages
    RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(m_lowerFlywheelSim.getCurrentDrawAmps( )));
  }

  /****************************************************************************
   * 
   * Initialize dashboard widgets
   */
  private void initDashboard( )
  {
    // Get the default instance of NetworkTables that was created automatically when the robot program starts
    NetworkTableInstance inst = NetworkTableInstance.getDefault( );
    NetworkTable table = inst.getTable("shooter");

    // Initialize network tables publishers
    m_lowerRPMPub = table.getDoubleTopic("lowerSpeed").publish( );

    // m_upperSpeedPub = table.getDoubleTopic("upperSpeed").publish( );
    m_targetRPMPub = table.getDoubleTopic("targetRPM").publish( );
    m_isAtTargetRPMPub = table.getBooleanTopic("atTargetRPM").publish( );

    m_flywheelScoreEntry = table.getDoubleTopic("flywheelRPM").getEntry(0.0);
    m_flywheelScoreEntry.set(kFlywheelScoreRPM);

    // Add commands
    SmartDashboard.putData("ShRunScore", getShooterScoreCommand( ));
    SmartDashboard.putData("ShRunStop", getShooterStopCommand( ));
  }

  // Put methods for controlling this subsystem here. Call these from Commands.

  /****************************************************************************
   * 
   * Initialize subsystem during robot mode changes
   */
  public void initialize( )
  {
    DataLogManager.log(String.format("%s: Subsystem initialized!", getSubsystem( )));
    setShooterMode(ShooterMode.STOP);
  }

  ////////////////////////////////////////////////////////////////////////////
  ///////////////////////// PRIVATE HELPERS //////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////

  /****************************************************************************
   * 
   * Set shooter RPM based on requested mode
   * 
   * @param mode
   *          requested RPM
   */
  private void setShooterMode(ShooterMode mode)
  {
    DataLogManager.log(String.format("%s: Set shooter mode to %s", getSubsystem( ), mode));

    // Select the shooter RPM for the requested mode - NEVER NEGATIVE when running!
    switch (mode)
    {
      default :
        DataLogManager.log(String.format("%s: Shooter mode is invalid: %s", getSubsystem( ), mode));
      case STOP :
        m_targetRPM = 0.0;
        break;
      case SCORE :
        m_targetRPM = m_flywheelScoreEntry.get(0.0);
        break;
    }

    double rotPerSecond = m_targetRPM / 60.0;
    if (m_lowerValid)
    {
      if (m_targetRPM > 100.0)
        setShooterVelocity(rotPerSecond);
      else
        setShooterStopped( );
    }

    DataLogManager.log(String.format("%s: Target rpm is %.1f rps %.1f", getSubsystem( ), m_targetRPM, rotPerSecond));
  }

  /****************************************************************************
   * 
   * Set shooter motors to requested velocity
   * 
   * @param rps
   *          rotations per second
   */
  private void setShooterVelocity(double rps)
  {
    m_lowerMotor.setControl(m_requestVelocity.withVelocity(Conversions.rotationsToInputRotations(rps, kFlywheelGearRatio)));
  }

  /****************************************************************************
   * 
   * Set shooter motors to stopped
   */
  private void setShooterStopped( )
  {
    m_lowerMotor.setControl(m_requestVolts);
  }

  ////////////////////////////////////////////////////////////////////////////
  ///////////////////////// PUBLIC HELPERS ///////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////

  /****************************************************************************
   * 
   * Return shooter speed check against target RPM
   * 
   * @return true if shooter is at target RPM
   */
  public boolean isAtTargetRPM( )
  {
    return m_isAtTargetRPM;
  }

  ////////////////////////////////////////////////////////////////////////////
  ///////////////////////// COMMAND FACTORIES ////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////

  /****************************************************************************
   * 
   * Create shooter command based on passed mode
   * 
   * @param mode
   *          shooter mode that detemines RPM
   * @return instant command that changes shooter motors
   */
  private Command getShooterCommand(ShooterMode mode)
  {
    return new InstantCommand( // Command that runs exactly once
        ( ) -> setShooterMode(mode), // Method to call
        this // Subsystem requirement
    );
  }

  public Command getShooterScoreCommand( )
  {
    return getShooterCommand(ShooterMode.SCORE).withName("ShooterScore");
  }

  /****************************************************************************
   * 
   * Create shooter mode command to stop motors
   * 
   * @return instant command that stops shooter motors
   */
  public Command getShooterStopCommand( )
  {
    return getShooterCommand(ShooterMode.STOP).withName("ShooterStop");
  }

}
