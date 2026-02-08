package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
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
import frc.robot.lib.phoenix.CTREConfigs5;
import frc.robot.lib.phoenix.CTREConfigs6;
import frc.robot.lib.phoenix.PhoenixUtil5;
import frc.robot.lib.phoenix.PhoenixUtil6;
import edu.wpi.first.wpilibj.Timer;

public class Shooter extends SubsystemBase
{
  private static final String kSubsystemName     = "Shooter";
  private static final double kMOI               = 0.001; // Simulation - Moment of Inertia
  private static final double kFlywheelScoreRPM  = 3500.0; // RPM to score
  private static final double kFlywheelPassRPM   = 3000.0; // RPM to pass
  private static final double kToleranceRPM      = 150.0; // Tolerance band around target RPM
  private static final double kFlywheelGearRatio = (18.0 / 18.0);
  private static final double kKickerVoltageOut  = 12.0;

  /** Shooter (speed) modes */
  private enum ShooterMode
  {
    STOP,       // Shooter is stopped
    PASS,       // Shooter speed for passing fuel
    SCORE,       // Shooter speed for shooting
    WAIT
  }

  // Devices objects
  private final TalonFX                       m_leftMotor             = new TalonFX(14);
  private final TalonFX                       m_rightMotor            = new TalonFX(15);
  private final WPI_TalonSRX                  m_kickerMotor           = new WPI_TalonSRX(16);

  // Alerts
  private final Alert                         m_leftAlert             =
      new Alert(String.format("%s: Left motor init failed!", getSubsystem( )), AlertType.kError);
  private final Alert                         m_rightAlert            =
      new Alert(String.format("%s: Right motor init failed!", getSubsystem( )), AlertType.kError);
  private final Alert                         m_kickerAlert           =
      new Alert(String.format("%s: Kicker motor init failed!", getSubsystem( )), AlertType.kError);

  // Simulation objects
  private final TalonFXSimState               m_leftMotorSim          = new TalonFXSimState(m_leftMotor);
  //private final TalonSRXSimState               m_kickerMotorSim          = new TalonSRXSimState(m_kickerMotor);
  private final FlywheelSim                   m_leftFlywheelSim       = new FlywheelSim(
      LinearSystemId.createFlywheelSystem(DCMotor.getFalcon500(1), kMOI, kFlywheelGearRatio), DCMotor.getFalcon500(1), 0.0);

  // CTRE Status signals for sensors
  private final StatusSignal<AngularVelocity> m_leftVelocity; // Default 4Hz (250ms)

  // Declare module variables
  private boolean                             m_leftValid;
  private boolean                             m_rightValid;
  private boolean                             m_kickerValid;
  private double                              m_leftRPM;                        // Current lower RPM
  private double                              m_targetRPM             = 0;      // Requested target flywheel RPM
  private boolean                             m_isAtTargetRPM         = false;  // Indicates flywheel RPM is close to target
  private boolean                             m_isAtTargetRPMPrevious = false;
  private VelocityVoltage                     m_requestVelocity       = new VelocityVoltage(0.0);
  private VoltageOut                          m_requestVolts          = new VoltageOut(0.0);
  private LinearFilter                        m_leftFlywheelFilter    = LinearFilter.singlePoleIIR(0.060, 0.020);

  // Network tables publisher objects
  private DoublePublisher                     m_leftRPMPub;

  private DoublePublisher                     m_targetRPMPub;
  private BooleanPublisher                    m_isAtTargetRPMPub;
  private DoubleEntry                         m_scoreRPMEntry;

  /****************************************************************************
   * 
   * Constructor
   */
  public Shooter( )
  {
    setName("Shooter");
    setSubsystem("Shooter");

    m_leftValid =
        PhoenixUtil6.getInstance( ).talonFXInitialize6(m_leftMotor, kSubsystemName + "Left", CTREConfigs6.shooterFXConfig( ));
    m_leftAlert.set(!m_leftValid);

    m_rightValid =
        PhoenixUtil6.getInstance( ).talonFXInitialize6(m_rightMotor, kSubsystemName + "Right", CTREConfigs6.shooterFXConfig( ));
    m_rightAlert.set(!m_rightValid);
    m_rightMotor.setControl(new Follower(m_leftMotor.getDeviceID( ), MotorAlignmentValue.Opposed));

    m_kickerValid =
        PhoenixUtil5.getInstance( ).talonSRXInitialize(m_kickerMotor, kSubsystemName + "Kicker", CTREConfigs5.kickerSRXConfig( ));
    m_kickerAlert.set(!m_kickerValid);
    m_kickerMotor.setInverted(false);

    // Initialize status signal objects
    m_leftVelocity = m_leftMotor.getRotorVelocity( );

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
    if (m_leftValid)
    {
      // Calculate flywheel RPM and update network tables publishers
      BaseStatusSignal.refreshAll(m_leftVelocity);
      m_leftRPM = m_leftFlywheelFilter.calculate((m_leftVelocity.getValue( ).in(RotationsPerSecond) * 60.0));
      m_leftRPMPub.set(m_leftRPM);

      m_isAtTargetRPM = ((m_leftRPM > kToleranceRPM) && MathUtil.isNear(m_targetRPM, m_leftRPM, kToleranceRPM));
      m_isAtTargetRPMPub.set(m_isAtTargetRPM);

      if (m_isAtTargetRPM != m_isAtTargetRPMPrevious)
      {
        DataLogManager.log(String.format("%s: At desired RPM: %.1f", getSubsystem( ), m_targetRPM));
        m_isAtTargetRPMPrevious = m_isAtTargetRPM;
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
    m_leftFlywheelSim.setInput(m_leftMotorSim.getMotorVoltage( ));

    // update for 20 msec loop
    m_leftFlywheelSim.update(0.020);

    // Finally, we set our simulated encoder's readings and simulated battery voltage
    m_leftMotorSim.setRotorVelocity(m_leftFlywheelSim.getAngularVelocity( ));
    //m_kickerMotorSim.setRotorVelocity(m_leftFlywheelSim.getAngularVelocity( ));

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
    NetworkTable table = inst.getTable("shooter");

    // Initialize network tables publishers
    m_leftRPMPub = table.getDoubleTopic("lowerSpeed").publish( );

    m_targetRPMPub = table.getDoubleTopic("targetRPM").publish( );
    m_isAtTargetRPMPub = table.getBooleanTopic("atTargetRPM").publish( );

    m_scoreRPMEntry = table.getDoubleTopic("scoreRPM").getEntry(0.0);
    m_scoreRPMEntry.set(kFlywheelScoreRPM);

    // Add commands
    SmartDashboard.putData("ShRunScore", getShooterScoreCommand( ));
    SmartDashboard.putData("ShRunStop", getShooterStopCommand( ));
  }

  // Put methods for controlling this subsystem below here. Call these from Commands.

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
      case PASS :
        m_targetRPM = kFlywheelPassRPM;
        break;
      case SCORE :
        m_targetRPM = m_scoreRPMEntry.get(0.0);
        break;
      

    }

    double rotPerSecond = m_targetRPM / 60.0;
    if (m_leftValid)
    {
      if (m_targetRPM < 0)
      {
        setShooterVelocity(rotPerSecond);
      }
      else if (m_targetRPM > 100.0)
      {
        setShooterVelocity(rotPerSecond);
      }
      else
      {
        setShooterStopped( );
      }
    }

    m_targetRPMPub.set(m_targetRPM);

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
    m_leftMotor.setControl(m_requestVelocity.withVelocity(Conversions.rotationsToInputRotations(rps, kFlywheelGearRatio)));
    Timer.delay(0.1);
    m_kickerMotor.setVoltage(kKickerVoltageOut);
  }

  /****************************************************************************
   * 
   * Set shooter motors to stopped
   */
  private void setShooterStopped( )
  {
    m_leftMotor.setControl(m_requestVolts.withOutput(0.0));
    m_kickerMotor.setVoltage(0.0);
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
    return new InstantCommand(        // Command that runs exactly once
        ( ) -> setShooterMode(mode),  // Method to call
        this                          // Subsystem requirement
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
