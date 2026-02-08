package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.INConsts.RollerMode;
import frc.robot.lib.phoenix.CTREConfigs6;
import frc.robot.lib.phoenix.PhoenixUtil6;

public class Intake extends SubsystemBase
{
  private static final String       kSubsystemName       = "Intake";
  // private static final boolean      kRollerMotorInvert   = false;

  private final TalonFX             m_upperRollerMotor   = new TalonFX(17);
  // private final TalonFX             m_lowerRollerMotor   = new TalonFX(18);

  private static final DutyCycleOut kUpperRollerStop     = new DutyCycleOut(0.0).withIgnoreHardwareLimits(true);
  private static final DutyCycleOut kFuelSpeedAcquire    = new DutyCycleOut(0.5).withIgnoreHardwareLimits(true);
  private static final DutyCycleOut kFuelSpeedExpel      = new DutyCycleOut(-0.27).withIgnoreHardwareLimits(true);

  private DutyCycleOut              m_rollerRequestVolts = kUpperRollerStop;

  private boolean                   m_upperrollerValid;   // Health indicator for motor
  // private boolean                   m_lowerrollerValid;

  private DoublePublisher           m_rollSpeedPub;
  // private DoublePublisher           m_rollSupCurPub;

  private final TalonFXSimState     m_UpperRollerSim     = new TalonFXSimState(m_upperRollerMotor);

  private final Alert               m_upperRollerAlert   =
      new Alert(String.format("%s: Roller motor init failed!", getSubsystem( )), AlertType.kError);

  // private final Alert               m_lowerRollerAlert    =
  //     new Alert(String.format("%s: Roller motor init failed!", getSubsystem( )), AlertType.kError);

  /****************************************************************************
   * 
   * Constructor
   */
  public Intake( )
  {
    setName(kSubsystemName);
    setSubsystem(kSubsystemName);

    m_upperrollerValid = PhoenixUtil6.getInstance( ).talonFXInitialize6(m_upperRollerMotor, kSubsystemName + "Claw",
        CTREConfigs6.upperRollerFXConfig( ));
    m_upperRollerAlert.set(!m_upperrollerValid);

    initDashboard( );
    initialize( );
  }

  /****************************************************************************
   * 
   * Periodic actions that run every scheduler loop time (20 msec)
   */
  public void periodic( )
  {
    // This method will be called once per scheduler run

    m_rollSpeedPub.set(m_upperRollerMotor.get( ));
  }

  private void initDashboard( )
  {
    NetworkTableInstance inst = NetworkTableInstance.getDefault( );
    NetworkTable table = inst.getTable("intake");
    // Get the default instance of NetworkTables that was created automatically when the robot program starts
    m_rollSpeedPub = table.getDoubleTopic("intakeSpeed").publish( );

    SmartDashboard.putData("IntakeRun", getIntakeOnCommand( ));
    SmartDashboard.putData("IntakeReverse", IntakeReverse( ));
    SmartDashboard.putData("IntakeOff", getIntakeOffCommand( ));
  }

  /****************************************************************************
   * 
   * Initialize subsystem during robot mode changes
   */
  public void initialize( )
  {
    DataLogManager.log(String.format("%s: Subsystem initialized!", getSubsystem( )));
    setRollerMode(RollerMode.STOP);
  }

  public void simulationPeriodic( )
  {
    // This method will be called once per scheduler run during simulation

    // Set input flywheel voltage from the motor setting
    m_UpperRollerSim.setSupplyVoltage(RobotController.getInputVoltage( ));
  }

  private void setRollerMode(RollerMode mode)
  {
    m_rollerRequestVolts = kUpperRollerStop;
    if (mode == RollerMode.HOLD)
    {
      DataLogManager
          .log(String.format("%s: Roller mode is unchanged - %s (%.3f)", getSubsystem( ), mode, m_upperRollerMotor.get( )));
    }
    else
    {
      switch (mode)
      {
        default :
          DataLogManager.log(String.format("%s: Claw mode is invalid: %s", getSubsystem( ), mode));
        case STOP :
          m_rollerRequestVolts = kUpperRollerStop;
          break;
        case ACQUIRE :
          m_rollerRequestVolts = kFuelSpeedAcquire;

          break;
        case EXPEL :
          m_rollerRequestVolts = kFuelSpeedExpel;
          break;
      }
      m_upperRollerMotor.setControl(m_rollerRequestVolts);
    }
  }

  private Command getRollerCommand(RollerMode mode)
  {
    return new InstantCommand(        // Command that runs exactly once
        ( ) -> setRollerMode(mode),  // Method to call
        this                          // Subsystem requirement
    );
  }

  public Command getIntakeOnCommand( )
  {
    return getRollerCommand(RollerMode.ACQUIRE).withName("getIntakeOnCommand");
  }

  public Command IntakeReverse( )
  {
    return getRollerCommand(RollerMode.EXPEL).withName("IntakeReverse");
  }

  public Command getIntakeOffCommand( )
  {
    return getRollerCommand(RollerMode.STOP).withName("IntakeStopper");
  }
}
