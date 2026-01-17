package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.INConsts.INRollerMode;
import frc.robot.lib.phoenix.CTREConfigs6;
import frc.robot.lib.phoenix.PhoenixUtil6;

import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

public class Intake extends SubsystemBase
{
  private static final String  kSubsystemName        = "Intake";
  private static final boolean kRollerMotorInvert    = false;

  private final TalonFX        m_upperrollerMotor    = new TalonFX(0);
  private final TalonFX        m_lowerrollerMotor    = new TalonFX(1);

  private static final double  kRollerSpeedAcquire   = 0.5;
  private static final double  kRollerSpeedExpel     = -0.4;
  private static final double  kRollerSpeedToShooter = -1.0;
  private static final double  kRollerSpeedToFeeder  = -0.4;
  private static final double  kRollerSpeedHold      = 0.1;

  private static final double  kNoteDebounceTime     = 0.045;
  private BooleanPublisher     m_fuelDetectedPub;

  private boolean              m_upperrollerValid; // Health indicator for motor
  private boolean              m_lowerrollerValid;
  private final DigitalInput   m_fuelInIntake        = new DigitalInput(0);

  private DoublePublisher      m_rollSpeedPub;
  private DoublePublisher      m_rollSupCurPub;

  private Debouncer            m_fuelDebouncer       = new Debouncer(kNoteDebounceTime, DebounceType.kBoth);
  private boolean              m_fuelDetected; // Detection state of note in rollers

  private final CANrange       m_fuelDetector        = new CANrange(5);

  private final Alert          m_upperrollerAlert    =
      new Alert(String.format("%s: Roller motor init failed!", getSubsystem( )), AlertType.kError);

  private final Alert          m_lowerrollerAlert    =
      new Alert(String.format("%s: Roller motor init failed!", getSubsystem( )), AlertType.kError);

  public Intake( )
  {

    setName(kSubsystemName);
    setSubsystem(kSubsystemName);

    m_upperrollerValid = PhoenixUtil6.getInstance( ).talonFXInitialize6(m_upperrollerMotor, kSubsystemName + "Claw",
        CTREConfigs6.upperRollerFXConfig(m_fuelDetector.getDeviceID( )));

    m_upperrollerAlert.set(!m_upperrollerValid);

    initDashboard( );
    initialize( );

  }

  public void periodic( )
  {
    m_rollSpeedPub.set(m_upperrollerMotor.get( ));
    // This method will be called once per scheduler run

  }

  private void initDashboard( )
  {
    // Get the default instance of NetworkTables that was created automatically when
    // the robot program starts

  }

  public void initialize( )
  {
    // setRollerMode(INRollerMode.STOP);
    // setRotaryStopped( );

  }

  // public Command setRollerMode(INRollerMode mode)
  // {
  //m_RequestVolts = kClawRollerStop;

  // if (mode == INRollerMode.HOLD)
  // {
  //   DataLogManager.log(String.format("%s: Claw mode is unchanged - %s (%.3f)", getSubsystem( ), mode, m_clawMotor.get( )));
  // }
  // else
  // {
  // if (mode == ClawMode.ALGAEHOLD)
  // {
  //   double rotations = m_clawMotor.getPosition( ).getValueAsDouble( );
  //   Slot0Configs slot0Configs = new Slot0Configs( ).withKP(25);
  //   m_clawMotor.getConfigurator( ).apply(slot0Configs);
  //   PositionDutyCycle positionDutyCycle = new PositionDutyCycle(rotations).withSlot(0).withEnableFOC(true);
  //   m_clawMotor.setControl(positionDutyCycle);
  // }
  // else
  {
    //   switch (mode)
    //   {
    //     default :
    //       DataLogManager.log(String.format("%s: Claw mode is invalid: %s", getSubsystem( ), mode));
    //     case STOP :
    //       m_clawRequestVolts = (m_algaeDetected) ? kAlgaeSpeedHold : kClawRollerStop;
    //       break;
    //     case ALGAEACQUIRE :
    //       m_clawRequestVolts = kAlgaeSpeedAcquire;
    //       break;
    //     case ALGAEEXPEL :
    //       m_clawRequestVolts = kAlgaeSpeedExpel;
    //       break;
    //     case ALGAESHOOT :
    //       m_clawRequestVolts = kAlgaeSpeedShoot;
    //       break;
    //     case ALGAEPROCESSOR :
    //       m_clawRequestVolts = kAlgaeSpeedProcessor;
    //       break;
    //     case CORALACQUIRE :
    //       m_clawRequestVolts = kCoralSpeedAcquire;
    //       break;
    //     case CORALEXPEL :
    //       m_clawRequestVolts = ((int) m_reefLevel.get( ) == 1) ? kCoralSpeedExpelL1 : kCoralSpeedExpel;
    //       DataLogManager.log(String.format("%s: reefLevel.get is %d", getSubsystem( ), (int) m_reefLevel.get( )));
    //       break;
    //     case ALGAEHOLD :  // Special case above the switch - this case doesn't execute!
    //       m_clawRequestVolts = kAlgaeSpeedHold;
    //       break;
    //   }

    //   m_clawMotor.setControl(m_clawRequestVolts);
    // }

    //     DataLogManager.log(String.format("%s: Claw mode is now - %s", getSubsystem( ), mode));
    //   }

    // }

    // public boolean isFuelDetected( )
    // {

    // }

  }
}
