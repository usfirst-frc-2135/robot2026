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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.INConsts.RollerMode;
import frc.robot.lib.phoenix.CTREConfigs6;
import frc.robot.lib.phoenix.PhoenixUtil6;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

public class Intake extends SubsystemBase
{
  private static final String       kSubsystemName        = "Intake";
  private static final boolean      kRollerMotorInvert    = false;

  private final TalonFX             m_upperrollerMotor    = new TalonFX(0);
  private final TalonFX             m_lowerrollerMotor    = new TalonFX(1);

  private static final DutyCycleOut kUpperRollerStop      = new DutyCycleOut(0.0).withIgnoreHardwareLimits(true);
  private static final DutyCycleOut kFuelSpeedAcquire     = new DutyCycleOut(0.5).withIgnoreHardwareLimits(true);
  private static final DutyCycleOut kFuelSpeedExpel       = new DutyCycleOut(-0.27).withIgnoreHardwareLimits(true);
  private static final DutyCycleOut kFuelSpeedHold        = new DutyCycleOut(0.2).withIgnoreHardwareLimits(true);

  private DutyCycleOut              m_rollerRequestVolts  = kUpperRollerStop;

  private static final double       kRollerSpeedToShooter = -1.0;
  private static final double       kRollerSpeedToFeeder  = -0.4;
  private static final double       kRollerSpeedHold      = 0.1;

  private static final double       kNoteDebounceTime     = 0.045;
  private BooleanPublisher          m_fuelDetectedPub;

  private boolean                   m_upperrollerValid; // Health indicator for motor
  private boolean                   m_lowerrollerValid;
  private final DigitalInput        m_fuelInIntake        = new DigitalInput(0);

  private DoublePublisher           m_rollSpeedPub;
  private DoublePublisher           m_rollSupCurPub;

  private Debouncer                 m_fuelDebouncer       = new Debouncer(kNoteDebounceTime, DebounceType.kBoth);
  private boolean                   m_fuelDetected; // Detection state of note in rollers

  private final CANrange            m_fuelDetector        = new CANrange(5);

  private final Alert               m_upperrollerAlert    =
      new Alert(String.format("%s: Roller motor init failed!", getSubsystem( )), AlertType.kError);

  private final Alert               m_lowerrollerAlert    =
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
    SmartDashboard.putData("IntakeRun", IntakeOn( ));
    SmartDashboard.putData("IntakeStop", IntakeStop( ));

  }

  public void initialize( )
  {
    DataLogManager.log(String.format("%s: Subsystem initialized!", getSubsystem( )));
    setRollerMode(RollerMode.STOP);
    // setRollerMode(INRollerMode.STOP);
    // setRotaryStopped( );
    //SmartDashboard.putData("ShRunScore", ( ));
    //SmartDashboard.putData("ShRunStop", getShooterStopCommand( ));
  }

  private void setRollerMode(RollerMode mode)
  {
    m_rollerRequestVolts = kUpperRollerStop;
    if (mode == RollerMode.FUELHOLD)
    {
      DataLogManager
          .log(String.format("%s: Roller mode is unchanged - %s (%.3f)", getSubsystem( ), mode, m_upperrollerMotor.get( )));
    }
    else
    {
      // if (mode == RollerMode.FUELHOLD)
      // {
      //   double rotations = m_upperrollerMotor.getPosition( ).getValueAsDouble( );
      //   Slot0Configs slot0Configs = new Slot0Configs( ).withKP(25);
      //   m_upperrollerMotor.getConfigurator( ).apply(slot0Configs);
      //   PositionDutyCycle positionDutyCycle = new PositionDutyCycle(rotations).withSlot(0).withEnableFOC(true);
      //   m_upperrollerMotor.setControl(positionDutyCycle);
      // }
      // else
      {
        switch (mode)
        {
          default :
            DataLogManager.log(String.format("%s: Claw mode is invalid: %s", getSubsystem( ), mode));
          case STOP :
            m_rollerRequestVolts = (m_fuelDetected) ? kFuelSpeedHold : kUpperRollerStop;
            break;
          case FUELACQUIRE :
            m_rollerRequestVolts = kFuelSpeedAcquire;
            break;
          case FUELEXPEL :
            m_rollerRequestVolts = kFuelSpeedExpel;
            break;
        }
        m_upperrollerMotor.setControl(m_rollerRequestVolts);
      }
    }
  }

  private Command getRollerCommand(RollerMode mode)
  {
    return new InstantCommand(        // Command that runs exactly once
        ( ) -> setRollerMode(mode),  // Method to call
        this                          // Subsystem requirement
    );
  }

  public Command IntakeOn( )
  {
    return getRollerCommand(RollerMode.FUELACQUIRE).withName("IntakeOn");
  }

  public Command IntakeStop( )
  {
    return getRollerCommand(RollerMode.STOP).withName("IntakeStopper");
  }
}

//   //     DataLogManager.log(String.format("%s: Claw mode is now - %s", getSubsystem( ), mode));
//   //   }

//   // }

//   // public boolean isFuelDetected( )
//   // {

//   // }
