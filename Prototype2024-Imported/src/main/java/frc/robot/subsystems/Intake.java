package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.INConsts.INRollerMode;
import frc.robot.lib.phoenix.CTREConfigs5;
import frc.robot.lib.phoenix.CTREConfigs6;
import frc.robot.lib.phoenix.PhoenixUtil5;
import frc.robot.lib.phoenix.PhoenixUtil6;

public class Intake extends SubsystemBase
{
  private static final String  kSubsystemName        = "Intake";
  private static final boolean kRollerMotorInvert    = false;

  private final WPI_TalonSRX   m_upperrollerMotor    = new WPI_TalonSRX(1);
  private final WPI_TalonSRX   m_lowerrollerMotor    = new WPI_TalonSRX(2);
  private final CANcoder       m_CANcoder            = new CANcoder(3);

  private static final double  kRollerSpeedAcquire   = 0.5;
  private static final double  kRollerSpeedExpel     = -0.4;
  private static final double  kRollerSpeedToShooter = -1.0;
  private static final double  kRollerSpeedToFeeder  = -0.4;
  private static final double  kRollerSpeedHold      = 0.1;

  private static final double  kRotaryGearRatio      = 30.83;
  private static final double  kRotaryLengthMeters   = 0.3;       // Simulation
  private static final double  kRotaryWeightKg       = 4.0;       // Simulation
  private static final Voltage kRotaryManualVolts    = Volts.of(3.5);
  private static final double  kNoteDebounceTime     = 0.045;
  private BooleanPublisher     m_fuelDetectedPub;

  private boolean              m_upperrollerValid;        // Health indicator for motor 
  private boolean              m_lowerrollerValid;
  private boolean              m_fuelDetected;
  public double                m_targetDegrees       = 0.0;
  public boolean               m_canCoderValid;
  private final DigitalInput   m_fuelInIntake        = new DigitalInput(0);

  private DoublePublisher      m_rollSpeedPub;
  private DoublePublisher      m_rollSupCurPub;

  private Debouncer            m_fuelDebouncer       = new Debouncer(kNoteDebounceTime, DebounceType.kBoth);
  //private boolean                   m_fuelDetected;       // Detection state of note in rollers

  private final Alert          m_upperrollerAlert    =
      new Alert(String.format("%s: Roller motor init failed!", getSubsystem( )), AlertType.kError);

  private final Alert          m_lowerrollerAlert    =
      new Alert(String.format("%s: Roller motor init failed!", getSubsystem( )), AlertType.kError);

  public Intake( )
  {

    setName(kSubsystemName);
    setSubsystem(kSubsystemName);

    m_upperrollerValid = PhoenixUtil5.getInstance( ).talonSRXInitialize(m_upperrollerMotor, kSubsystemName + "Roller",
        CTREConfigs5.intakeRollerConfig( ));
    m_upperrollerMotor.setInverted(kRollerMotorInvert);
    PhoenixUtil5.getInstance( ).talonSRXCheckError(m_upperrollerMotor, "setInverted");

    m_upperrollerAlert.set(!m_upperrollerValid);

    m_lowerrollerValid = PhoenixUtil5.getInstance( ).talonSRXInitialize(m_lowerrollerMotor, kSubsystemName + "Roller",
        CTREConfigs5.intakeRollerConfig( ));
    m_lowerrollerMotor.setInverted(kRollerMotorInvert);
    PhoenixUtil5.getInstance( ).talonSRXCheckError(m_lowerrollerMotor, "setInverted");

    m_lowerrollerAlert.set(!m_lowerrollerValid);

    initDashboard( );
    initialize( );

  }

  public void periodic( )
  {
    // This method will be called once per scheduler run
    m_fuelDetected = m_fuelDebouncer.calculate(m_fuelInIntake.get( ));

  }

  private void initDashboard( )
  {
    // Get the default instance of NetworkTables that was created automatically when the robot program starts
    NetworkTableInstance inst = NetworkTableInstance.getDefault( );
    NetworkTable table = inst.getTable("intake");

    // Initialize network tables publishers
    m_rollSpeedPub = table.getDoubleTopic("rollSpeed").publish( );
    m_rollSupCurPub = table.getDoubleTopic("rollSupCur").publish( );

    DoublePublisher m_ccDegreesPub = table.getDoubleTopic("ccDegrees").publish( );
    m_fuelDetectedPub = table.getBooleanTopic("fuelDetected").publish( );

  }

  public void initialize( )
  {
    // setRollerMode(INRollerMode.STOP);
    // setRotaryStopped( );
    setRollerMode(INRollerMode.STOP);

  }

  private void setRollerMode(INRollerMode mode)
  {
    double output = 0.0;

    if (mode == INRollerMode.HOLD)
    {
      DataLogManager
          .log(String.format("%s: Roller mode is unchanged - %s (%.3f)", getSubsystem( ), mode, m_upperrollerMotor.get( )));
      DataLogManager
          .log(String.format("%s: Roller mode is unchanged - %s (%.3f)", getSubsystem( ), mode, m_lowerrollerMotor.get( )));
    }
    else
    {
      switch (mode)
      {
        default :
          DataLogManager.log(String.format("%s: Roller mode is invalid: %s", getSubsystem( ), mode));
        case STOP :
          output = (m_fuelDetected) ? kRollerSpeedHold : 0.0;
          break;
        case ACQUIRE :
          output = kRollerSpeedAcquire;
          break;
        case EXPEL :
          output = kRollerSpeedExpel;
          break;
        case SHOOT :
          output = kRollerSpeedToShooter;
          break;
        case HANDOFF :
          output = kRollerSpeedToFeeder;
      }
      DataLogManager.log(String.format("%s: Roller mode is now - %s", getSubsystem( ), mode));
      m_upperrollerMotor.set(output);
      m_lowerrollerMotor.set(output);
    }
  }

  public boolean isFuelDetected( )
  {
    return m_fuelDetected;
  }

}
