
// Phoenix 6 configurations

package frc.robot.lib.phoenix;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.signals.UpdateModeValue;

import frc.robot.Robot;

/****************************************************************************
 * 
 * CTRE configuration structure for v6 devices
 */
public final class CTREConfigs6
{

  /****************************************************************************
   * 
   * Intake roller motor - Kraken X44
   * 
   * @return inRollerConfig
   */
  public static TalonFXConfiguration intakeRollerFXConfig( )
  {
    TalonFXConfiguration inRollerConfig = new TalonFXConfiguration( );

    // Current limit settings
    inRollerConfig.CurrentLimits.SupplyCurrentLimit = 25.0;       // Amps
    inRollerConfig.CurrentLimits.SupplyCurrentLowerLimit = 25.0;  // Amps
    inRollerConfig.CurrentLimits.SupplyCurrentLowerTime = 0.100;  // Seconds
    inRollerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    inRollerConfig.CurrentLimits.StatorCurrentLimit = 200.0;      // Amps
    inRollerConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    // Motor output settings
    inRollerConfig.MotorOutput.DutyCycleNeutralDeadband = 0.001;    // Percentage
    inRollerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    inRollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    return inRollerConfig;
  }

  /****************************************************************************
   * 
   * Intake rotary motor - Kraken X44
   * 
   * @param min
   *          minimum angle of rotation
   * @param max
   *          maximum angle of rotation (must be greater than min)
   * @param ccPort
   *          CANcoder port number
   * @param gearRatio
   *          gear box ratio
   * @return inRotaryConfig
   */
  public static TalonFXConfiguration intakeRotaryFXConfig(double min, double max, int ccPort, double gearRatio)
  {
    TalonFXConfiguration inRotaryConfig = new TalonFXConfiguration( );

    // Closed Loop settings
    // inRotaryConfig.ClosedLoopGeneral.*
    // inRotaryConfig.ClosedLoopRamps.*                           // Seconds to ramp

    // Current limit settings
    inRotaryConfig.CurrentLimits.SupplyCurrentLimit = 25.0;       // Amps
    inRotaryConfig.CurrentLimits.SupplyCurrentLowerLimit = 25.0;  // Amps
    inRotaryConfig.CurrentLimits.SupplyCurrentLowerTime = 0.001;  // Seconds
    inRotaryConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    inRotaryConfig.CurrentLimits.StatorCurrentLimit = 100.0;      // Amps
    inRotaryConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    // Feedback settings
    inRotaryConfig.Feedback.FeedbackRemoteSensorID = ccPort;
    inRotaryConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    inRotaryConfig.Feedback.SensorToMechanismRatio = 1.0;
    inRotaryConfig.Feedback.RotorToSensorRatio = gearRatio;

    // Hardware limit switches - NONE
    // inRotaryConfig.HardwareLimitSwitch.*

    // Motion Magic settings - fused CANcoder affects all feedback constants by the gearRatio
    inRotaryConfig.MotionMagic.MotionMagicCruiseVelocity = 50.0 / gearRatio;  // Rotations / second
    inRotaryConfig.MotionMagic.MotionMagicAcceleration = 220.0 / gearRatio;   // Rotations / second ^ 2
    inRotaryConfig.MotionMagic.MotionMagicJerk = 1600.0 / gearRatio;          // Rotations / second ^ 3

    // Motor output settings
    inRotaryConfig.MotorOutput.DutyCycleNeutralDeadband = 0.001;    // Percentage
    inRotaryConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    inRotaryConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // Open Loop settings
    // inRotaryConfig.OpenLoopRamps.*                               // Seconds to ramp

    // Slot settings - remote/fused CANcoder affects all feedback constants by the gearRatio
    // kG = (0.40 + 0.25) / 2
    // kS = (0.40 - 0.25) / 2
    inRotaryConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine; // Feedforward: Mechanism is an arm and needs cosine
    inRotaryConfig.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;
    inRotaryConfig.Slot0.kS = 0.0;                                  // Feedforward: Voltage or duty cycle to overcome static friction
    inRotaryConfig.Slot0.kG = -0.50;                                // Feedforward: Voltage or duty cycle to overcome gravity (arbitrary feedforward)
    inRotaryConfig.Slot0.kV = 0.1129;                               // Feedforward: Voltage or duty cycle per requested RPS (velocity modes)

    // NOTE: Motion Magic settings are scaled by gear ration when using a FusecCANCoder
    inRotaryConfig.Slot0.kP = 3.6 * gearRatio;                      // Feedback: Voltage or duty cycle per velocity unit (velocity modes)
    inRotaryConfig.Slot0.kI = 0.0 * gearRatio;                      // Feedback: Voltage or duty cycle per accumulated unit
    inRotaryConfig.Slot0.kD = 0.0 * gearRatio;                      // Feedback: Voltage or duty cycle per unit of acceleration unit (velocity modes)

    // Software limit switches
    inRotaryConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = min;  // Rotations
    inRotaryConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    inRotaryConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = max;  // Rotations
    inRotaryConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;

    return inRotaryConfig;
  }

  /****************************************************************************
   * 
   * Intake rotary CANcoder
   * 
   * @return inRotaryConfig
   */
  public static CANcoderConfiguration intakeRotaryCCConfig( )
  {
    CANcoderConfiguration inRotaryConfig = new CANcoderConfiguration( );
    double kQuarterRotation = 0.25;

    inRotaryConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    inRotaryConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.25;

    if (Robot.isReal( ))
      inRotaryConfig.MagnetSensor.MagnetOffset = (Robot.isComp( )) ? (-0.311768 - kQuarterRotation) : (0.1184 - kQuarterRotation); // TODO: Update for 2026
    else
      inRotaryConfig.MagnetSensor.MagnetOffset = -0.25;                   // Simulated CANcoder default in rotations

    return inRotaryConfig;
  }

  /****************************************************************************
   * 
   * Hopper roller motor - Kraken X44
   * 
   * @return hpRollerConfig
   */
  public static TalonFXConfiguration hopperRollerFXConfig( )
  {
    TalonFXConfiguration hpRollerConfig = new TalonFXConfiguration( );

    // Current limit settings
    hpRollerConfig.CurrentLimits.SupplyCurrentLimit = 25.0;       // Amps
    hpRollerConfig.CurrentLimits.SupplyCurrentLowerLimit = 25.0;  // Amps
    hpRollerConfig.CurrentLimits.SupplyCurrentLowerTime = 0.100;  // Seconds
    hpRollerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    hpRollerConfig.CurrentLimits.StatorCurrentLimit = 200.0;      // Amps
    hpRollerConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    // Motor output settings
    hpRollerConfig.MotorOutput.DutyCycleNeutralDeadband = 0.001;    // Percentage
    hpRollerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    hpRollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    return hpRollerConfig;
  }

  /****************************************************************************
   * 
   * Kicker roller motor - Kraken X44
   * 
   * @return kkRollerConfig
   */
  public static TalonFXConfiguration kickerRollerFXConfig( )
  {
    TalonFXConfiguration kkRollerConfig = new TalonFXConfiguration( );

    // Current limit settings
    kkRollerConfig.CurrentLimits.SupplyCurrentLimit = 25.0;       // Amps
    kkRollerConfig.CurrentLimits.SupplyCurrentLowerLimit = 25.0;  // Amps
    kkRollerConfig.CurrentLimits.SupplyCurrentLowerTime = 0.100;  // Seconds
    kkRollerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    kkRollerConfig.CurrentLimits.StatorCurrentLimit = 200.0;      // Amps
    kkRollerConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    // Motor output settings
    kkRollerConfig.MotorOutput.DutyCycleNeutralDeadband = 0.001;    // Percentage
    kkRollerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    kkRollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    return kkRollerConfig;
  }

  /****************************************************************************
   * 
   * Launcher motors - Kraken X60 (2)
   * 
   * @return launcherConfig
   */
  public static TalonFXConfiguration launcherFXConfig( )      // TODO: needs to be updated for Launcher
  {
    TalonFXConfiguration launcherConfig = new TalonFXConfiguration( );

    // Closed Loop settings
    // inRotaryConfig.ClosedLoopGeneral.*
    // inRotaryConfig.ClosedLoopRamps.*                             // Seconds to ramp

    launcherConfig.CurrentLimits.SupplyCurrentLimit = 35.0;          // Amps
    launcherConfig.CurrentLimits.SupplyCurrentLowerLimit = 35.0;     // Amps
    launcherConfig.CurrentLimits.SupplyCurrentLowerTime = 0.001;     // Seconds
    launcherConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    // launcherConfig.CurrentLimits.StatorCurrentLimit = 100.0;      // Amps
    // launcherConfig.CurrentLimits.StatorCurrentLimitEnable = false;

    // launcherConfig.Feedback.*
    // launcherConfig.HardwareLimitSwitch.*
    // launcherConfig.MotionMagic.*

    // Motor output settings
    // launcherConfig.MotorOutput.DutyCycleNeutralDeadband = 0.001;
    launcherConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    launcherConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    // Open Loop settings
    // launcherConfig.OpenLoopRamps.*                                  // Seconds to ramp

    // Slot settings
    // launcherConfig.Slot0.GravityType = *; 
    launcherConfig.Slot0.kS = 0.0;                                     // Feedforward: Voltage or duty cycle to overcome static friction
    launcherConfig.Slot0.kG = 0.0;                                     // Feedforward: Voltage or duty cycle to overcome gravity (arbitrary feedforward)
    launcherConfig.Slot0.kV = 0.1140;                                  // Feedforward: Voltage or duty cycle per requested RPS (velocity modes)

    launcherConfig.Slot0.kP = 0.25;                                    // Voltage or duty cycle per velocity unit (velocity modes)
    launcherConfig.Slot0.kI = 0.0;                                     // Voltage or duty cycle per accumulated unit
    launcherConfig.Slot0.kD = 0.0;                                     // Voltage or duty cycle per unit of acceleration unit (velocity modes)

    // launcherConfig.SoftwareLimitSwitch.*

    return launcherConfig;
  }

  /****************************************************************************
   * 
   * Climber motors (2 - one for left and right) - Kraken X60 (2)
   * 
   * @param inverted
   *          motor inversion request
   * @param min
   *          minimum deployment distance
   * @param max
   *          maximum deployement distance (must be greater than min)
   */
  public static TalonFXConfiguration climberFXConfig(boolean inverted, double min, double max)
  {
    TalonFXConfiguration climberConfig = new TalonFXConfiguration( );

    // Closed Loop settings
    // exConfig.ClosedLoopGeneral.*
    // exConfig.ClosedLoopRamps.*

    // Current limit settings
    climberConfig.CurrentLimits.SupplyCurrentLimit = 80.0;        // Amps
    climberConfig.CurrentLimits.SupplyCurrentLowerLimit = 80.0;   // Amps
    climberConfig.CurrentLimits.SupplyCurrentLowerTime = 0.001;   // Seconds
    climberConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    climberConfig.CurrentLimits.StatorCurrentLimit = 800.0;        // Amps
    climberConfig.CurrentLimits.StatorCurrentLimitEnable = false;

    // Feedback settings
    // climberConfig.Feedback.*

    // Hardware limit switches - NONE
    // climberConfig.HardwareLimitSwitch.*

    // Motion Magic settings
    climberConfig.MotionMagic.MotionMagicCruiseVelocity = 79.75;  // Rotations / second
    climberConfig.MotionMagic.MotionMagicAcceleration = 159.5;    // Rotations / second ^ 2
    climberConfig.MotionMagic.MotionMagicJerk = 3544;             // Rotations / second ^ 3

    // Motor output settings
    climberConfig.MotorOutput.DutyCycleNeutralDeadband = 0.001;   // Percentage
    climberConfig.MotorOutput.Inverted = (inverted) ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
    climberConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // Open Loop settings
    // climberConfig.OpenLoopRamps.*                              // Seconds to ramp

    // Slot settings
    climberConfig.Slot0.kS = 0.0;                                 // Feedforward: Voltage or duty cylce to overcome static friction
    climberConfig.Slot0.kG = 0.0;                                 // Feedforward: Voltage or duty cylce to overcome gravity (arbitrary feedforward)
    climberConfig.Slot0.kV = 0.1129;                              // Feedforward: Voltage or duty cycle per requested RPS (velocity modes)

    climberConfig.Slot0.kP = 9.60;                                // Feedback: Voltage or duty cycle per velocity unit (velocity modes)
    climberConfig.Slot0.kI = 0.0;                                 // Feedback: Voltage or duty cycle per accumulated unit
    climberConfig.Slot0.kD = 0.0;                                 // Feedback: Voltage or duty cycle per unit of acceleration unit (velocity modes)

    // Software limit switches
    // climberConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = min;   // Rotations
    // climberConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    // climberConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = max;   // Rotations
    // climberConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;

    return climberConfig;
  }

  /****************************************************************************
   * 
   * XXX CANRange detector
   */
  public static CANrangeConfiguration xxxCANRangeConfig( )
  {
    CANrangeConfiguration crConfig = new CANrangeConfiguration( );

    crConfig.ProximityParams.ProximityThreshold = 0.1;                  // Proximity distance in meters (about 8 inches)
    crConfig.ToFParams.UpdateMode = UpdateModeValue.ShortRange100Hz;

    return crConfig;
  }

  // Swerve module configs are built into swerve subsystem

}
