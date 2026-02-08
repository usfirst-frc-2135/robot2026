
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
   * @param min
   *          minimum angle of rotation
   * @param max
   *          maximum angle of rotation (must be greater than min)
   * @param ccPort
   *          CANcoder port number
   * @param gearRatio
   *          gear box ratio
   */
  public static TalonFXConfiguration intakeRollerFXConfig( )
  {
    TalonFXConfiguration inRollerConfig = new TalonFXConfiguration( );

    // Closed Loop settings
    // inRollerConfig.ClosedLoopGeneral.*
    // inRollerConfig.ClosedLoopRamps.*                           // Seconds to ramp

    // Current limit settings
    inRollerConfig.CurrentLimits.SupplyCurrentLimit = 25.0;       // Amps
    inRollerConfig.CurrentLimits.SupplyCurrentLowerLimit = 25.0;  // Amps
    inRollerConfig.CurrentLimits.SupplyCurrentLowerTime = 0.100;  // Seconds
    inRollerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    inRollerConfig.CurrentLimits.StatorCurrentLimit = 200.0;      // Amps
    inRollerConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    // Feedback settings
    // inRollerConfig.Feedback.*

    // Hardware limit switches - CANrange
    // inRollerConfig.HardwareLimitSwitch.*

    // Motion Magic settings - fused CANcoder affects all feedback constants by the gearRatio
    // inRollerConfig.MotionMagic.*

    // Motor output settings
    inRollerConfig.MotorOutput.DutyCycleNeutralDeadband = 0.001;    // Percentage
    inRollerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    inRollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    // Open Loop settings
    // inRollerConfig.OpenLoopRamps.*                               // Seconds to ramp

    // Slot settings - remote/fused CANcoder affects all feedback constants by the gearRatio
    // inRollerConfig.Slot0.*                                       // Feedback: Voltage or duty cycle per unit of acceleration unit (velocity modes)

    // Software limit switches
    // inRollerConfig.SoftwareLimitSwitch.*

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
   */
  public static TalonFXConfiguration intakeRotaryFXConfig(double min, double max, int ccPort, double gearRatio) // TODO: needs to be updated for Intake
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
   */
  public static CANcoderConfiguration intakeRotaryCCConfig( )
  {
    CANcoderConfiguration config = new CANcoderConfiguration( );
    double kQuarterRotation = 0.25;

    config.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.25;

    if (Robot.isReal( ))
      config.MagnetSensor.MagnetOffset = (Robot.isComp( )) ? (-0.311768 - kQuarterRotation) : (0.1184 - kQuarterRotation); // TODO: Update for 2026
    else
      config.MagnetSensor.MagnetOffset = -0.25;                   // Simulated CANcoder default in rotations

    return config;
  }

  /****************************************************************************
   * 
   * Hopper roller motor - Kraken X44
   * 
   */
  public static TalonFXConfiguration hopperRollerFXConfig( )
  {
    TalonFXConfiguration hpRollerConfig = new TalonFXConfiguration( );

    // Closed Loop settings
    // hpRollerConfig.ClosedLoopGeneral.*
    // hpRollerConfig.ClosedLoopRamps.*                           // Seconds to ramp

    // Current limit settings
    hpRollerConfig.CurrentLimits.SupplyCurrentLimit = 25.0;       // Amps
    hpRollerConfig.CurrentLimits.SupplyCurrentLowerLimit = 25.0;  // Amps
    hpRollerConfig.CurrentLimits.SupplyCurrentLowerTime = 0.100;  // Seconds
    hpRollerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    hpRollerConfig.CurrentLimits.StatorCurrentLimit = 200.0;      // Amps
    hpRollerConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    // Feedback settings
    // hpRollerConfig.Feedback.*

    // Hardware limit switches - CANrange
    // hpRollerConfig.HardwareLimitSwitch.*

    // Motion Magic settings - fused CANcoder affects all feedback constants by the gearRatio
    // hpRollerConfig.MotionMagic.*

    // Motor output settings
    hpRollerConfig.MotorOutput.DutyCycleNeutralDeadband = 0.001;    // Percentage
    hpRollerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    hpRollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    // Open Loop settings
    // hpRollerConfig.OpenLoopRamps.*                               // Seconds to ramp

    // Slot settings - remote/fused CANcoder affects all feedback constants by the gearRatio
    // hpRollerConfig.Slot0.*                                       // Feedback: Voltage or duty cycle per unit of acceleration unit (velocity modes)

    // Software limit switches
    // hpRollerConfig.SoftwareLimitSwitch.*

    return hpRollerConfig;
  }

  /****************************************************************************
   * 
   * Kicker roller motor - Kraken X44
   * 
   */
  public static TalonFXConfiguration kickerRollerFXConfig( )
  {
    TalonFXConfiguration kkRollerConfig = new TalonFXConfiguration( );

    // Closed Loop settings
    // kkRollerConfig.ClosedLoopGeneral.*
    // kkRollerConfig.ClosedLoopRamps.*                           // Seconds to ramp

    // Current limit settings
    kkRollerConfig.CurrentLimits.SupplyCurrentLimit = 25.0;       // Amps
    kkRollerConfig.CurrentLimits.SupplyCurrentLowerLimit = 25.0;  // Amps
    kkRollerConfig.CurrentLimits.SupplyCurrentLowerTime = 0.100;  // Seconds
    kkRollerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    kkRollerConfig.CurrentLimits.StatorCurrentLimit = 200.0;      // Amps
    kkRollerConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    // Feedback settings
    // kkRollerConfig.Feedback.*

    // Hardware limit switches - CANrange
    // kkRollerConfig.HardwareLimitSwitch.*

    // Motion Magic settings - fused CANcoder affects all feedback constants by the gearRatio
    // kkRollerConfig.MotionMagic.*

    // Motor output settings
    kkRollerConfig.MotorOutput.DutyCycleNeutralDeadband = 0.001;    // Percentage
    kkRollerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    kkRollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    // Open Loop settings
    // kkRollerConfig.OpenLoopRamps.*                               // Seconds to ramp

    // Slot settings - remote/fused CANcoder affects all feedback constants by the gearRatio
    // kkRollerConfig.Slot0.*                                       // Feedback: Voltage or duty cycle per unit of acceleration unit (velocity modes)

    // Software limit switches
    // kkRollerConfig.SoftwareLimitSwitch.*

    return kkRollerConfig;
  }

  /****************************************************************************
   * 
   * Shooter motors - Falcon 500 (2)
   */
  public static TalonFXConfiguration shooterFXConfig( )
  {     // TODO: needs to be updated for Shooter
    TalonFXConfiguration shooterConfig = new TalonFXConfiguration( );

    // Closed Loop settings
    // inRotaryConfig.ClosedLoopGeneral.*
    // inRotaryConfig.ClosedLoopRamps.*                             // Seconds to ramp

    shooterConfig.CurrentLimits.SupplyCurrentLimit = 35.0;          // Amps
    shooterConfig.CurrentLimits.SupplyCurrentLowerLimit = 35.0;     // Amps
    shooterConfig.CurrentLimits.SupplyCurrentLowerTime = 0.001;     // Seconds
    shooterConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    // shooterConfig.CurrentLimits.StatorCurrentLimit = 100.0;      // Amps
    // shooterConfig.CurrentLimits.StatorCurrentLimitEnable = false;

    // shooterConfig.Feedback.*
    // shooterConfig.HardwareLimitSwitch.*
    // shooterConfig.MotionMagic.*

    // Motor output settings
    // shooterConfig.MotorOutput.DutyCycleNeutralDeadband = 0.001;
    shooterConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    shooterConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    // Open Loop settings
    // shooterConfig.OpenLoopRamps.*                                  // Seconds to ramp

    // Slot settings
    // shooterConfig.Slot0.GravityType = *; 
    shooterConfig.Slot0.kS = 0.0;                                     // Feedforward: Voltage or duty cycle to overcome static friction
    shooterConfig.Slot0.kG = 0.0;                                     // Feedforward: Voltage or duty cycle to overcome gravity (arbitrary feedforward)
    shooterConfig.Slot0.kV = 0.1140;                                  // Feedforward: Voltage or duty cycle per requested RPS (velocity modes)

    shooterConfig.Slot0.kP = 0.25;                                    // Voltage or duty cycle per velocity unit (velocity modes)
    shooterConfig.Slot0.kI = 0.0;                                     // Voltage or duty cycle per accumulated unit
    shooterConfig.Slot0.kD = 0.0;                                     // Voltage or duty cycle per unit of acceleration unit (velocity modes)

    // shooterConfig.SoftwareLimitSwitch.*

    return shooterConfig;
  }

  /****************************************************************************
   * 
   * Climber motors (2 - one for left and right) - Falcon 500
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
