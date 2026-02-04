
// Phoenix 6 configurations

package frc.robot.lib.phoenix;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitSourceValue;
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
    inRotaryConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine; // Feedforward: Mechanism is an arm and needs cosine
    inRotaryConfig.Slot0.kS = 0.0;                                  // Feedforward: Voltage or duty cylce to overcome static friction
    inRotaryConfig.Slot0.kG = -0.50;                                // Feedforward: Voltage or duty cylce to overcome gravity (arbitrary feedforward)
    inRotaryConfig.Slot0.kV = 0.1129;                               // Feedforward: Voltage or duty cycle per requested RPS (velocity modes)

    inRotaryConfig.Slot0.kP = 3.6 * gearRatio;                      // Feedback: Voltage or duty cycle per velocity unit (velocity modes)
    inRotaryConfig.Slot0.kI = 0.0 * gearRatio;                      // Feedback: Voltage or duty cycle per accumulated unit
    inRotaryConfig.Slot0.kD = 0.0 * gearRatio;                      // Feedback: Voltage or duty cycle per unit of acceleration unit (velocity modes)

    // Software limit switches
    inRotaryConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = min;  // Rotations
    // inRotaryConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    inRotaryConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = max;  // Rotations
    // inRotaryConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;

    return inRotaryConfig;
  }

  /****************************************************************************
   * 
   * Intake rotary CANcoder
   */
  public static CANcoderConfiguration intakeRotaryCancoderConfig( )
  {
    CANcoderConfiguration config = new CANcoderConfiguration( );
    double kQuarterRotation = 0.25;
    double CompRobotOffset = -0.015;

    config.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.25;
    if (Robot.isReal( ))
      config.MagnetSensor.MagnetOffset =
          (Robot.isComp( )) ? (-0.311768 - kQuarterRotation + CompRobotOffset) : (0.1184 - kQuarterRotation);
    else
      config.MagnetSensor.MagnetOffset = -0.25;                   // Simulated CANcoder default in rotations

    return config;
  }
  // Swerve module configs are built into swerve subsystem

  /****************************************************************************
   * 
   * Shooter motors - Falcon 500 (2)
   */
  public static TalonFXConfiguration shooterFXConfig() {     // TODO: needs to be updated for Shooter
    TalonFXConfiguration shooterConfig = new TalonFXConfiguration();

    // Closed Loop settings
    // inRotaryConfig.ClosedLoopGeneral.*
    // inRotaryConfig.ClosedLoopRamps.* // Seconds to ramp

    shooterConfig.CurrentLimits.SupplyCurrentLimit = 35.0; // Amps
    shooterConfig.CurrentLimits.SupplyCurrentLowerLimit = 35.0; // Amps
    shooterConfig.CurrentLimits.SupplyCurrentLowerTime = 0.001; // Seconds
    shooterConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    // shooterConfig.CurrentLimits.StatorCurrentLimit = 100.0; // Amps
    // shooterConfig.CurrentLimits.StatorCurrentLimitEnable = false;

    // shooterConfig.Feedback.*
    // shooterConfig.HardwareLimitSwitch.*
    // shooterConfig.MotionMagic.*

    // Motor output settings
    // shooterConfig.MotorOutput.DutyCycleNeutralDeadband = 0.001;
    shooterConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    shooterConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    // Open Loop settings
    // shooterConfig.OpenLoopRamps.* // Seconds to ramp

    // Slot settings
    // shooterConfig.Slot0.GravityType = *; // Feedforward: Mechanism is an elevator
    // or arm
    shooterConfig.Slot0.kS = 0.0; // Feedforward: Voltage or duty cylce to overcome static friction
    shooterConfig.Slot0.kG = 0.0; // Feedforward: Voltage or duty cylce to overcome gravity (arbitrary
                                  // feedforward)
    shooterConfig.Slot0.kV = 0.1140; // Feedforward: Voltage or duty cycle per requested RPS (velocity modes)

    shooterConfig.Slot0.kP = 0.25; // Voltage or duty cycle per velocity unit (velocity modes)
    shooterConfig.Slot0.kI = 0.0; // Voltage or duty cycle per accumulated unit
    shooterConfig.Slot0.kD = 0.0; // Voltage or duty cycle per unit of acceleration unit (velocity modes)

    // shooterConfig.SoftwareLimitSwitch.*

    return shooterConfig;
  }

  /****************************************************************************
   * 
   * Manipulator wrist rotary motor - Kraken X60
   * 
   * @param min
   *                  minimum angle of rotation
   * @param max
   *                  maximum angle of rotation (must be greater than min)
   * @param ccPort
   *                  CANcoder port number
   * @param gearRatio
   *                  gear box ratio
   */
  public static TalonFXConfiguration wristRotaryFXConfig(double min, double max, int ccPort, double gearRatio) {
    TalonFXConfiguration wristRotaryConfig = new TalonFXConfiguration();

    // Closed Loop settings
    // wristRotaryConfig.ClosedLoopGeneral.*
    // wristRotaryConfig.ClosedLoopRamps.* // Seconds to ramp

    // Current limit settings
    wristRotaryConfig.CurrentLimits.SupplyCurrentLimit = 40.0; // Amps
    wristRotaryConfig.CurrentLimits.SupplyCurrentLowerLimit = 35.0; // Amps
    wristRotaryConfig.CurrentLimits.SupplyCurrentLowerTime = 0.050; // Seconds
    wristRotaryConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    wristRotaryConfig.CurrentLimits.StatorCurrentLimit = 400.0; // Amps
    wristRotaryConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    // Feedback settings
    wristRotaryConfig.Feedback.FeedbackRemoteSensorID = ccPort;
    wristRotaryConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    wristRotaryConfig.Feedback.SensorToMechanismRatio = 1.0;
    wristRotaryConfig.Feedback.RotorToSensorRatio = gearRatio;

    // Hardware limit switches - NONE
    // wristRotaryConfig.HardwareLimitSwitch.*

    // Motion Magic settings - fused CANcoder affects all feedback constants by the
    // gearRatio
    wristRotaryConfig.MotionMagic.MotionMagicCruiseVelocity = 82.17 / gearRatio; // 1.67 Rotations / second
    wristRotaryConfig.MotionMagic.MotionMagicAcceleration = 950.0 / gearRatio; // 19.3 Rotations / second ^ 2
    wristRotaryConfig.MotionMagic.MotionMagicJerk = 0 / gearRatio; // 500 Rotations / second ^ 3

    // Motor output settings
    wristRotaryConfig.MotorOutput.DutyCycleNeutralDeadband = 0.001; // Percentage
    wristRotaryConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    wristRotaryConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // Open Loop settings
    // wristRotaryConfig.OpenLoopRamps.* // Seconds to ramp

    // Slot settings - remote/fused CANcoder affects all feedback constants by the
    // gearRatio
    // Wrist Upward was x.x V, Elevator Downward was x.x.
    // kG = (0.40 + 0.25) / 2
    // kS = (0.40 - 0.25) / 2
    wristRotaryConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine; // Feedforward: Mechanism is an arm and needs
                                                                       // cosine
    wristRotaryConfig.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;
    wristRotaryConfig.Slot0.kS = 0.05; // Feedforward: Voltage or duty cycle to overcome static friction (Measured
                                       // 0.7V)
    wristRotaryConfig.Slot0.kG = -0.25; // Feedforward: Voltage or duty cycle to overcome gravity (arbitrary
                                        // feedforward) (Measured -0.18 to -0.32)
    wristRotaryConfig.Slot0.kV = 0.1241; // Feedforward: Voltage or duty cycle per requested RPS (velocity modes)

    // NOTE: Motion Magic settings are scaled by gear ration when using a
    // FusecCANCoder
    wristRotaryConfig.Slot0.kP = 1.62 * gearRatio; // 79.75 Feedback: Voltage or duty cycle per velocity unit (velocity
                                                   // modes)
    wristRotaryConfig.Slot0.kI = 0.0 * gearRatio; // 0.0 Feedback: Voltage or duty cycle per accumulated unit
    wristRotaryConfig.Slot0.kD = 0.0609 * gearRatio; // 2.998 Feedback: Voltage or duty cycle per unit of acceleration
                                                     // unit (velocity modes)

    // Software limit switches
    wristRotaryConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = min; // Rotations
    wristRotaryConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    wristRotaryConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = max; // Rotations
    wristRotaryConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;

    return wristRotaryConfig;
  }

  /****************************************************************************
   * 
   * Manipulator wrist rotary CANcoder
   */
  public static CANcoderConfiguration wristRotaryCANcoderConfig() {
    CANcoderConfiguration ccConfig = new CANcoderConfiguration();

    ccConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    ccConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.25;
    if (Robot.isReal( ))
      ccConfig.MagnetSensor.MagnetOffset = -0.5345; //TODO: Update for 2026
    else
      ccConfig.MagnetSensor.MagnetOffset = -0.25; // Simulated CANcoder default in rotations

    return ccConfig;
  }

  /****************************************************************************
   * 
   * Manipulator claw roller motor - Kraken X60
   * 
   * @param min
   *                  minimum angle of rotation
   * @param max
   *                  maximum angle of rotation (must be greater than min)
   * @param ccPort
   *                  CANcoder port number
   * @param gearRatio
   *                  gear box ratio
   */
  public static TalonFXConfiguration clawRollerFXConfig(int canRangeID) {
    TalonFXConfiguration clawRollerConfig = new TalonFXConfiguration();

    // Closed Loop settings
    // clawRollerConfig.ClosedLoopGeneral.*
    // clawRollerConfig.ClosedLoopRamps.* // Seconds to ramp

    // Current limit settings
    clawRollerConfig.CurrentLimits.SupplyCurrentLimit = 25.0; // Amps
    clawRollerConfig.CurrentLimits.SupplyCurrentLowerLimit = 25.0; // Amps
    clawRollerConfig.CurrentLimits.SupplyCurrentLowerTime = 0.100; // Seconds
    clawRollerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    clawRollerConfig.CurrentLimits.StatorCurrentLimit = 200.0; // Amps
    clawRollerConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    // Feedback settings
    // clawRollerConfig.Feedback.*

    // Hardware limit switches - CANrange
    clawRollerConfig.HardwareLimitSwitch.ReverseLimitRemoteSensorID = canRangeID; // Stop coral on CANrange detection
    clawRollerConfig.HardwareLimitSwitch.ReverseLimitSource = ReverseLimitSourceValue.RemoteCANrange;
    clawRollerConfig.HardwareLimitSwitch.ReverseLimitEnable = true;

    // Motion Magic settings - fused CANcoder affects all feedback constants by the
    // gearRatio
    // clawRollerConfig.MotionMagic.*

    // Motor output settings
    clawRollerConfig.MotorOutput.DutyCycleNeutralDeadband = 0.001; // Percentage
    clawRollerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    clawRollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // Open Loop settings
    // clawRollerConfig.OpenLoopRamps.* // Seconds to ramp

    // Slot settings - remote/fused CANcoder affects all feedback constants by the
    // gearRatio
    // clawRollerConfig.Slot0.* // Feedback: Voltage or duty cycle per unit of
    // acceleration unit (velocity modes)
    // Slot settings - remote/fused CANcoder affects all feedback constants by the
    // gearRatio
    // Wrist Upward was x.x V, Elevator Downward was x.x.
    // kG = (0.40 + 0.25) / 2
    // kS = (0.40 - 0.25) / 2
    // clawRollerConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine; //
    // Feedforward: Mechanism is an arm and needs cosine
    // clawRollerConfig.Slot0.kS = 0.0; // Feedforward: Voltage or duty cycle to
    // overcome static friction
    // clawRollerConfig.Slot0.kG = 0.0; // Feedforward: Voltage or duty cycle to
    // overcome gravity (arbitrary feedforward)
    // clawRollerConfig.Slot0.kV = 0.1241; // Feedforward: Voltage or duty cycle per
    // requested RPS (velocity modes)

    // clawRollerConfig.Slot0.kP = 0.9; // Feedback: Voltage or duty cycle per
    // velocity unit (velocity modes)
    // clawRollerConfig.Slot0.kI = 0.0; // Feedback: Voltage or duty cycle per
    // accumulated unit
    // clawRollerConfig.Slot0.kD = 0.0; // Feedback: Voltage or duty cycle per unit
    // of acceleration unit (velocity modes)

    // Software limit switches
    // clawRollerConfig.SoftwareLimitSwitch.*

    return clawRollerConfig;
  }

  /****************************************************************************
   * 
   * Manipulator coral CANRange detector
   */
  public static CANrangeConfiguration coralCANRangeConfig() {
    CANrangeConfiguration crConfig = new CANrangeConfiguration();

    crConfig.ProximityParams.ProximityThreshold = 0.1; // Proximity distance in meters (about 8 inches)
    crConfig.ToFParams.UpdateMode = UpdateModeValue.ShortRange100Hz;

    return crConfig;
  }

  /****************************************************************************
   * 
   * Manipulator algae CANRange detector
   */
  public static CANrangeConfiguration algaeCANRangeConfig() {
    CANrangeConfiguration crConfig = new CANrangeConfiguration();

    crConfig.ProximityParams.ProximityThreshold = 0.1; // Proximity distance in meters (about 4 inches)
    crConfig.ToFParams.UpdateMode = UpdateModeValue.ShortRange100Hz;

    return crConfig;
  }

}
