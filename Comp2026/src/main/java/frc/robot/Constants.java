// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{
  // bot serial nums
  public static final String  kCompSN               = "03220587";
  public static final String  kPracticeSN           = "032B1F7E";

  // Game controller definitions
  public static final int     kDriverPadPort        = 0;
  public static final int     kOperatorPadPort      = 1;

  public static final double  kStickDeadband        = 0.15;
  public static final double  kTriggerThreshold     = 0.25;

  public static final boolean kRumbleOn             = true;
  public static final boolean kRumbleOff            = false;
  public static final double  kRumbleIntensity      = 0.5;    // 0.0 is low, 1.0 is high

  // Phoenix firmware versions expected
  public static final int     kPhoenix6MajorVersion = 26;

  public static final String  kRobotString          = "RobotContainer";

  public static final String  kLLFrontName          = "limelight-front";
  public static final String  kLLBackName           = "limelight-back";

  // Robot physical dimensions
  public static final double  kChassisLength        = 26.0;                                     // Length (and width) of chassis frame
  public static final double  kFastenerAllowance    = 0.25;                                     // Gap from chassis frame to bumper backing
  public static final double  kFramePerimeter       = kChassisLength + 2 * kFastenerAllowance;  // Official frame perimeter
  public static final double  kBumperGap            = 0.25;                                     // Gap from chassis frame to bumper backing for fabric
  public static final double  kBumperBacking        = 0.75;                                     // Thickness of bumper backing plywood
  public static final double  kBumperPadding        = 2.5;                                      // Thickness of foam padding      
  public static final double  kBumperTotal          = kBumperBacking + kBumperPadding;
  public static final double  kRobotLength          = Units.inchesToMeters(kFramePerimeter + 2 * kBumperGap + 2 * kBumperTotal); // Our robot length

  /****************************************************************************
   * CAN IDs and PWM IDs
   ****************************************************************************/
  public static final class Ports
  {
    public static final String kCANCarnivore         = "canivore1";
    public static final String kCANRio               = "rio";

    // CANivore CAN IDs - Swerve
    public static final int    kCANID_DriveLF        = 1; // Kraken X60
    public static final int    kCANID_SteerLF        = 2; // Kraken X60
    public static final int    kCANID_CANcoderLF     = 3; // CANcoder

    public static final int    kCANID_DriveRF        = 4; // Kraken X60
    public static final int    kCANID_SteerRF        = 5; // Kraken X60
    public static final int    kCANID_CANcoderRF     = 6; // CANcoder

    public static final int    kCANID_DriveLR        = 7; // Kraken X60
    public static final int    kCANID_SteerLR        = 8; // Kraken X60
    public static final int    kCANID_CANcoderLR     = 9; // CANcoder

    public static final int    kCANID_DriveRR        = 10; // Kraken X60
    public static final int    kCANID_SteerRR        = 11; // Kraken X60
    public static final int    kCANID_CANcoderRR     = 12; // CANcoder

    public static final int    kCANID_Pigeon2        = 13; // Pigeon2 IMU

    // RoboRIO CAN IDs
    public static final int    kCANID_IntakeRoller   = 15; // Kraken X44 (Intake)
    public static final int    kCANID_IntakeRotary   = 16; // Kraken X44 (Intake rotary)
    public static final int    kCANID_IntakeCANcoder = 17; // CANcoder (intake)

    public static final int    kCANID_HopperRoller   = 19; // Kraken X44 (Hopper) 

    public static final int    kCANID_KickerRoller   = 21; // Kraken X44 (Kicker)
    public static final int    kCANID_LauncherLeft   = 22; // Kraken X60 (Launcher)
    public static final int    kCANID_LauncherRight  = 23; // Kraken X60 (Launcher)

    public static final int    kCANID_ClimberLeft    = 25; // Kraken X60 (Climber telescope)
    public static final int    kCANID_ClimberRight   = 26; // Kraken X60 (Climber telescope)

    public static final int    kDIO0_FuelDetected    = 28; // CANrange (Hopper). // TODO: Do we need one?

    public static final int    kCANID_CANdle         = 0;
  }

  public static class OperatorConstants
  {
    public static final int kDriverControllerPort = 0;
  }

  /****************************************************************************
   * Intake subsystem constants
   ****************************************************************************/
  public static final class INConsts
  {
    /** Intake roller modes */
    public enum INRollerMode
    {
      STOP,    // Stop all rotation
      ACQUIRE, // Speed for acquiring a game piece
      EXPEL,   // Speed for expelling a game piece
      HOLD     // Maintain existing speed setting
    }
  }

  /****************************************************************************
   * Hopper subsystem constants
   ****************************************************************************/
  public static final class HPConsts
  {
    /** Hopper roller modes */
    public enum HPRollerMode
    {
      STOP,     // Stop all rotation
      ACQUIRE,  // Speed for moving a game piece toward launcher
      EXPEL,    // Speed for moving a game piece toward intake (to expel)
      HOLD      // Maintain existing speed setting
    }
  }

  /****************************************************************************
   * Kicker subsystem constants
   ****************************************************************************/
  public static final class KKConsts
  {
    /** Kicker roller modes */
    public enum KKRollerMode
    {
      STOP,     // Stop all rotation
      ACQUIRE,  // Speed for moving a game piece toward launcher
      EXPEL,    // Speed for moving a game piece toward intake (to expel)
      HOLD      // Maintain existing speed setting
    }
  }

  /****************************************************************************
   * Launcher subsystem constants
   ****************************************************************************/
  public static final class SHConsts
  {

  }

  /****************************************************************************
   * Vision (Limelight) constants
   ****************************************************************************/
  public static final class VIConsts
  {
    public static final AprilTagFields      kGameField = AprilTagFields.k2026RebuiltWelded;
    public static final AprilTagFieldLayout kATField   = AprilTagFieldLayout.loadField(kGameField);
  }

  /****************************************************************************
   * LED (CANdle) subsystem constants
   ****************************************************************************/
  public static final class LEDConsts
  {
    /** LED color to be used */
    public enum COLOR
    {
      OFF, // CANdle off
      WHITE, // CANdle white
      RED, // CANdle red
      ORANGE, // CANdle orange
      YELLOW, // CANdle yellow
      GREEN, // CANdle green
      BLUE, // CANdle blue
      PURPLE, // CANdle purple
      DASHBOARD // CANdle color taken from dashboard
    }

    /** LED animation to be used */
    public enum ANIMATION
    {
      SOLID, // Clears animation for solid colors (including OFF)
      COLORFLOW, // Single color flow through string
      FIRE, // Fire pattern from one end of string
      LARSON, // Ping-pong pattern bouncing between string ends
      RAINBOW, // Fading rainbow colors
      RGBFADE, // Fading red, then green, then blue
      SINGLEFADE, // Fading with a single color
      STROBE, // Strobe flashing with a single color
      TWINKLE, // Twinkles leds on
      TWINKLEOFF, // Twinkles leds off
      DASHBOARD // Animation taken from the dashboard
    }
  }
}
