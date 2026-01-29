// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
   // bot serial nums
  public static final String      kCompSN               = "03260A3A";
  public static final String      kPracticeSN           = "03238074";

  // Game controller definitions
  public static final int         kDriverPadPort        = 0;
  public static final int         kOperatorPadPort      = 1;

  public static final double      kStickDeadband        = 0.15;
  public static final double      kTriggerThreshold     = 0.25;

  public static final boolean     kRumbleOn             = true;
  public static final boolean     kRumbleOff            = false;
  public static final double      kRumbleIntensity      = 0.5;  // 0.0 is low, 1.0 is high

  // Phoenix firmware versions expected
  public static final int         kPhoenix6MajorVersion = 25;

  public static final String      kRobotString          = "RobotContainer";

  public static final String      kLLLeftName           = "limelight-left";
  public static final String      kLLRightName          = "limelight-right";

  // Robot physical dimensions

  public static final double      kBranchSpacing        = Units.inchesToMeters(13.0);  // Distance between branches
  public static final double      kL1Spacing            = Units.inchesToMeters(5.5);   // Distance to outside of branches
  public static final double      kRobotLength          = Units.inchesToMeters(34.5);  // Our robot length
  public static final double      kSetbackReefCoral     = kRobotLength / 2;                                       // Distance robot is set back from branch to score coral
  public static final double      kSetbackReefAlgae     = kRobotLength / 2 + Units.inchesToMeters(8.0);   // Distance robot is set back from reef to grab algae
  public static final double      kSetbackProcAlgae     = kRobotLength / 2 + Units.inchesToMeters(20.0);   // Distance robot is set back from reef to grab algae

  // Scoring poses relative to an AprilTag (X - robot setback, Y - left, center, right)

  public static final Transform2d kBranchCoralLeft      =
      new Transform2d(kSetbackReefCoral, -kBranchSpacing / 2, Rotation2d.k180deg);
  public static final Transform2d kBranchCoralCenter    = new Transform2d(kSetbackReefCoral, 0, Rotation2d.k180deg);
  public static final Transform2d kBranchCoralRight     =
      new Transform2d(kSetbackReefCoral, +kBranchSpacing / 2, Rotation2d.k180deg);
  public static final Transform2d kBranchCoralLeftL1    =
      new Transform2d(kSetbackReefCoral, (-kBranchSpacing / 2) - kL1Spacing, Rotation2d.k180deg);
  public static final Transform2d kBranchCoralRightL1   =
      new Transform2d(kSetbackReefCoral, (+kBranchSpacing / 2) + kL1Spacing, Rotation2d.k180deg);

  public static final Transform2d kBranchAquireAlgae    = new Transform2d(kSetbackReefAlgae, 0, Rotation2d.k180deg);

  /****************************************************************************
   * CAN IDs and PWM IDs
   ****************************************************************************/
  public static final class Ports
  {
    public static final String kCANCarnivore        = "canivore1";
    public static final String kCANRio              = "rio";

    // CANivore CAN IDs - Swerve
    public static final int    kCANID_DriveLF       = 1;    // Kraken X60
    public static final int    kCANID_SteerLF       = 2;    // Kraken X60
    public static final int    kCANID_CANcoderLF    = 3;    // CANcoder

    public static final int    kCANID_DriveRF       = 4;    // Kraken X60     
    public static final int    kCANID_SteerRF       = 5;    // Kraken X60
    public static final int    kCANID_CANcoderRF    = 6;    // CANcoder

    public static final int    kCANID_DriveLR       = 7;    // Kraken X60
    public static final int    kCANID_SteerLR       = 8;    // Kraken X60
    public static final int    kCANID_CANcoderLR    = 9;    // CANcoder

    public static final int    kCANID_DriveRR       = 10;   // Kraken X60
    public static final int    kCANID_SteerRR       = 11;   // Kraken X60
    public static final int    kCANID_CANcoderRR    = 12;   // CANcoder

    public static final int    kCANID_Pigeon2       = 13;   // Pigeon2 IMU

    // RoboRIO CAN IDs
    public static final int    kCANID_ElevatorLeft  = 15;   // Kraken X60
    public static final int    kCANID_ElevatorRight = 16;   // Kraken X60

    public static final int    kCANID_WristRotary   = 18;   // Kraken X60 (Manipulator)
    public static final int    kCANID_WristCANcoder = 19;   // CANcoder   (Manipulator)

    public static final int    kCANID_ClawRoller    = 21;   // Kraken X60 (Manipulator)
    public static final int    kCANID_CoralDetector = 22;   // CANrange   (Manipulator)
    public static final int    kCANID_AlgaeDetector = 23;   // CANrange   (Manipulator)

    public static final int    kCANID_CANdle        = 0;

    // Digital I/Os
    public static final int    kDIO0_ElevatorDown   = 0;    // REV Magnetic Limit Switch
  }
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
}
