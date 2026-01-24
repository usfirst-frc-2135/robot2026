// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.DriveConstants.LEFT_FOLLOWER_ID;
import static frc.robot.Constants.DriveConstants.LEFT_LEADER_ID;
import static frc.robot.Constants.DriveConstants.RIGHT_FOLLOWER_ID;
import static frc.robot.Constants.DriveConstants.RIGHT_LEADER_ID;

import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CANDriveSubsystem extends SubsystemBase
{
  private final WPI_TalonSRX      leftFront  = new WPI_TalonSRX(LEFT_LEADER_ID);
  private final WPI_TalonSRX      leftBack   = new WPI_TalonSRX(LEFT_FOLLOWER_ID);
  private final WPI_TalonSRX      rightFront = new WPI_TalonSRX(RIGHT_LEADER_ID);
  private final WPI_TalonSRX      rightBack  = new WPI_TalonSRX(RIGHT_FOLLOWER_ID);
  private final DifferentialDrive drive      = new DifferentialDrive(leftFront, rightFront);

  // These represent our regular encoder objects, which we would
  // create to use on a real robot.

  public CANDriveSubsystem( )
  {
    leftFront.configFactoryDefault( );
    leftBack.configFactoryDefault( );
    rightFront.configFactoryDefault( );
    rightBack.configFactoryDefault( );
    leftBack.follow(leftFront);
    rightBack.follow(rightFront);

    rightFront.setInverted(true);

    // rightSide.setInverted(true);

    // this.leftFront.setSafetyEnabled(true);
    // this.leftFront.setExpiration(250.0);
    // this.rightFront.setSafetyEnabled(true);
    // this.rightFront.setExpiration(250.0);

    TalonSRXConfiguration config = new TalonSRXConfiguration( );
  }

  public void periodic( )
  {}

  public void driveArcade(double xSpeed, double zRotation)
  {
    drive.arcadeDrive(xSpeed, zRotation);
  }
}
