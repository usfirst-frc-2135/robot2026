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
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CANDriveSubsystem extends SubsystemBase
{
  private final WPI_TalonSRX      leftFront  = new WPI_TalonSRX(LEFT_LEADER_ID);
  private final WPI_TalonSRX      leftBack   = new WPI_TalonSRX(LEFT_FOLLOWER_ID);;
  private final WPI_TalonSRX      rightFront = new WPI_TalonSRX(RIGHT_LEADER_ID);
  private final WPI_TalonSRX      rightBack  = new WPI_TalonSRX(RIGHT_FOLLOWER_ID);
  MotorControllerGroup            leftSide   = new MotorControllerGroup(leftFront, leftBack);
  MotorControllerGroup            rightSide  = new MotorControllerGroup(rightFront, rightBack);
  private final DifferentialDrive drive      = new DifferentialDrive(leftSide, rightSide);

  // These represent our regular encoder objects, which we would
  // create to use on a real robot.

  public CANDriveSubsystem( )
  {
    leftFront.configFactoryDefault( );
    leftBack.configFactoryDefault( );
    rightFront.configFactoryDefault( );
    rightBack.configFactoryDefault( );

    rightSide.setInverted(true);

    // this.leftFront.setSafetyEnabled(true);

    this.leftFront.setExpiration(250.0);
    this.leftBack.setSafetyEnabled(true);
    this.leftBack.setExpiration(250.0);
    this.rightFront.setSafetyEnabled(true);
    this.rightFront.setExpiration(250.0);
    this.rightBack.setSafetyEnabled(true);
    this.rightBack.setExpiration(250.0);
    this.rightFront.setInverted(true);

    TalonSRXConfiguration config = new TalonSRXConfiguration( );
    config.peakCurrentLimit = 60;
    config.continuousCurrentLimit = 40;
    config.peakCurrentDuration = 1000;
  }

  public void periodic( )
  {}

  public void driveArcade(double xSpeed, double zRotation)
  {
    this.drive.arcadeDrive(xSpeed, zRotation);
  }
}
