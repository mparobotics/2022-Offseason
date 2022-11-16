// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveSubsystem extends SubsystemBase {
  /** Creates a new DriveSubsystem. */

  //create motors
  private final WPI_TalonSRX motorFL = new WPI_TalonSRX (Constants.MOTOR_FL_ID);
  private final WPI_TalonSRX motorFR = new WPI_TalonSRX (Constants.MOTOR_FR_ID);
  private final WPI_TalonSRX motorBL = new WPI_TalonSRX (Constants.MOTOR_BL_ID);
  private final WPI_TalonSRX motorBR = new WPI_TalonSRX (Constants.MOTOR_BR_ID);

  DifferentialDrive differentialDrive = new DifferentialDrive(motorFL, motorFR);

  public DriveSubsystem() {
    //makes back motors follow front
    motorBR.follow(motorFR); 
    motorBL.follow(motorFL);

    motorFL.setInverted(true); //totest
    motorFR.setInverted(false); //to  test
    //invert followers
    motorBR.setInverted(InvertType.FollowMaster);
    motorBL.setInverted(InvertType.FollowMaster);
    

  }

  public void setDriveSpeedTank (double leftSpeed, double rightSpeed) {
    //speed limiter
    leftSpeed = leftSpeed * .5;
    rightSpeed = rightSpeed * .5;

    if (Math.abs(leftSpeed) < .1) {leftSpeed = 0;}
    if (Math.abs(rightSpeed) < .1) {rightSpeed = 0;}

    differentialDrive.tankDrive(leftSpeed, rightSpeed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
