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
  //create new motor object
  private final WPI_TalonSRX motorFL = new WPI_TalonSRX (Constants.MOTOR_FL_ID);
  private final WPI_TalonSRX motorBL = new WPI_TalonSRX (Constants.MOTOR_BL_ID);
  private final WPI_TalonSRX motorFR = new WPI_TalonSRX (Constants.MOTOR_FR_ID);
  private final WPI_TalonSRX motorBR = new WPI_TalonSRX (Constants.MOTOR_BR_ID);

  DifferentialDrive differentialDrive = new DifferentialDrive(motorFL, motorFR);

  /** Creates a new DriveSubsystem. */

  public DriveSubsystem() {

    // makes the back motors follow the front motors. 
    motorBR.follow(motorFR);
    motorBL.follow(motorFL);
    // inverting motors
    motorFR.setInverted(false); //to test
    motorFL.setInverted(true); //to test
    //set invert to the respective lead
    motorBR.setInverted(InvertType.FollowMaster);
    motorBL.setInverted(InvertType.FollowMaster);
  }


  @Override
  public void periodic() {

    // This method will be called once per scheduler run
  }
  
  public void setDriveSpeedArcade (double xSpeed, double zRotation){
    //Speed limmiter 
    xSpeed = xSpeed * 0.5;
    zRotation = zRotation * 0.5;
    //deadband makes small imputs read as 0
    if (Math.abs(xSpeed) < 0.1){xSpeed = 0;}
    if (Math.abs(zRotation) < 0.1){zRotation = 0;}
     
    differentialDrive.arcadeDrive(xSpeed, zRotation);}
    

  public void setDriveSpeedTank (double leftSpeed, double rightSpeed){
  //Speed limmiter 
  leftSpeed = leftSpeed * 0.5;
  rightSpeed = rightSpeed * 0.5;
  //deadband makes small imputs read as 0
  if (Math.abs(leftSpeed) < 0.1){leftSpeed = 0;}
  if (Math.abs(rightSpeed) < 0.1){rightSpeed = 0;}
   
  differentialDrive.tankDrive(leftSpeed, rightSpeed);
  
    
  }
  

}
