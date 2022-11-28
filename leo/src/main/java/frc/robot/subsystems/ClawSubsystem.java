// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClawSubsystem extends SubsystemBase {
  
  //motor that powers the claw
  private final CANSparkMax clawMotor = new CANSparkMax(Constants.CLAW_MOTOR_ID,MotorType.kBrushless);
  //encoder
  public RelativeEncoder clawEncoder = clawMotor.getEncoder();
  //PID controller for the claw
  public PIDController clawPID;
  //target position of the motor
  private double setPoint;

  public ClawSubsystem() {
    //reset the encoder when the game starts
    clawEncoder.setPosition(0);
    //set the idle mode to brake
    clawMotor.setIdleMode(IdleMode.kBrake);
    clawPID = new PIDController(0, 0, 0);
  }
  
  public void setClawPosition(double position){
    setPoint = position;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    clawMotor.set(clawPID.calculate(clawEncoder.getPosition(), setPoint));
  }
}
