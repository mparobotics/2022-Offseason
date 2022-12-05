// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {
  /** Creates a new elevatorSubsystem. */
  public double kP = 5e-5;
  public double kI = 0;
  public double kD = 0;
  public double kIz = 0;
  public double kFF = .000156;
  public double kMaxOutput = 1;
  public double kMinOutput = -1;
  public double maxRPM = 5700;
  public double maxVel = 2000;
  public double minVel = 0;
  public double maxAcc = 1500;
  public double allowedErr = 10;
  int smartMotionSlot = 0;

  public double m_setPoint;

  private final CANSparkMax elevatorMotor = new CANSparkMax(Constants.ELEVATORMOTOR_ID, MotorType.kBrushless);
  private RelativeEncoder elevatorEncoder = elevatorMotor.getEncoder();
  private SparkMaxPIDController elevator_pidController = elevatorMotor.getPIDController();


  public ElevatorSubsystem() {
    elevatorMotor.restoreFactoryDefaults();
    elevatorEncoder.setPosition(0);
    elevatorMotor.setIdleMode(IdleMode.kBrake);

    elevator_pidController.setP(kP);
    elevator_pidController.setI(kI);
    elevator_pidController.setD(kD);
    elevator_pidController.setIZone(kIz);
    elevator_pidController.setFF(kFF);
    elevator_pidController.setOutputRange(kMinOutput, kMaxOutput);

    elevator_pidController.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
    elevator_pidController.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot);
    elevator_pidController.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
    elevator_pidController.setSmartMotionAllowedClosedLoopError(allowedErr, smartMotionSlot);

    SmartDashboard.putNumber("P Gain", kP);
    SmartDashboard.putNumber("I Gain", kI);
    SmartDashboard.putNumber("D Gain", kD);
    SmartDashboard.putNumber("I Zone", kIz);
    SmartDashboard.putNumber("Feed Forward", kFF);
    SmartDashboard.putNumber("Max Output", kMaxOutput);
    SmartDashboard.putNumber("Min Output", kMinOutput);

    // display Smart Motion coefficients
    SmartDashboard.putNumber("Max Velocity", maxVel);
    SmartDashboard.putNumber("Min Velocity", minVel);
    SmartDashboard.putNumber("Max Acceleration", maxAcc);
    SmartDashboard.putNumber("Allowed Closed Loop Error", allowedErr);
    SmartDashboard.putNumber("Set Position", 0);
    SmartDashboard.putNumber("Set Velocity", 0);


  }



  public void elevatorVelocityControl (double setPointVel){
    m_setPoint = setPointVel;
    elevator_pidController.setReference(m_setPoint, CANSparkMax.ControlType.kVelocity);
    double processVariable = elevatorEncoder.getVelocity();
    
    SmartDashboard.putNumber("SetPoint", m_setPoint);
    SmartDashboard.putNumber("Process Variable", processVariable);
    SmartDashboard.putNumber("Output", elevatorMotor.getAppliedOutput());
  }

  public void elevatorMotionControl (double setPointPos){
    m_setPoint = setPointPos;
    //limit the elevators height so that it's always between the MAX and MIN values
    m_setPoint = Math.min(Math.max(m_setPoint,Constants.MIN_ELEVATOR_SETPOINT),Constants.MAX_ELEVATOR_SETPOINT);
    //update the PID controller
    elevator_pidController.setReference(m_setPoint, CANSparkMax.ControlType.kPosition);
    double processVariable = elevatorEncoder.getPosition();
    //display PID values
    SmartDashboard.putNumber("SetPoint", m_setPoint);
    SmartDashboard.putNumber("Process Variable", processVariable);
    SmartDashboard.putNumber("Output", elevatorMotor.getAppliedOutput());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Claw Encoder", elevatorEncoder.getPosition());
    double p = SmartDashboard.getNumber("P Gain", 0);
    double i = SmartDashboard.getNumber("I Gain", 0);
    double d = SmartDashboard.getNumber("D Gain", 0);
    double iz = SmartDashboard.getNumber("I Zone", 0);
    double ff = SmartDashboard.getNumber("Feed Forward", 0);
    double max = SmartDashboard.getNumber("Max Output", 0);
    double min = SmartDashboard.getNumber("Min Output", 0);
    double maxV = SmartDashboard.getNumber("Max Velocity", 0);
    double minV = SmartDashboard.getNumber("Min Velocity", 0);
    double maxA = SmartDashboard.getNumber("Max Acceleration", 0);
    double allE = SmartDashboard.getNumber("Allowed Closed Loop Error", 0);

    // if PID coefficients on SmartDashboard have changed, write new values to controller
    if((p != kP)) { elevator_pidController.setP(p); kP = p; }
    if((i != kI)) { elevator_pidController.setI(i); kI = i; }
    if((d != kD)) { elevator_pidController.setD(d); kD = d; }
    if((iz != kIz)) { elevator_pidController.setIZone(iz); kIz = iz; }
    if((ff != kFF)) { elevator_pidController.setFF(ff); kFF = ff; }
    if((max != kMaxOutput) || (min != kMinOutput)) { 
      elevator_pidController.setOutputRange(min, max); 
      kMinOutput = min; kMaxOutput = max;} 

    if((maxV != maxVel)) { elevator_pidController.setSmartMotionMaxVelocity(maxV,0); maxVel = maxV; }
    if((minV != minVel)) { elevator_pidController.setSmartMotionMinOutputVelocity(minV,0); minVel = minV; }
    if((maxA != maxAcc)) { elevator_pidController.setSmartMotionMaxAccel(maxA,0); maxAcc = maxA; }
    if((allE != allowedErr)) { elevator_pidController.setSmartMotionAllowedClosedLoopError(allE,0); allowedErr = allE; }

 
  }
}

