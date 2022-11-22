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
import frc.robot.Constants.IntakeConstants;


public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  public double kP = 5e-5; 
  public double kI = 1e-6;
  public double kD = 0; 
  public double kIz = 0; 
  public double kFF = 0.000156; 
  public double kMaxOutput = 1; 
  public double kMinOutput = -1;
  public double maxRPM = 5700;
      // Smart Motion Coefficients
  public double maxVel = 2000; // rpm
  public double minVel = 0;
  public double maxAcc = 1500;
  public double allowedErr = 10;

  private final CANSparkMax intakeMotor = new CANSparkMax(Constants.INTAKE_ID, MotorType.kBrushless);


  private RelativeEncoder intakeEncoder = intakeMotor.getEncoder();

  
  private SparkMaxPIDController intake_pidController = intakeMotor.getPIDController();

  int smartMotionSlot = 0;



  public IntakeSubsystem() {
    intakeMotor.restoreFactoryDefaults();
    intakeEncoder.setPosition(0);
    intakeMotor.setIdleMode(IdleMode.kBrake);

    intake_pidController.setP(kP);
    intake_pidController.setI(kI);
    intake_pidController.setD(kD);
    intake_pidController.setIZone(kIz);
    intake_pidController.setFF(kFF);
    intake_pidController.setOutputRange(kMinOutput, kMaxOutput);

    /**
     * Smart Motion coefficients are set on a SparkMaxPIDController object
     * 
     * - setSmartMotionMaxVelocity() will limit the velocity in RPM of
     * the pid controller in Smart Motion mode
     * - setSmartMotionMinOutputVelocity() will put a lower bound in
     * RPM of the pid controller in Smart Motion mode
     * - setSmartMotionMaxAccel() will limit the acceleration in RPM^2
     * of the pid controller in Smart Motion mode
     * - setSmartMotionAllowedClosedLoopError() will set the max allowed
     * error for the pid controller in Smart Motion mode
     */


    intake_pidController.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
    intake_pidController.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot); 
    intake_pidController.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
    intake_pidController.setSmartMotionAllowedClosedLoopError(allowedErr, smartMotionSlot); //what does this do?
    // display PID coefficients on SmartDashboard
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

  public void moveClaw(double speed){
    intakeMotor.set(speed);
  }

  public void moveClawBangBang(double speed)
  
  //bang bang control
  {
    if (speed < 0){
      if (intakeEncoder.getPosition() <= Constants.INTAKE_MIN){intakeMotor.set(0);} else{intakeMotor.set(speed);}
    }
    else if (speed > 0){
      if (intakeEncoder.getPosition() >= Constants.INTAKE_MAX){intakeMotor.set(0);} else{intakeMotor.set(speed);}
    }
    else {intakeMotor.set(speed);}
  }
  

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Claw Encoder", intakeEncoder.getPosition());
    // This method will be called once per scheduler run
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
    if((p != kP)) { intake_pidController.setP(p); kP = p; }
    if((i != kI)) { intake_pidController.setI(i); kI = i; }
    if((d != kD)) { intake_pidController.setD(d); kD = d; }
    if((iz != kIz)) { intake_pidController.setIZone(iz); kIz = iz; }
    if((ff != kFF)) { intake_pidController.setFF(ff); kFF = ff; }
    if((max != kMaxOutput) || (min != kMinOutput)) { 
      intake_pidController.setOutputRange(min, max); 
      kMinOutput = min; kMaxOutput = max;} 
    
    if((maxV != maxVel)) { intake_pidController.setSmartMotionMaxVelocity(maxV,0); maxVel = maxV; }
    if((minV != minVel)) { intake_pidController.setSmartMotionMinOutputVelocity(minV,0); minVel = minV; }
    if((maxA != maxAcc)) { intake_pidController.setSmartMotionMaxAccel(maxA,0); maxAcc = maxA; }
    if((allE != allowedErr)) { intake_pidController.setSmartMotionAllowedClosedLoopError(allE,0); allowedErr = allE; }
  
  }

  public void intakeVelocityControl (double setpointVel){
    double m_setPoint = setpointVel;
    intake_pidController.setReference(m_setPoint, CANSparkMax.ControlType.kSmartVelocity);
    double processVariable = intakeEncoder.getVelocity();

    SmartDashboard.putNumber("SetPoint", m_setPoint);
    SmartDashboard.putNumber("Process Variable", processVariable);
    SmartDashboard.putNumber("Output", intakeMotor.getAppliedOutput());

  }

  public void intakeSmartMotion (double setpointPos){
    double m_setPoint = setpointPos;
    intake_pidController.setReference(m_setPoint, CANSparkMax.ControlType.kSmartMotion);
    double processVariable = intakeEncoder.getPosition();

    SmartDashboard.putNumber("SetPoint", m_setPoint);
    SmartDashboard.putNumber("Process Variable", processVariable);
    SmartDashboard.putNumber("Output", intakeMotor.getAppliedOutput());

  }
}
