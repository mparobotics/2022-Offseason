// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  private final CANSparkMax intakeMotor = new CANSparkMax(Constants.INTAKE_ID, MotorType.kBrushless);
  private RelativeEncoder intakeEncoder = intakeMotor.getEncoder();
  public IntakeSubsystem() {
    intakeEncoder.setPosition(0);
    intakeMotor.setIdleMode(IdleMode.kBrake);
  }

  public void moveClaw(double speed)
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
  }
}
