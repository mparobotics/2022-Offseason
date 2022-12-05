// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class AutoDriveForward extends CommandBase {
 
  private final DriveSubsystem m_DriveSubsystem;
  private final Double m_forwardSpeed;
  private final Double m_distance;
  private final Double m_time;
  private final Double m_startTime;
  
  public AutoDriveForward(DriveSubsystem driveSub,Double speed, Double distance, Double time) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_forwardSpeed = speed;
    this.m_distance = distance;
    this.m_time = time;
    this.m_startTime = Timer.getFPGATimestamp();
    
    m_DriveSubsystem = driveSub;

    addRequirements(driveSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_DriveSubsystem.setDriveSpeedArcade(m_forwardSpeed, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Timer.getFPGATimestamp() - m_startTime) > m_time;
  }
}
