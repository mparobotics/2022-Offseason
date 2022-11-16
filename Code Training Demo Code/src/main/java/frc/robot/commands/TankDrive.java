// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class TankDrive extends CommandBase {
  /** Creates a new TankDrive. */
  private final DriveSubsystem m_driveSubsystem;
  private final DoubleSupplier m_xSpeed;
  private final DoubleSupplier m_ySpeed;
  
  public TankDrive(DriveSubsystem driveSub, DoubleSupplier xSpeed, DoubleSupplier ySpeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_driveSubsystem = driveSub;
    this.m_xSpeed = xSpeed;
    this.m_ySpeed = ySpeed;
    addRequirements(driveSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_driveSubsystem.setDriveSpeedTank(
      m_xSpeed.getAsDouble(),
      m_ySpeed.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
