// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class ArcadeDrive extends CommandBase {
    /** Creates a new TankDrive. */


  private final DriveSubsystem m_driveSubsystem;
  private final DoubleSupplier m_xSpeed;
  private final DoubleSupplier m_zRotation;
  
  

  public ArcadeDrive(DriveSubsystem driveSub, DoubleSupplier xSpeed, DoubleSupplier zRotation) {
  
    // Use addRequirements() here to declare subsystem dependencies.
    m_driveSubsystem = driveSub;
    this.m_xSpeed = xSpeed;
    this.m_zRotation = zRotation;
    addRequirements(driveSub);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_driveSubsystem.setDriveSpeedArcade(m_xSpeed.getAsDouble(), m_zRotation.getAsDouble());
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
