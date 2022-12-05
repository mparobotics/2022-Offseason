// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ElevatorSubsystem;

public class raiseElevator extends CommandBase {
  /** Creates a new raiseElevator. */
  private final ElevatorSubsystem m_elevatorSubsystem;
  public raiseElevator(ElevatorSubsystem elevSub) {
    m_elevatorSubsystem = elevSub;

    addRequirements(elevSub);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_elevatorSubsystem.elevatorMotionControl(m_elevatorSubsystem.m_setPoint + Constants.ELEVATOR_INCREMENT_AMOUNT);
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
