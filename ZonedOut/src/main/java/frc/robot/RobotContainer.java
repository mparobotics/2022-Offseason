// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;



import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.ElevatorGround;
import frc.robot.commands.ElevatorHigh;
import frc.robot.commands.ElevatorLow;
import frc.robot.commands.ElevatorMid;
import frc.robot.commands.ElevatorReset;

import frc.robot.commands.OverrideElevatorMax;
import frc.robot.commands.OverrideElevatorMin;
import frc.robot.commands.lowerElevator;
import frc.robot.commands.raiseElevator;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ElevatorSubsystem m_elevatorSubsystem = new ElevatorSubsystem();
  private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();


  
  public static Joystick box = new Joystick(Constants.BOX_ID);

  public static XboxController xbox = new XboxController(Constants.CONTROLLER_PORT);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    //arcade drive
    m_driveSubsystem.setDefaultCommand(new ArcadeDrive(m_driveSubsystem, () -> xbox.getRightX() ,() -> -xbox.getLeftY()));
    
    
    
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    if(Constants.ELEVATOR_IS_PID){
      //buttons to control the PID version of the elevator

      //buttons for moving the elevator to each of the four positions
      new JoystickButton(box, 0).whenPressed(new ElevatorGround(m_elevatorSubsystem));
      new JoystickButton(box, 1).whenPressed(new ElevatorLow(m_elevatorSubsystem));
      new JoystickButton(box, 2).whenPressed(new ElevatorMid(m_elevatorSubsystem));
      new JoystickButton(box, 3).whenPressed(new ElevatorHigh(m_elevatorSubsystem));

      //raise elevator with Y button and lower with B button
      new JoystickButton(xbox, Button.kY.value).whenPressed(new raiseElevator(m_elevatorSubsystem));
      new JoystickButton(xbox, Button.kB.value).whenPressed(new lowerElevator(m_elevatorSubsystem));
    }
    else{
      //buttons to control the not PID version of the elevator
      
      //button 1 raises the elevator
      new JoystickButton(box, 1).whenHeld(new raiseElevator(m_elevatorSubsystem));
      //button 3 lowers the elevator
      new JoystickButton(box, 3).whenHeld(new lowerElevator(m_elevatorSubsystem));
      //button 2 raises the elevator at a slower speed and overrides the maximum position
      new JoystickButton(box, 2).whenHeld(new OverrideElevatorMax(m_elevatorSubsystem));
      //button 4 lowers the elevator at a slower speed and overrides the minimum position
      new JoystickButton(box, 4).whenHeld(new OverrideElevatorMin(m_elevatorSubsystem));
      //button 5 resets the elevator's encoder to 0 rotations
      new JoystickButton(box, 5).whenHeld(new ElevatorReset(m_elevatorSubsystem));
    }
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return null;
  }
}
