// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {
  private final static WPI_TalonFX falconBL = new WPI_TalonFX(DriveConstants.FALCON_BL_ID); //Declares and initializes falcons
  private final static WPI_TalonFX falconFL = new WPI_TalonFX(DriveConstants.FALCON_FL_ID); 
  private final static WPI_TalonFX falconBR = new WPI_TalonFX(DriveConstants.FALCON_BR_ID); 
  private final static WPI_TalonFX falconFR = new WPI_TalonFX(DriveConstants.FALCON_FR_ID); 

  DifferentialDrive differentialDrive = new DifferentialDrive(falconFL, falconFR);

  //private final TalonFXConfiguration fxConfig = new TalonFXConfiguration(); we might need this

  /** Creates a new ExampleSubsystem. */
  public DriveSubsystem() {
    falconBR.follow(falconFR); //talonBR follows TalonFR
    falconBL.follow(falconFL); //talonBL follows TalonFR 


     /* [3] flip values so robot moves forward when stick-forward/LEDs-green */
    falconFR.setInverted(TalonFXInvertType.Clockwise); // !< Update this
    falconFL.setInverted(TalonFXInvertType.CounterClockwise); // !< Update this

    /*
    * set the invert of the followers to match their respective master controllers
    */
    falconBR.setInverted(InvertType.FollowMaster);
    falconBL.setInverted(InvertType.FollowMaster);

    /*sets the falcons to brakemode */
    setBrake();


    //setting ramp up time to full power. prevents tipping but causes input delay sluggish feeling
    falconFR.configOpenloopRamp(0.4); // 0.5 seconds from neutral to full output (during open-loop control)
    //falconFR.configClosedloopRamp(0.1); // 0 disables ramping (during closed-loop control)
    
    falconFL.configOpenloopRamp(0.4); // 0.5 seconds from neutral to full output (during open-loop control)
    //falconFL.configClosedloopRamp(0.1); // 0 disables ramping (during closed-loop control)
    
    falconBL.configOpenloopRamp(0.4); // 0.5 seconds from neutral to full output (during open-loop control)
    //falconBL.configClosedloopRamp(0.1); // 0 disables ramping (during closed-loop control)
    
    falconBR.configOpenloopRamp(0.4); // 0.5 seconds from neutral to full output (during open-loop control)
    //falconBR.configClosedloopRamp(0.1); // 0 disables ramping (during closed-loop control)


  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setDriveSpeed_Arcade(double xSpeed, double zRotation) {
    xSpeed = xSpeed * .85; //speed reducers
    zRotation = zRotation * .9; //speed reducers change per robot

    if (Math.abs(xSpeed) < .1) {xSpeed = 0;}//deadzones, pervents stick drift or small movements from effecting robot motion
    if (Math.abs(zRotation) < .1) {zRotation = 0;}//deadzones
  

    differentialDrive.arcadeDrive(xSpeed, zRotation);
  }

  public void setBrake() {
    //setting coast or brake mode, can also be done in Phoenix tuner
    falconFR.setNeutralMode(NeutralMode.Brake);
    falconFL.setNeutralMode(NeutralMode.Brake);
    falconBR.setNeutralMode(NeutralMode.Brake);
    falconBL.setNeutralMode(NeutralMode.Brake);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
