package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.AnalogInput;


public class SwerveModule {
    private final WPI_TalonFX driveMotor;
    private final WPI_TalonFX turningMotor; 

    private final AnalogInput absoluteEncoder;
    private final boolean absoluteEncoderReversed;
    private final double absoluteEncoderOffsetRad; //todo get offsets


    public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed,
           int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed){
        
            this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
            this.absoluteEncoderReversed = absoluteEncoderReversed;
            absoluteEncoder = new AnalogInput(absoluteEncoderId);

            driveMotor = new WPI_TalonFX(driveMotorId);
            turningMotor = new WPI_TalonFX(turningMotorId);

            driveMotor.setInverted(driveMotorReversed);
            turningMotor.setInverted(turningMotorReversed);

            driveMotor.se

            


    }
 
}
