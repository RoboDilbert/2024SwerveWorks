package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.AlternateEncoderType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class RotaterSubsystem extends SubsystemBase{
     
    public static CANSparkMax rotater = new CANSparkMax(Constants.SparkIDs.rotaterID, MotorType.kBrushless);
    public static RelativeEncoder rotaterAbsoluteEncoder = rotater.getEncoder();

    public enum RotaterState{
        SHOOT,
        INTAKE
    }

    public RotaterState rotaterState = RotaterState.SHOOT;

    public RotaterSubsystem(){
        rotater.restoreFactoryDefaults();
        rotater.setIdleMode(IdleMode.kBrake);
        rotater.setSmartCurrentLimit(20);
    }
    
    public double getPosition(){
        return rotaterAbsoluteEncoder.getPosition();
    }

    public void run(double power){
        rotater.set(power);
    }
    
    public void periodic(){
        SmartDashboard.putNumber("Encoder: ", rotaterAbsoluteEncoder.getPosition());
    }
}
