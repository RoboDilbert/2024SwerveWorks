package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class LifterSubsystem extends SubsystemBase{
     
    public static CANSparkMax lifter1 = new CANSparkMax(Constants.SparkIDs.lifter1ID, MotorType.kBrushless);
    public static CANSparkMax lifter2 = new CANSparkMax(Constants.SparkIDs.lifter2ID, MotorType.kBrushless);
    
    public LifterSubsystem(){
        lifter1.restoreFactoryDefaults();
        lifter1.setIdleMode(IdleMode.kBrake);
        lifter1.setSmartCurrentLimit(20);
        lifter2.restoreFactoryDefaults();
        lifter2.setIdleMode(IdleMode.kBrake);
        lifter2.setSmartCurrentLimit(20);
    }

    public void run(DoubleSupplier power1, DoubleSupplier power2){
        lifter1.set(power1.getAsDouble());
        lifter2.set(-power2.getAsDouble());
    }

    public static void stop(){
        lifter1.stopMotor();
        lifter2.stopMotor();
    }
    
    public void periodic(){

    }
}
