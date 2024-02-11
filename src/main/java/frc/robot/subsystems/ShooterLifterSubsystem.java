package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class ShooterLifterSubsystem extends SubsystemBase{
     
    public static CANSparkMax shooterLifter1 = new CANSparkMax(Constants.SparkIDs.shooterLifter1ID, MotorType.kBrushless);
    public static CANSparkMax shooterLifter2 = new CANSparkMax(Constants.SparkIDs.shooterLifter2ID, MotorType.kBrushless);
    
    public ShooterLifterSubsystem(){
        shooterLifter1.restoreFactoryDefaults();
        shooterLifter1.setIdleMode(IdleMode.kBrake);
        shooterLifter1.setSmartCurrentLimit(20);
        shooterLifter1.restoreFactoryDefaults();
        shooterLifter1.setIdleMode(IdleMode.kBrake);
        shooterLifter1.setSmartCurrentLimit(20);
    }

     public void run(DoubleSupplier power){
        shooterLifter1.set(power.getAsDouble());
        shooterLifter2.set(-power.getAsDouble());
    }

    public static void stop(){
        shooterLifter1.stopMotor();
        shooterLifter2.stopMotor();
    }
    public void periodic(){

    }
}
