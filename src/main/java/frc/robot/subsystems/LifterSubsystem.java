package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterLifterSubsystem.ShooterLifterState;


public class LifterSubsystem extends SubsystemBase{
     
    public CANSparkMax lifter1 = new CANSparkMax(Constants.SparkIDs.lifter1ID, MotorType.kBrushless);
    public CANSparkMax lifter2 = new CANSparkMax(Constants.SparkIDs.lifter2ID, MotorType.kBrushless);

    public RelativeEncoder lifter1Encoder = lifter1.getEncoder();
    public RelativeEncoder lifter2Encoder = lifter2.getEncoder();

    public static enum LifterState{
        MANUAL,
        UP
    }

    public static LifterState lifterState = LifterState.MANUAL;

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

    public void runOne(DoubleSupplier power){
        lifter1.set(power.getAsDouble());
    }

    public void runTwo(DoubleSupplier power){
        lifter2.set(-power.getAsDouble());
    }

    public void stop(){
        lifter1.stopMotor();
        lifter2.stopMotor();
    }
    
    public double getOne(){
        return lifter1Encoder.getPosition();
    }

    public double getTwo(){
        return lifter2Encoder.getPosition();
    }

    public Command lifterUpTrap(){
        return runOnce(() -> LifterSubsystem.lifterState = LifterState.UP);
    }

    public void periodic(){
        SmartDashboard.putString("Lifter State: ", "" + LifterSubsystem.lifterState);
    }
}
