package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class FeederSubsystem extends SubsystemBase{
     
    private CANSparkMax shooterFeeder1;
    private CANSparkMax shooterFeeder2;
    private RelativeEncoder feeder1Encoder;
    

    public static enum FeederState{
        OFF,
        FEED,
        BACK,
        SHOOT,
        AMP,
        SLOW,
        SLOWER,
        FORWARD
    }

    public static FeederState feederState = FeederState.OFF;

    public FeederSubsystem(){
        shooterFeeder1 = new CANSparkMax(Constants.SparkIDs.shooterFeeder1ID, MotorType.kBrushless);
        shooterFeeder2 = new CANSparkMax(Constants.SparkIDs.shooterFeeder2ID, MotorType.kBrushless);

        feeder1Encoder = shooterFeeder1.getEncoder();
    }

    public void feed(DoubleSupplier speed){
        shooterFeeder1.set(speed.getAsDouble());
        shooterFeeder2.set(-speed.getAsDouble());
    }

    public void feedStop(){
        shooterFeeder1.stopMotor();
        shooterFeeder2.stopMotor();
    }

    public double getFeederPosition(){
        return feeder1Encoder.getPosition();
    }

    public Command back(){
        return runOnce(() -> feederState = FeederState.BACK);
    }

    public Command shootUpOBlock(){
        return runOnce(() -> feederState = FeederState.SHOOT);
    }

    public Command feedPlease(){
        return runOnce(() -> {
                feed(() -> -0.75);
                FeederSubsystem.feederState = FeederState.FEED;
            }
            );
    }

    public Command shootUpAmp(){
        return runOnce(() -> feederState = FeederState.AMP);
    }

    public void periodic(){
        SmartDashboard.putString("Feeder State: ", "" + FeederSubsystem.feederState);
        SmartDashboard.putNumber("Feeder Pos: ", getFeederPosition());
    }
}
