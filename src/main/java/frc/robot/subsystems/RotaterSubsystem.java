package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class RotaterSubsystem extends SubsystemBase{
     
    public static CANSparkMax rotater = new CANSparkMax(Constants.SparkIDs.rotaterID, MotorType.kBrushless);
    public static RelativeEncoder rotaterEncoder = rotater.getEncoder();

    public static enum RotaterState{
        INTAKE,
        SHOOT,
        AUTO,
        RESET
    }

    public static RotaterState rotaterState = RotaterState.INTAKE;

    public RotaterSubsystem(){
        rotater.restoreFactoryDefaults();
        rotater.setIdleMode(IdleMode.kBrake);
        rotater.setSmartCurrentLimit(20);
    }

    public RotaterState getRotaterState(){
        return rotaterState;
    }
    
    public double getPosition(){
        return rotaterEncoder.getPosition();
    }

    public void toPosition(double position){
        double power = ((((getPosition() - position) / -15) - 0.05));
        if(Math.abs(power) > 0.25){
            power = 0.25 * Math.signum(power);
        }
        run(power);
    }

    public void run(double power){
        rotater.set(power);
    }
    
    public void resetPosition(){
        rotaterEncoder.setPosition(0);
    }

    public void setAuto(){
        RotaterSubsystem.rotaterState = RotaterState.AUTO;
    }

    public Command reset(){
        return runOnce(() -> RotaterSubsystem.rotaterState = RotaterState.RESET);
    }

    public void periodic(){
        SmartDashboard.putNumber("Encoder: ", rotaterEncoder.getPosition());
        SmartDashboard.putString("Rotater State: ", "" + RotaterSubsystem.rotaterState);
    }
}
