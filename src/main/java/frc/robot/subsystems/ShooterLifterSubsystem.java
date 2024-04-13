package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class ShooterLifterSubsystem extends SubsystemBase{
     
    public static CANSparkMax shooterLifter1 = new CANSparkMax(Constants.SparkIDs.shooterLifter1ID, MotorType.kBrushless);
    public static RelativeEncoder shooterEncoder = shooterLifter1.getEncoder();

    public static enum ShooterLifterState{
        MANUAL,
        AMP,
        TRAP,
        DOWN
    }

    public static ShooterLifterState shooterLifterState = ShooterLifterState.MANUAL;

    public ShooterLifterSubsystem(){
        shooterLifter1.restoreFactoryDefaults();
        shooterLifter1.setIdleMode(IdleMode.kBrake);
        shooterLifter1.setSmartCurrentLimit(20);
    }

     public void run(double power){
        shooterLifter1.set(power);
    }

    public double getPosition(){
        return shooterEncoder.getPosition();
    }

    public void toPosition(double position){
        SmartDashboard.putNumber("pos", getPosition());
        double power = ((((getPosition() - position) / -5.0)));
        if(Math.abs(power) > 0.5){
            power = 0.5 * Math.signum(power);
        }
        if(Math.abs(getPosition() - position) < 0.5){
            power = 0.05;
        }
        SmartDashboard.putNumber("shooter lifterpwoer", power);
        run(power);
    }

    public Command trap(){
        return runOnce(() -> ShooterLifterSubsystem.shooterLifterState = ShooterLifterState.TRAP);
    }

    public Command amp(){
        return runOnce(() -> ShooterLifterSubsystem.shooterLifterState = ShooterLifterState.AMP);
    }

    public Command resetShooterLifter(){
        return runOnce(() -> shooterEncoder.setPosition(0));
    }

    public static void stop(){
        shooterLifter1.stopMotor();
    }
}
