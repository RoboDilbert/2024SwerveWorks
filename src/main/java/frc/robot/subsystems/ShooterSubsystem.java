package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.playingwithfusion.TimeOfFlight;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.RotaterSubsystem.RotaterState;


public class ShooterSubsystem extends SubsystemBase{

    private CANSparkFlex shooterMotor1;
    private CANSparkMax shooterMotor2;
    private SparkPIDController shooterMotor1_PidController;
    private SparkPIDController shooterMotor2_PidController;
    private RelativeEncoder shooterMotor1_Encoder;
    private RelativeEncoder shooterMotor2_Encoder;
    
    private final double MAX_RPM = 5800;

    public static enum ShooterState{
        SUB,
        LINE,
        STAGE,
        AMP
    }

    public static enum ShooterSpeedState{
        OFF,
        MAX
    }

    public static ShooterState shooterState = ShooterState.SUB;

    public static ShooterSpeedState speedState = ShooterSpeedState.OFF;

    public ShooterSubsystem(){
        shooterMotor1 = new CANSparkFlex(Constants.SparkIDs.shooter1SparkID, MotorType.kBrushless);
        shooterMotor2 = new CANSparkMax(Constants.SparkIDs.shooter2SparkID, MotorType.kBrushless);

        shooterMotor1.restoreFactoryDefaults();
        shooterMotor2.restoreFactoryDefaults();
        shooterMotor1_PidController = shooterMotor1.getPIDController();
        shooterMotor2_PidController = shooterMotor2.getPIDController();
        shooterMotor1_Encoder = shooterMotor1.getEncoder();
        shooterMotor2_Encoder = shooterMotor2.getEncoder();
        
        shooterMotor1_PidController.setP(Constants.PID.kP);
        shooterMotor2_PidController.setP(Constants.PID.kP);
        shooterMotor1_PidController.setI(Constants.PID.kI);
        shooterMotor2_PidController.setP(Constants.PID.kP);
        shooterMotor1_PidController.setD(Constants.PID.kD);
        shooterMotor2_PidController.setP(Constants.PID.kP);
        shooterMotor1_PidController.setIZone(Constants.PID.kIz);
        shooterMotor2_PidController.setIZone(Constants.PID.kIz);
        shooterMotor1_PidController.setFF(Constants.PID.kFF);
        shooterMotor2_PidController.setFF(Constants.PID.kFF);
        shooterMotor1_PidController.setOutputRange(Constants.PID.kMinOutput, Constants.PID.kMaxOutput);
        shooterMotor2_PidController.setOutputRange(Constants.PID.kMinOutput, Constants.PID.kMaxOutput);
        shooterMotor1.setIdleMode(IdleMode.kBrake);
        shooterMotor2.setIdleMode(IdleMode.kBrake);
    }

    public void run(DoubleSupplier speed){
        shooterMotor1_PidController.setReference(speed.getAsDouble() * MAX_RPM, com.revrobotics.CANSparkBase.ControlType.kVelocity);
        shooterMotor2_PidController.setReference(-speed.getAsDouble() * MAX_RPM, com.revrobotics.CANSparkBase.ControlType.kVelocity);
    }

    public void maxSpeed(){
        //shooterMotor1_PidController.setReference(-(MAX_RPM - 1000), com.revrobotics.CANSparkBase.ControlType.kVelocity);
        //shooterMotor2_PidController.setReference(MAX_RPM, com.revrobotics.CANSparkBase.ControlType.kVelocity);
        shooterMotor1.set(-.6);
        shooterMotor2.set(.8);
    }

    public void coast(){
        shooterMotor1.stopMotor();
        shooterMotor2.stopMotor();
    }

    public double getPosition(){
        return (shooterMotor1_Encoder.getPosition() + shooterMotor2_Encoder.getPosition()) / 2;
    }

    public void setSub(){
        if(RotaterSubsystem.rotaterState == RotaterState.INTAKE){
            RotaterSubsystem.rotaterState = RotaterState.SHOOT;
        }
        shooterState = ShooterState.SUB;
    }

    public void setLine(){
        if(RotaterSubsystem.rotaterState == RotaterState.INTAKE){
            RotaterSubsystem.rotaterState = RotaterState.SHOOT;
        }
        shooterState = ShooterState.LINE;
    }

    public void setStage(){
        if(RotaterSubsystem.rotaterState == RotaterState.INTAKE){
            RotaterSubsystem.rotaterState = RotaterState.SHOOT;
        }
        shooterState = ShooterState.STAGE;
    }

    public Command toggleShooter() {
        return runOnce(
            () -> {
                if(ShooterSubsystem.speedState == ShooterSpeedState.OFF){
                    ShooterSubsystem.speedState = ShooterSpeedState.MAX;
                }
                else if(ShooterSubsystem.speedState == ShooterSpeedState.MAX){
                    ShooterSubsystem.speedState = ShooterSpeedState.OFF;
                }
            });
    }


    public void stop(){
        //ShooterSubsystem.shooterMotor.stopMotor();
        //ShooterSubsystem.shooterMotor2.stopMotor();
    }
    
    public void periodic(){
          SmartDashboard.putString("Shooter State: ", "" + ShooterSubsystem.shooterState);
    }
}
