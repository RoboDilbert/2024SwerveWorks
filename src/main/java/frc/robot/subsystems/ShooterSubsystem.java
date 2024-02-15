package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.playingwithfusion.TimeOfFlight;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class ShooterSubsystem extends SubsystemBase{

    private CANSparkFlex shooterMotor1;
    private CANSparkFlex shooterMotor2;
    private CANSparkMax shooterFeeder1;
    private CANSparkMax shooterFeeder2;
    private SparkPIDController shooterMotor1_PidController;
    private SparkPIDController shooterMotor2_PidController;
    private RelativeEncoder shooterMotor1_Encoder;
    private RelativeEncoder shooterMotor2_Encoder;
    public TimeOfFlight shooterSensor1 = new TimeOfFlight(Constants.SensorConstants.shooterSensor1ID);
    public TimeOfFlight shooterSensor2 = new TimeOfFlight(Constants.SensorConstants.shooterSensor2ID);

    private final double MAX_RPM = 5000;

    public ShooterSubsystem(){
        shooterMotor1 = new CANSparkFlex(Constants.SparkIDs.shooter1SparkID, MotorType.kBrushless);
        shooterMotor2 = new CANSparkFlex(Constants.SparkIDs.shooter2SparkID, MotorType.kBrushless);

        shooterFeeder1 = new CANSparkMax(Constants.SparkIDs.shooterFeeder1ID, MotorType.kBrushless);
        shooterFeeder2 = new CANSparkMax(Constants.SparkIDs.shooterFeeder2ID, MotorType.kBrushless);

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
        shooterMotor1.setIdleMode(IdleMode.kCoast);
        shooterMotor1.setIdleMode(IdleMode.kCoast);
    }

    public void run(DoubleSupplier speed){
        shooterMotor1_PidController.setReference(speed.getAsDouble() * MAX_RPM, com.revrobotics.CANSparkBase.ControlType.kVelocity);
        shooterMotor2_PidController.setReference(-speed.getAsDouble() * MAX_RPM, com.revrobotics.CANSparkBase.ControlType.kVelocity);
    }

    public void maxSpeed(){
        shooterMotor1_PidController.setReference(MAX_RPM, com.revrobotics.CANSparkBase.ControlType.kVelocity);
        shooterMotor2_PidController.setReference(-MAX_RPM, com.revrobotics.CANSparkBase.ControlType.kVelocity);
    }

    public void feed(DoubleSupplier speed){
        shooterFeeder1.set(speed.getAsDouble());
        shooterFeeder2.set(-speed.getAsDouble());
    }

    public void feedStop(){
        shooterFeeder1.stopMotor();
        shooterFeeder2.stopMotor();
    }

    public void coast(){
        shooterMotor1.stopMotor();
        shooterMotor2.stopMotor();
    }

    public double getPosition(){
        return (shooterMotor1_Encoder.getPosition() + shooterMotor2_Encoder.getPosition()) / 2;
    }

    public void stop(){
        //ShooterSubsystem.shooterMotor.stopMotor();
        //ShooterSubsystem.shooterMotor2.stopMotor();
    }
    
    public void periodic(){
        
    }
}
