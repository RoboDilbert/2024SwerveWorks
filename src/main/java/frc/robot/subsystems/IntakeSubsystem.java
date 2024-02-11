package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.playingwithfusion.TimeOfFlight;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class IntakeSubsystem extends SubsystemBase{
     
    public static CANSparkMax intakeMotor = new CANSparkMax(Constants.SparkIDs.intake1SparkID, MotorType.kBrushless);
    public static CANSparkMax intakeMotor2 = new CANSparkMax(Constants.SparkIDs.intake2SparkID, MotorType.kBrushless);

    public static TimeOfFlight intakeSensor1 = new TimeOfFlight(Constants.SensorConstants.intake1ID);
    public static TimeOfFlight intakeSensor2 = new TimeOfFlight(Constants.SensorConstants.intake2ID);

    public static boolean intake = false;

    public enum IntakeState{
        OFF,
        INTAKE,
        FEED
    }

    public IntakeState intakeState = IntakeState.OFF;
    
    public IntakeSubsystem(){
        intakeMotor.restoreFactoryDefaults();
        intakeMotor.setIdleMode(IdleMode.kBrake);
        intakeMotor.setSmartCurrentLimit(20);
        intakeMotor2.restoreFactoryDefaults();
        intakeMotor2.setIdleMode(IdleMode.kBrake);
        intakeMotor2.setSmartCurrentLimit(20);
    }

    public void run(double speed){
        IntakeSubsystem.intakeMotor.set(-speed);
        IntakeSubsystem.intakeMotor2.set(-speed);

    }

    public IntakeState getState(){
        return intakeState;
    }

    public double getDistance() {
        return (intakeSensor1.getRange() + intakeSensor2.getRange()) / 2;
    }

    public Command toggleIntake() {
        return runOnce(
            () -> {
                if(intakeState == IntakeState.OFF){
                    intakeState = IntakeState.INTAKE;
                    run(1);
                }
                else if(intakeState == IntakeState.INTAKE){
                    intakeState = IntakeState.OFF;
                    stop();
                }
            });
      }

    public void stop(){
        IntakeSubsystem.intakeMotor.stopMotor();
        IntakeSubsystem.intakeMotor2.stopMotor();
    }
    
    public void periodic(){

    }
}
