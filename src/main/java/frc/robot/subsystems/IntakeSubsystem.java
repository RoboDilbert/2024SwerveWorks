package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.playingwithfusion.TimeOfFlight;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.FeederSubsystem.FeederState;
import frc.robot.subsystems.RotaterSubsystem.RotaterState;


public class IntakeSubsystem extends SubsystemBase{
     
    public static CANSparkMax intakeMotor = new CANSparkMax(Constants.SparkIDs.intake1SparkID, MotorType.kBrushless);
    public static CANSparkMax intakeMotor2 = new CANSparkMax(Constants.SparkIDs.intake2SparkID, MotorType.kBrushless);

    public static TimeOfFlight intakeSensor1 = new TimeOfFlight(Constants.SensorConstants.intake1ID);
    public static TimeOfFlight intakeSensor2 = new TimeOfFlight(Constants.SensorConstants.intake2ID);

    public static boolean intake = false;

    public static enum IntakeState{
        OFF,
        INTAKE,
        FEED,
        REVERSE,
        SLOW,
        SLOWER
    }

    public static IntakeState intakeState = IntakeState.OFF;
    
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
        return intakeSensor1.getRange();
    }

    public double getIntakeDistance() {
        return intakeSensor2.getRange();
    }

    public void stop(){
        IntakeSubsystem.intakeMotor.stopMotor();
        IntakeSubsystem.intakeMotor2.stopMotor();
    }

    public Command toggleIntake() {
        return runOnce(
            () -> {
                if(IntakeSubsystem.intakeState == IntakeState.OFF || IntakeSubsystem.intakeState == IntakeState.REVERSE){
                    IntakeSubsystem.intakeState = IntakeState.INTAKE;
                    RotaterSubsystem.rotaterState = RotaterState.INTAKE;
                }
                else if(IntakeSubsystem.intakeState == IntakeState.INTAKE){
                    IntakeSubsystem.intakeState = IntakeState.OFF;
                    FeederSubsystem.feederState = FeederState.OFF;
                }
            });
      }

    public Command reverse(){
        return runOnce(
            () -> {
                if(IntakeSubsystem.intakeState == IntakeState.OFF || IntakeSubsystem.intakeState == IntakeState.INTAKE){
                    IntakeSubsystem.intakeState = IntakeState.REVERSE;     
                }
                else if(IntakeSubsystem.intakeState == IntakeState.REVERSE){
                    IntakeSubsystem.intakeState = IntakeState.OFF;
                }
            });
    }

    public Command autoIntake(){
        return runOnce(
            () -> {
                    IntakeSubsystem.intakeState = IntakeState.INTAKE;
                    FeederSubsystem.feederState = FeederState.FEED;
                    run(.75);
            });
    }
    
    public void periodic(){
        SmartDashboard.putString("Intake State: ", "" + IntakeSubsystem.intakeState);
        SmartDashboard.putNumber("Intake Sensor: ", intakeSensor1.getRange());
        SmartDashboard.putNumber("Intake Sensor 2", intakeSensor2.getRange());
        SmartDashboard.putNumber("Intake Motor Current", intakeMotor.getOutputCurrent());
        SmartDashboard.putNumber("Intake Motor 2 Current", intakeMotor2.getOutputCurrent());

    }
}
