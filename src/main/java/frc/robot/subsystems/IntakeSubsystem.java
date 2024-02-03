package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class IntakeSubsystem extends SubsystemBase{
     
    public static CANSparkMax intakeMotor = new CANSparkMax(Constants.SparkIDs.intake1SparkID, MotorType.kBrushless);
    public static CANSparkMax intakeMotor2 = new CANSparkMax(Constants.SparkIDs.intake2SparkID, MotorType.kBrushless);
    public static boolean intake = false;
    
    public IntakeSubsystem(){
        intakeMotor.restoreFactoryDefaults();
        intakeMotor.setIdleMode(IdleMode.kBrake);
        intakeMotor.setSmartCurrentLimit(20);
        intakeMotor2.restoreFactoryDefaults();
        intakeMotor2.setIdleMode(IdleMode.kBrake);
        intakeMotor2.setSmartCurrentLimit(20);
    }

    public void run(double speed){
        IntakeSubsystem.intakeMotor.set(speed);
        IntakeSubsystem.intakeMotor2.set(-speed);

    }

    public void toggle(){
        if(!intake){
            run(1.0);
            intake = true;
        }
        else{
            stop();
            intake = false;
        }
    }

    public Command toggleIntake() {
        return runOnce(
            () -> {
                toggle();
            });
      }

    public static void stop(){
        IntakeSubsystem.intakeMotor.stopMotor();
        IntakeSubsystem.intakeMotor2.stopMotor();
    }
    
    public void periodic(){
        SmartDashboard.putBoolean("intake test", intake);
    }
}
