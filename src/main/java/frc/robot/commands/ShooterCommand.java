package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.ShooterSubsystem;

public class ShooterCommand extends Command{

   
    public ShooterCommand(ShooterSubsystem shooter){
        addRequirements(shooter);
    }

    public void initialize(){

    }

    public static void run(double speed){
        //ShooterSubsystem.shooterMotor.set(speed);
        //ShooterSubsystem.shooterMotor2.set(0.5);

    }

    public static void stop(){
        //ShooterSubsystem.shooterMotor.stopMotor();
        //ShooterSubsystem.shooterMotor2.stopMotor();
    }

    public boolean isFinished(){
        return false;
    }
    
}
