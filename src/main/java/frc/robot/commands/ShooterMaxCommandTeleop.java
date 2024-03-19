package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.RotaterSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShooterSubsystem.ShooterSpeedState;
import frc.robot.subsystems.RotaterSubsystem.RotaterState;

public class ShooterMaxCommandTeleop extends Command{

    private final ShooterSubsystem m_shooterSubsystem;

    public ShooterMaxCommandTeleop(ShooterSubsystem shooter){
        m_shooterSubsystem = shooter;
        addRequirements(shooter);
    }

    public void initialize(){

    }

    public void execute(){
        m_shooterSubsystem.maxSpeed();
    }
    
    public void end(){
        m_shooterSubsystem.coast();
        RotaterSubsystem.rotaterState = RotaterState.INTAKE;
    }

    public boolean isFinished(){
        return false;
    }
    
}
