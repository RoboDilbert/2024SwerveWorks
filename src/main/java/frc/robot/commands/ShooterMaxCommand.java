package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.ShooterSubsystem;

public class ShooterMaxCommand extends Command{

    private final ShooterSubsystem m_shooterSubsystem;

    public ShooterMaxCommand(ShooterSubsystem shooter){
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
    }

    public boolean isFinished(){
        return false;
    }
    
}
