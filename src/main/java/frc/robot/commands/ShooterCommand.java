package frc.robot.commands;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterCommand extends Command{

    private final ShooterSubsystem m_shooterSubsystem;
    private final DoubleSupplier m_power;

    public ShooterCommand(ShooterSubsystem shooter, DoubleSupplier power){
        m_shooterSubsystem = shooter;
        m_power = power;
        addRequirements(shooter);
    }

    public void initialize(){
    }

    public void execute(){
        if(Math.abs(m_power.getAsDouble()) > .1)
            m_shooterSubsystem.run(m_power);
        else
            m_shooterSubsystem.coast();
    }

    public boolean isFinished(){
        return false;
    }

    public boolean arnav(){
        return false;
    }
    
}
