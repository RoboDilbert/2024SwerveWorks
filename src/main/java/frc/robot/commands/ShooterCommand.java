package frc.robot.commands;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterCommand extends Command{

    private final ShooterSubsystem m_shooterSubsystem;
    private final DoubleSupplier m_power;
    private final DoubleSupplier m_feedPower;

    public ShooterCommand(ShooterSubsystem shooter, DoubleSupplier power, DoubleSupplier feedPower){
        m_shooterSubsystem = shooter;
        m_power = power;
        m_feedPower = feedPower;
        addRequirements(shooter);
    }

    public void initialize(){
    }

    public void execute(){
        if(Math.abs(m_power.getAsDouble()) > .1)
            m_shooterSubsystem.run(m_power);
        else
            m_shooterSubsystem.coast();

        m_shooterSubsystem.feed(m_feedPower);
    }

    public boolean isFinished(){
        return false;
    }
    
}
