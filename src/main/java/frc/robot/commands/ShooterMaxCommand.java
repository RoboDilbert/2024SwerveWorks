package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.RotaterSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterMaxCommand extends Command{

    private final ShooterSubsystem m_shooterSubsystem;
    private final RotaterSubsystem m_rotaterSubsystem;
    private final DoubleSupplier m_power;

    public ShooterMaxCommand(ShooterSubsystem shooter, RotaterSubsystem rotater, DoubleSupplier power){
        m_shooterSubsystem = shooter;
        m_rotaterSubsystem = rotater;
        m_power = power;
        addRequirements(shooter);
    }

    public void initialize(){
    }

    public void execute(){
        m_shooterSubsystem.maxSpeed();
        m_shooterSubsystem.feed(m_power);
        double power = (((m_rotaterSubsystem.getPosition() - Constants.TeleOpConstants.kRotaterIntakePosition) / -15) -.05);
        if(Math.abs(power) > 0.2){
            power = 0.2 * Math.signum(power);
        }
        m_rotaterSubsystem.run(power);
    }
    
    public void end(){
        m_shooterSubsystem.coast();
    }

    public boolean isFinished(){
        return false;
    }
    
}
