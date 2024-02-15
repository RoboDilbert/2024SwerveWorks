package frc.robot.commands;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.RotaterSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class RotaterIntakeCommand extends Command{

    private final RotaterSubsystem m_rotaterSubsystem;
    private final DoubleSupplier m_power;

    public RotaterIntakeCommand(RotaterSubsystem rotater, DoubleSupplier power){
        m_rotaterSubsystem = rotater;
        m_power = power;
        addRequirements(rotater);
    }

    public void initialize(){

    }

    public void execute(){
        //if(m_rotaterSubsystem.getPosition() > Constants)
        double power = (((m_rotaterSubsystem.getPosition() - Constants.TeleOpConstants.kRotaterIntakePosition) / -15) -.05);
        if(Math.abs(power) > 0.2){
            power = 0.2 * Math.signum(power);
        }
        m_rotaterSubsystem.run(power);
        //m_rotaterSubsystem.run(m_power);
        
    }

    public boolean isFinished(){
        return false;
    }
    
}
