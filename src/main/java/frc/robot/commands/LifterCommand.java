package frc.robot.commands;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LifterSubsystem;

public class LifterCommand extends Command{

    private final LifterSubsystem m_lifterSubsystem;
    private final DoubleSupplier m_leftPower;
    private final DoubleSupplier m_rightPower;

    public LifterCommand(LifterSubsystem lifter, DoubleSupplier leftPower, DoubleSupplier rightPower){
        m_lifterSubsystem = lifter;
        m_leftPower = leftPower;
        m_rightPower = rightPower;
        addRequirements(lifter);
    }

    public void initialize(){
    }

    public void execute(){
        if(Math.abs(m_leftPower.getAsDouble()) > .05 || Math.abs(m_rightPower.getAsDouble()) > .05){
            m_lifterSubsystem.run(m_leftPower, m_rightPower);
        }
        else{
            m_lifterSubsystem.run(() -> 0, () -> 0);
        }
    }

    public boolean isFinished(){
        return false;
    }
    
}
