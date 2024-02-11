package frc.robot.commands;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterLifterSubsystem;

public class ShooterLifterCommand extends Command{

    private final ShooterLifterSubsystem m_ShooterLifterSubsystem;
    private final DoubleSupplier m_upPower;
    private final DoubleSupplier m_downPower;

    public ShooterLifterCommand(ShooterLifterSubsystem shooterLifter, DoubleSupplier upPower, DoubleSupplier downPower){
        m_ShooterLifterSubsystem = shooterLifter;
        m_upPower = upPower;
        m_downPower = downPower;
        addRequirements(shooterLifter);
    }

    public void initialize(){
    }

    public void execute(){
        if(m_upPower.getAsDouble() > .05){
            m_ShooterLifterSubsystem.run(m_upPower);
        }
        else if(m_downPower.getAsDouble() > .05){
            m_ShooterLifterSubsystem.run(()-> -m_downPower.getAsDouble());
        }
        else{
            m_ShooterLifterSubsystem.run(() -> 0);
        }
    }

    public boolean isFinished(){
        return false;
    }
    
}
