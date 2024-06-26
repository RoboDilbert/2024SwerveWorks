package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterLifterSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShooterLifterSubsystem.ShooterLifterState;
import frc.robot.subsystems.ShooterSubsystem.ShooterState;

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
        SmartDashboard.putNumber("shooter pos", m_ShooterLifterSubsystem.getPosition());
        if(ShooterLifterSubsystem.shooterLifterState == ShooterLifterState.MANUAL){
            if(m_upPower.getAsDouble() > .05){
            m_ShooterLifterSubsystem.run(m_upPower.getAsDouble());
            }
            else if(m_downPower.getAsDouble() > .05){
                m_ShooterLifterSubsystem.run(-m_downPower.getAsDouble());
            }
            else{
                m_ShooterLifterSubsystem.run(0);
            }
        }
        else if(ShooterLifterSubsystem.shooterLifterState == ShooterLifterState.TRAP && ShooterSubsystem.shooterState == ShooterState.TRAP){
            if(m_ShooterLifterSubsystem.getPosition() < 70.5){
                m_ShooterLifterSubsystem.run(0.5);
            }
            else if(m_ShooterLifterSubsystem.getPosition() > 70.5){
                //m_ShooterLifterSubsystem.run(-0.75);
            }
    
            if(Math.abs(m_ShooterLifterSubsystem.getPosition() - 70.5) < 1){
                m_ShooterLifterSubsystem.run(0);
            }
        }
        else if(ShooterLifterSubsystem.shooterLifterState == ShooterLifterState.AMP && ShooterSubsystem.shooterState == ShooterState.AMP){
            if(m_ShooterLifterSubsystem.getPosition() < 42){
                m_ShooterLifterSubsystem.run(0.9);
            }
            else if(m_ShooterLifterSubsystem.getPosition() > 42){
                //m_ShooterLifterSubsystem.run(-0.75);
            }
    
            if(Math.abs(m_ShooterLifterSubsystem.getPosition() - 42) < 1){
                m_ShooterLifterSubsystem.run(0);
            }
        }
        else if(ShooterLifterSubsystem.shooterLifterState == ShooterLifterState.DOWN){
            if(m_ShooterLifterSubsystem.getPosition() > 0){
                m_ShooterLifterSubsystem.run(-.65);
            }
            else if(m_ShooterLifterSubsystem.getPosition() < 0){
                m_ShooterLifterSubsystem.run(0.65);
            }
    
            if(Math.abs(m_ShooterLifterSubsystem.getPosition() - 0) < 3){
                m_ShooterLifterSubsystem.run(0);
                ShooterLifterSubsystem.shooterLifterState = ShooterLifterState.MANUAL;
            }
        }
    }

    public boolean isFinished(){
        return false;
    }
    
}
