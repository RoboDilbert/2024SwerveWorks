package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LifterSubsystem;

public class LifterCommand extends Command{

    private final LifterSubsystem m_lifterSubsystem;
    private final DoubleSupplier m_leftPower;
    private final DoubleSupplier m_rightPower;
    private final DoubleSupplier m_bothPower;

    public LifterCommand(LifterSubsystem lifter, DoubleSupplier leftPower, DoubleSupplier rightPower, DoubleSupplier bothPower){
        m_lifterSubsystem = lifter;
        m_leftPower = leftPower;
        m_rightPower = rightPower;
        m_bothPower = bothPower;
        addRequirements(lifter);
    }

    public void initialize(){
    }

    public void execute(){
        SmartDashboard.putNumber("Lifter 1", m_lifterSubsystem.getOne());
        SmartDashboard.putNumber("Lifter 2", m_lifterSubsystem.getTwo());
        if(Math.abs(m_leftPower.getAsDouble()) > .05 || Math.abs(m_rightPower.getAsDouble()) > .05){
            if(Math.abs(m_leftPower.getAsDouble()) > .05){
                if(m_lifterSubsystem.getOne() > -280 && m_lifterSubsystem.getOne() < 10){
                    m_lifterSubsystem.runOne(m_leftPower);
                }
                else if(m_lifterSubsystem.getOne() < -280 && m_leftPower.getAsDouble() > 0){
                    m_lifterSubsystem.runOne(m_leftPower);
                }
                else if(m_lifterSubsystem.getOne() > 10 && m_leftPower.getAsDouble() < 0){
                    m_lifterSubsystem.runOne(m_leftPower);
                }
                else{
                    m_lifterSubsystem.runOne(() -> 0);       
                }
            }
            if(Math.abs(m_rightPower.getAsDouble()) > .05){
                if(m_lifterSubsystem.getTwo() < 250 && m_lifterSubsystem.getTwo() > -10){
                    m_lifterSubsystem.runTwo(m_rightPower);
                }
                else if(m_lifterSubsystem.getTwo() > 250 && m_rightPower.getAsDouble() > 0){
                    m_lifterSubsystem.runTwo(m_rightPower);
                }
                else if(m_lifterSubsystem.getTwo() < -10 && m_rightPower.getAsDouble() < 0){
                    m_lifterSubsystem.runTwo(m_rightPower);
                }
                else{
                    m_lifterSubsystem.runTwo(() -> 0);       
                }
            }
        }
        else if(Math.abs(m_bothPower.getAsDouble()) > .05){
            if(m_lifterSubsystem.getOne() > -280 && m_lifterSubsystem.getTwo() < 250){
                m_lifterSubsystem.run(m_bothPower, m_bothPower);
            }
            else if(m_bothPower.getAsDouble() > 0){
                m_lifterSubsystem.run(m_bothPower, m_bothPower);
            }
            else{
                m_lifterSubsystem.run(() -> 0, () -> 0);
            }
        }
        else{
            m_lifterSubsystem.run(() -> 0, () -> 0);
        }
    }

    public boolean isFinished(){
        return false;
    }
    
}
