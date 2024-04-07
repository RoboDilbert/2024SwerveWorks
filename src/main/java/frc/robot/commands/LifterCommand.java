package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LifterSubsystem;
import frc.robot.subsystems.LifterSubsystem.LifterState;
import frc.robot.subsystems.ShooterLifterSubsystem;
import frc.robot.subsystems.ShooterLifterSubsystem.ShooterLifterState;

public class LifterCommand extends Command{

    private final LifterSubsystem m_lifterSubsystem;
    private final DoubleSupplier m_leftPower;
    private final DoubleSupplier m_rightPower;
    private final DoubleSupplier m_bothPower;

    private boolean done1;
    private boolean done2;

    public LifterCommand(LifterSubsystem lifter, DoubleSupplier leftPower, DoubleSupplier rightPower, DoubleSupplier bothPower){
        m_lifterSubsystem = lifter;
        m_leftPower = leftPower;
        m_rightPower = rightPower;
        m_bothPower = bothPower;
        done1 = false;
        done2 = false;
        addRequirements(lifter);
    }

    public void initialize(){
    }

    public void execute(){
        SmartDashboard.putNumber("Lifter 1", m_lifterSubsystem.getOne());
        SmartDashboard.putNumber("Lifter 2", m_lifterSubsystem.getTwo());

        if(Math.abs(m_bothPower.getAsDouble()) > .25){
            LifterSubsystem.lifterState = LifterState.MANUAL;
        }
        if(LifterSubsystem.lifterState == LifterState.MANUAL){
            if(Math.abs(m_leftPower.getAsDouble()) > .05 || Math.abs(m_rightPower.getAsDouble()) > .05){
            m_lifterSubsystem.run(m_leftPower, m_rightPower);
            }
            else if(Math.abs(m_bothPower.getAsDouble()) > .05){
                m_lifterSubsystem.run(m_bothPower, m_bothPower);
            }
            else{
                m_lifterSubsystem.run(() -> 0, () -> 0);
            }

            if(ShooterLifterSubsystem.shooterLifterState == ShooterLifterState.TRAP){
                m_lifterSubsystem.run(() -> -.04, () -> -.04);
            }
        }
        else if(LifterSubsystem.lifterState == LifterState.UP){
            if(m_lifterSubsystem.getOne() > 42){
                m_lifterSubsystem.runOne(() -> 0);
            }
            else if(m_lifterSubsystem.getOne() < 42){
                m_lifterSubsystem.runOne(() -> .3);
            }

            if(m_lifterSubsystem.getTwo() < -42){
                m_lifterSubsystem.runTwo(() -> 0);
            }
            else if(m_lifterSubsystem.getTwo() > -42){
                m_lifterSubsystem.runTwo(() -> .3);
            }

            if(m_lifterSubsystem.getTwo() < -41 && m_lifterSubsystem.getOne() > 41){
                LifterSubsystem.lifterState = LifterState.MANUAL;
                m_lifterSubsystem.runOne(() -> 0);
                m_lifterSubsystem.runTwo(() -> 0);
            }
        }
    }

    public boolean isFinished(){
        return false;
    }
    
}
