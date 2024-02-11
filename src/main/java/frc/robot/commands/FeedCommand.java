package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.RotaterSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.IntakeSubsystem.IntakeState;
import frc.robot.subsystems.RotaterSubsystem.RotaterState;

public class FeedCommand extends Command{

    private final IntakeSubsystem m_intakeSubsystem;
    private final ShooterSubsystem m_shooterSubsystem;
    private final RotaterSubsystem m_rotaterSubsystem;

    public FeedCommand(IntakeSubsystem intake, ShooterSubsystem shooter, RotaterSubsystem rotater){
        m_intakeSubsystem = intake;
        m_shooterSubsystem = shooter;
        m_rotaterSubsystem = rotater;
        addRequirements(intake, shooter, rotater);
    }

    public void initialize(){
    }

    public void execute(){
        if(m_rotaterSubsystem.rotaterState == RotaterState.SHOOT){
            if(m_rotaterSubsystem.getPosition() < 1000 || m_rotaterSubsystem.getPosition() > 2000){
                if(m_rotaterSubsystem.getPosition() < 1000){
                    m_rotaterSubsystem.run(.5);
                }
                else if(m_rotaterSubsystem.getPosition() > 2000){
                    m_rotaterSubsystem.run(-.5);
                }
            }
            if(m_rotaterSubsystem.getPosition() > 1000 && m_rotaterSubsystem.getPosition() < 2000){
                m_rotaterSubsystem.rotaterState = RotaterState.INTAKE;
            }
        }
        if(m_rotaterSubsystem.rotaterState == RotaterState.INTAKE){
            m_intakeSubsystem.intakeState = IntakeState.FEED;
            m_intakeSubsystem.run(1);
            m_shooterSubsystem.feed(() -> 1);
            if(m_shooterSubsystem.shooterSensor1.getRange() == 5){
                m_intakeSubsystem.intakeState = IntakeState.OFF;
                m_intakeSubsystem.stop();
                m_shooterSubsystem.feedStop();
                m_rotaterSubsystem.rotaterState = RotaterState.SHOOT;
            }
        }
    }

    public boolean isFinished(){
        return false;
    }
    
}
