package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterLifterSubsystem;
import frc.robot.subsystems.FeederSubsystem.FeederState;
import frc.robot.subsystems.IntakeSubsystem.IntakeState;
import frc.robot.subsystems.ShooterLifterSubsystem.ShooterLifterState;

public class IntakeCommand extends Command{

    private final IntakeSubsystem m_intakeSubsystem;

    public IntakeCommand(IntakeSubsystem intake){
        m_intakeSubsystem = intake;
        addRequirements(intake);
    }

    public void initialize(){
        
    }

    public void execute(){
        if(IntakeSubsystem.intakeState == IntakeState.OFF){
            m_intakeSubsystem.run(0);
        }
        else if(IntakeSubsystem.intakeState == IntakeState.INTAKE){
            IntakeSubsystem.intakeState = IntakeState.INTAKE;
            FeederSubsystem.feederState = FeederState.FEED;
            if(ShooterLifterSubsystem.shooterLifterState == ShooterLifterState.MANUAL){
                m_intakeSubsystem.run(.75);
            }
            else{
                m_intakeSubsystem.run(0);
            }
            if(m_intakeSubsystem.getDistance() < 42){
                m_intakeSubsystem.run(0);
                FeederSubsystem.feederState = FeederState.BACK;
                IntakeSubsystem.intakeState = IntakeState.OFF;
            }
        }
        else if(IntakeSubsystem.intakeState == IntakeState.SLOW){
            m_intakeSubsystem.run(.15);
            if(m_intakeSubsystem.getIntakeDistance() > 250){
                m_intakeSubsystem.run(0);
                FeederSubsystem.feederState = FeederState.SLOWER;
                IntakeSubsystem.intakeState = IntakeState.SLOWER;
            }
        }
        else if(IntakeSubsystem.intakeState == IntakeState.SLOWER){
            m_intakeSubsystem.run(.05);
            if(m_intakeSubsystem.getDistance() < 54){
                m_intakeSubsystem.run(0);
                FeederSubsystem.feederState = FeederState.OFF;
                IntakeSubsystem.intakeState = IntakeState.OFF;
            }
        }
        else if(IntakeSubsystem.intakeState == IntakeState.REVERSE){
            m_intakeSubsystem.run(-1);
        }
    }

    public void end(){
    }

    public boolean isFinished(){
        return false;
    }
    
}
