package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.RotaterSubsystem;
import frc.robot.subsystems.FeederSubsystem.FeederState;
import frc.robot.subsystems.IntakeSubsystem.IntakeState;
import frc.robot.subsystems.RotaterSubsystem.RotaterState;

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
            m_intakeSubsystem.run(.75);
            if(m_intakeSubsystem.getDistance() < 40){
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
