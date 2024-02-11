package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IntakeSubsystem.IntakeState;

public class IntakeCommand extends Command{

    private final IntakeSubsystem m_intakeSubsystem;

    public IntakeCommand(IntakeSubsystem intake){
        m_intakeSubsystem = intake;
        addRequirements(intake);
    }

    public void initialize(){
    }

    public void execute(){
        if(m_intakeSubsystem.getState() == IntakeState.INTAKE){
            if(m_intakeSubsystem.getDistance() == 100){
                m_intakeSubsystem.intakeState = IntakeState.OFF;
                m_intakeSubsystem.stop();
            }
        }
    }

    public boolean isFinished(){
        return false;
    }
    
}
