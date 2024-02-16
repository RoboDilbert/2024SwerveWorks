package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.RotaterSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.RotaterSubsystem.RotaterState;
import frc.robot.subsystems.ShooterSubsystem.ShooterState;

public class RotaterCommand extends Command{

    private final RotaterSubsystem m_rotaterSubsystem;

    public RotaterCommand(RotaterSubsystem rotater){
        m_rotaterSubsystem = rotater;
        addRequirements(rotater);
    }

    public void initialize(){

    }

    public void execute(){
        if(RotaterSubsystem.rotaterState == RotaterState.INTAKE){
            m_rotaterSubsystem.toPosition(Constants.TeleOpConstants.kRotaterIntakePosition);
        }
        else if(RotaterSubsystem.rotaterState == RotaterState.SHOOT){
            if(ShooterSubsystem.shooterState == ShooterState.SUB){
                m_rotaterSubsystem.toPosition(Constants.TeleOpConstants.kSubShootPosition);
            }
            else if(ShooterSubsystem.shooterState == ShooterState.LINE){
                m_rotaterSubsystem.toPosition(Constants.TeleOpConstants.kLineShootPosition);
            }
            else if(ShooterSubsystem.shooterState == ShooterState.STAGE){
                m_rotaterSubsystem.toPosition(Constants.TeleOpConstants.kStageShootPosition);
            }
            
        }
    }

    public boolean isFinished(){
        return false;
    }
    
}
