package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.FeederSubsystem.FeederState;

public class FeederCommand extends Command{

    private final FeederSubsystem m_feederSubsystem;

    private boolean backSpin;
    private boolean shoot;
    private double initialPos;

    public FeederCommand(FeederSubsystem feed){
        m_feederSubsystem = feed;
        backSpin = false;
        shoot = false;
        initialPos = 0;
        addRequirements(feed);
    }

    public void initialize(){
    }

    public void execute(){
        if(FeederSubsystem.feederState == FeederState.OFF){
            m_feederSubsystem.feed(() -> 0);
        }
        else if(FeederSubsystem.feederState == FeederState.FEED){
            m_feederSubsystem.feed(() -> -.75);
        }
        else if(FeederSubsystem.feederState == FeederState.BACK){
            if(!backSpin){
                backSpin = true;
                initialPos = m_feederSubsystem.getFeederPosition();
                m_feederSubsystem.feed(() -> .25);
            }
            if(backSpin){
                if(m_feederSubsystem.getFeederPosition() > initialPos + 5){
                    backSpin = false;
                    m_feederSubsystem.feed(() -> 0);
                    FeederSubsystem.feederState = FeederState.OFF;
                }
            }
        }
        else if(FeederSubsystem.feederState == FeederState.SHOOT){
            if(!shoot){
                shoot = true;
                initialPos = m_feederSubsystem.getFeederPosition();
                m_feederSubsystem.feed(() -> -1);
            }
            if(shoot){
                if(m_feederSubsystem.getFeederPosition() < initialPos - 200){
                    shoot = false;
                    m_feederSubsystem.feed(() -> 0);
                    FeederSubsystem.feederState = FeederState.OFF;
                }
            }
        }
    }

    public boolean isFinished(){
        return false;
    }
    
}
