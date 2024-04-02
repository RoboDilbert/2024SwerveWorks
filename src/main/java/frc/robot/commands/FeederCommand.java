package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.FeederSubsystem.FeederState;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShooterSubsystem.ShooterSpeedState;

public class FeederCommand extends Command{

    private final FeederSubsystem m_feederSubsystem;

    private boolean backSpin;
    private boolean shoot;
    private double initialPosBack;
    private double initialPosShoot;

    public FeederCommand(FeederSubsystem feed){
        m_feederSubsystem = feed;
        backSpin = false;
        shoot = false;
        initialPosBack = 0;
        initialPosShoot = 0;
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
            backSpin = false;
        }
        else if(FeederSubsystem.feederState == FeederState.SLOW){
            m_feederSubsystem.feed(() -> -.15);
        }
        else if(FeederSubsystem.feederState == FeederState.SLOWER){
            m_feederSubsystem.feed(() -> -.05);
        }
        else if(FeederSubsystem.feederState == FeederState.BACK){
            if(!backSpin){
                backSpin = true;
                initialPosBack = m_feederSubsystem.getFeederPosition();
                m_feederSubsystem.feed(() -> .125);
            }
            if(backSpin){
                m_feederSubsystem.feed(() -> .125);
                if(m_feederSubsystem.getFeederPosition() > initialPosBack + 4){
                    backSpin = false;
                    m_feederSubsystem.feed(() -> 0);
                    FeederSubsystem.feederState = FeederState.OFF;
                    shoot = false;
                }
            }
        }
        else if(FeederSubsystem.feederState == FeederState.SHOOT){
            if(!shoot){
                shoot = true;
                initialPosShoot = m_feederSubsystem.getFeederPosition();
                m_feederSubsystem.feed(() -> -1);
            }
            if(shoot){
                m_feederSubsystem.feed(() -> -1);
                if(m_feederSubsystem.getFeederPosition() < initialPosShoot - 200){
                    shoot = false;
                    m_feederSubsystem.feed(() -> 0);
                    FeederSubsystem.feederState = FeederState.OFF;
                    ShooterSubsystem.speedState = ShooterSpeedState.OFF;
                }
            }
        }
        else if(FeederSubsystem.feederState == FeederState.AMP){
            if(!shoot){
                shoot = true;
                initialPosShoot = m_feederSubsystem.getFeederPosition();
                m_feederSubsystem.feed(() -> 1);
            }
            if(shoot){
                if(m_feederSubsystem.getFeederPosition() > initialPosShoot + 50){
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
