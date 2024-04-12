package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.FeederSubsystem.FeederState;
import frc.robot.subsystems.IntakeSubsystem.IntakeState;
import frc.robot.subsystems.RotaterSubsystem;
import frc.robot.subsystems.RotaterSubsystem.RotaterState;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class FeedBackCommand extends Command{

    private final FeederSubsystem m_feederSubsystem;

    private boolean end;
    private double wait;
    private double initialPosShoot;

    public FeedBackCommand(FeederSubsystem feed){
        m_feederSubsystem = feed;
        end = false;
        wait = 0;
        addRequirements(feed);
    }

    public void initialize(){
        FeederSubsystem.feederState = FeederState.OFF;
        initialPosShoot = m_feederSubsystem.getFeederPosition();
        m_feederSubsystem.feed(() -> .5);
    }

    public void execute(){
        wait++;
        if(wait > 4){
            m_feederSubsystem.feed(() -> 0);
            end = true;
        }
    }

    public boolean isFinished(){
        return end;
    }

}