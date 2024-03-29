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

public class AutoShootClose extends Command{

    private final FeederSubsystem m_feederSubsystem;
    private final ShooterSubsystem m_shooterSubsystem;
    private final IntakeSubsystem m_intakeSubsystem;
    private final RotaterSubsystem m_rotaterSubsystem;

    private boolean shoot;
    private double initialPosShoot;
    private boolean end;
    private double angle;
    private double wait;


    public AutoShootClose(FeederSubsystem feed, ShooterSubsystem shooter, IntakeSubsystem intake, RotaterSubsystem rotaterSubsystem){
        m_feederSubsystem = feed;
        m_shooterSubsystem = shooter;
        m_intakeSubsystem = intake;
        m_rotaterSubsystem = rotaterSubsystem;
        shoot = false;
        initialPosShoot = 0;
        end = false;
        wait = 0;
        addRequirements(feed);
    }

    public void initialize(){
        FeederSubsystem.feederState = FeederState.OFF;
        shoot = true;
        initialPosShoot = m_feederSubsystem.getFeederPosition();
    }

    public double evalAngle() {
        LimelightHelpers.setPipelineIndex("limelight", 0);
        angle = ((Constants.ShooterConstants.kHorizontalAngle + Constants.ShooterConstants.kGearRatio*(LimelightHelpers.getTY("limelight") + 53)) - 0.5) * .87;                 // *APRIL_TAG_VALUE
        //angle += Constants.ShooterConstants.kAngleDistanceMultiplier;   // *LIDAR_DISTANCE_VALUE
        //angle += Constants.ShooterConstants.kAngleSpeedMultiplier;      // *ROBOT_SPEED_Y_VALUE
        return angle;
    }

    public void execute(){
        m_rotaterSubsystem.toPosition(evalAngle());
        m_shooterSubsystem.maxSpeed();
        if(FeederSubsystem.feederState == FeederState.OFF){
            wait++;
        }
        if(wait > 20){
            m_feederSubsystem.feed(() -> -1);
            FeederSubsystem.feederState = FeederState.FEED;
        }
        if(m_feederSubsystem.getFeederPosition() < initialPosShoot - 50){
            shoot = false;
            m_feederSubsystem.feed(() -> 0);
            FeederSubsystem.feederState = FeederState.OFF;
        }
        if(FeederSubsystem.feederState == FeederState.OFF && shoot == false){
            end = true;
        }
    }

    public boolean isFinished(){
        return end;
    }
    
}
