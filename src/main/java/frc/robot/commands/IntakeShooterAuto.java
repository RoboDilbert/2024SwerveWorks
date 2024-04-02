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

public class IntakeShooterAuto extends Command{

    private final FeederSubsystem m_feederSubsystem;
    private final ShooterSubsystem m_shooterSubsystem;
    private final IntakeSubsystem m_intakeSubsystem;
    private final RotaterSubsystem m_rotaterSubsystem;

    private boolean end;
    private double angle;
    private double wait;
    private boolean shooterSpin;


    public IntakeShooterAuto(FeederSubsystem feed, ShooterSubsystem shooter, IntakeSubsystem intake, RotaterSubsystem rotaterSubsystem){
        m_feederSubsystem = feed;
        m_shooterSubsystem = shooter;
        m_intakeSubsystem = intake;
        m_rotaterSubsystem = rotaterSubsystem;
        end = false;
        wait = 0;
        shooterSpin = false;
        addRequirements(feed);
    }

    public void initialize(){
        wait = 0;
        FeederSubsystem.feederState = FeederState.FEED;
        RotaterSubsystem.rotaterState = RotaterState.INTAKE;
    }

    public double evalAngle() {
        LimelightHelpers.setPipelineIndex("limelight", 0);
        angle = ((Constants.ShooterConstants.kHorizontalAngle + Constants.ShooterConstants.kGearRatio*(LimelightHelpers.getTY("limelight") + 53))) * .75;                 // *APRIL_TAG_VALUE
        //angle += Constants.ShooterConstants.kAngleDistanceMultiplier;   // *LIDAR_DISTANCE_VALUE
        //angle += Constants.ShooterConstants.kAngleSpeedMultiplier;      // *ROBOT_SPEED_Y_VALUE
        return angle;
    }

    public void execute(){
        if(IntakeSubsystem.intakeState == IntakeState.INTAKE && !shooterSpin){
            m_rotaterSubsystem.toPosition(Constants.TeleOpConstants.kRotaterIntakePosition);
        } 
        else{
            m_rotaterSubsystem.toPosition(Constants.TeleOpConstants.kSubShootPosition);
        } 

        if(m_intakeSubsystem.getDistance() < 50 && !shooterSpin){
            m_intakeSubsystem.run(1);
            FeederSubsystem.feederState = FeederState.BACK;
            IntakeSubsystem.intakeState = IntakeState.OFF;
            m_intakeSubsystem.run(0);
            m_feederSubsystem.feed(() -> .5);
            shooterSpin = true;
            m_rotaterSubsystem.toPosition(Constants.TeleOpConstants.kSubShootPosition);
        }
        if(shooterSpin){
            wait++;
        }
        if(wait > 10){
            m_rotaterSubsystem.toPosition(Constants.TeleOpConstants.kSubShootPosition);
            m_shooterSubsystem.maxSpeed();
            m_feederSubsystem.feed(() -> 0);
            end = false;
        }
    }

    public boolean isFinished(){
        return end;
    }
    
}
