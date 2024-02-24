package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.RotaterSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.RotaterSubsystem.RotaterState;
import frc.robot.subsystems.ShooterSubsystem.ShooterState;

public class RotaterCommand extends Command{

    private final RotaterSubsystem m_rotaterSubsystem;
    public double angle;

    public RotaterCommand(RotaterSubsystem rotater){
        m_rotaterSubsystem = rotater;
        addRequirements(rotater);
    }

    public void initialize(){

    }

    public double evalAngle() {
        angle = Constants.ShooterConstants.kHorizontalAngle;
        LimelightHelpers.setPipelineIndex("limelight", 0);
        angle += Constants.ShooterConstants.kGearRatio*(LimelightHelpers.getTY("limelight") + 53);                 // *APRIL_TAG_VALUE
        //angle += Constants.ShooterConstants.kAngleDistanceMultiplier;   // *LIDAR_DISTANCE_VALUE
        //angle += Constants.ShooterConstants.kAngleSpeedMultiplier;      // *ROBOT_SPEED_Y_VALUE
        return angle;
    }

    public boolean angleReasonable() {
        if(angle < Constants.ShooterConstants.kVerticalAngle) {
            return true;
        } else {
            return false;
        }
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
            } else if(ShooterSubsystem.shooterState == ShooterState.AUTO){
                m_rotaterSubsystem.toPosition(evalAngle());
            }
            
        }
        else if(RotaterSubsystem.rotaterState == RotaterState.RESET){
            m_rotaterSubsystem.run(-.25);
            if(m_rotaterSubsystem.getPosition() == 0){
                RotaterSubsystem.rotaterState = RotaterState.INTAKE;
            }
        }

    }

    public boolean isFinished(){
        return false;
    }
    
}
