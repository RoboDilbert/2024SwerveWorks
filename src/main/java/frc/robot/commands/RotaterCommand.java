package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.RotaterSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.RotaterSubsystem.RotaterState;
import frc.robot.subsystems.ShooterSubsystem.ShooterState;

public class RotaterCommand extends Command{

    private final RotaterSubsystem m_rotaterSubsystem;
    private double angle;

    public RotaterCommand(RotaterSubsystem rotater){
        m_rotaterSubsystem = rotater;
        angle = 0;
        addRequirements(rotater);
    }

    public void initialize(){

    }

    public double evalAngle() {
        LimelightHelpers.setPipelineIndex("limelight", 0);
        angle = (((Constants.ShooterConstants.kHorizontalAngle + Constants.ShooterConstants.kGearRatio*(LimelightHelpers.getTY("limelight") + 53))) * .71) + .2;                 // *APRIL_TAG_VALUE
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
        SmartDashboard.putNumber("Auto Angle 6", evalAngle());
        if(RotaterSubsystem.rotaterState == RotaterState.INTAKE){
            m_rotaterSubsystem.toPosition(Constants.TeleOpConstants.kRotaterIntakePosition);
        } 
        else if(RotaterSubsystem.rotaterState == RotaterState.SHOOT){
            if(ShooterSubsystem.shooterState == ShooterState.SUB){
                m_rotaterSubsystem.toPosition(Constants.TeleOpConstants.kSubShootPosition);
            }
            else if(ShooterSubsystem.shooterState == ShooterState.TRAP){
                m_rotaterSubsystem.toPosition(Constants.TeleOpConstants.kTrapShootPosition);
            }
            else if(ShooterSubsystem.shooterState == ShooterState.STAGE){
                m_rotaterSubsystem.toPosition(Constants.TeleOpConstants.kStageShootPosition);
            }
            else if(ShooterSubsystem.shooterState == ShooterState.AMP){
                m_rotaterSubsystem.toPosition(Constants.TeleOpConstants.kAmpShootPosition);
            }
        }
        else if(RotaterSubsystem.rotaterState == RotaterState.RESET){
            m_rotaterSubsystem.run(.05);
            if(m_rotaterSubsystem.getPosition() == 0){
                RotaterSubsystem.rotaterState = RotaterState.INTAKE;
            }
        }
        else if(RotaterSubsystem.rotaterState == RotaterState.AUTO){
            if(Math.abs(evalAngle() + 4.37) < .1){
                m_rotaterSubsystem.toPosition(-9.09);
            }
            else{
                m_rotaterSubsystem.toPosition(evalAngle());
            }
        }
        else if(RotaterSubsystem.rotaterState == RotaterState.OFF){

        }
    }

    public boolean isFinished(){
        return false;
    }
    
}
