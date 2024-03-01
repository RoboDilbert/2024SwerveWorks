package frc.robot.commands;

import javax.xml.XMLConstants;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.RotaterSubsystem;
import frc.robot.subsystems.RotaterSubsystem.RotaterState;
import frc.robot.subsystems.SwerveSubsystem;

public class AutoMoveCommand extends Command {

    private final SwerveSubsystem swerveSubsystem;
    private final RotaterSubsystem rotaterSubsystem;
    private double desiredX;
    private double desiredY;

    private double xDistance;
    private double yDistance;
    private double angle;

    private double gyroOffset;

    public AutoMoveCommand(SwerveSubsystem swerveSubsystem, RotaterSubsystem rotaterSubsystem, double x, double y) {
        this.swerveSubsystem = swerveSubsystem;
        this.rotaterSubsystem = rotaterSubsystem;
        desiredX = x;
        desiredY = y;
        gyroOffset = swerveSubsystem.getHeading();
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        LimelightHelpers.setPipelineIndex("limelight", 0);
    }

    @Override
    public void execute() {
        if(RotaterSubsystem.rotaterState == RotaterState.INTAKE){
            rotaterSubsystem.toPosition(Constants.TeleOpConstants.kRotaterIntakePosition);
        } 
        //Figure out distance and angle to apriltag
        double xSpeed = 0;
        double ySpeed = 0;
        double heading = swerveSubsystem.getHeading() - gyroOffset;
        angle = 0;
        double kPturning = 0.5;
        double kPdistance = 0.5;

        //yDistance = ((LimelightHelpers.getTargetPose3d_RobotSpace("limelight").getZ()) * Math.tan(LimelightHelpers.getTargetPose3d_RobotSpace("limelight").getX())) + desiredY;
        xDistance = (LimelightHelpers.getTargetPose3d_RobotSpace("limelight").getZ()) - desiredX;

        //Set turning speed and y speed based off of apriltag
        //angle = -kPturning * (LimelightHelpers.getTargetPose3d_RobotSpace("limelight").getX());
        xSpeed = kPdistance * xDistance;
        ySpeed = kPdistance * yDistance;

        SmartDashboard.putNumber("pre y", ySpeed);
        SmartDashboard.putNumber("rotate", angle);
        SmartDashboard.putNumber("auto heading", heading);

        if(Math.abs(xSpeed) > .3){
            xSpeed = .3 * Math.signum(xSpeed);
        }
        if(Math.abs(ySpeed) > .1){
            ySpeed = .1 * Math.signum(ySpeed);
        }
        if(Math.abs(angle) > .3){
            ySpeed = .3 * Math.signum(ySpeed);
        }

        SmartDashboard.putNumber("y", ySpeed);
        //Set speeds to chassis
        ChassisSpeeds chassisSpeeds;
        chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, angle);
        
        //Convert chassis speeds to individual module states
        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

        //Output each module states to wheels
        swerveSubsystem.setModuleStates(moduleStates);
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() {
        if(Math.abs(xDistance) < .01 && Math.abs(yDistance) < .01 && Math.abs(angle) < .01){
            return true;
        }
        return false;
    }
}
