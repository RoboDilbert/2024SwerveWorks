package frc.robot.commands;

import javax.xml.XMLConstants;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.SwerveSubsystem;

public class AutoMoveCommand extends Command {

    private final SwerveSubsystem swerveSubsystem;
    private double desiredX;
    private double desiredY;
    private double desiredRotation;

    private double xDistance;
    private double yDistance;
    private double angle;

    public AutoMoveCommand(SwerveSubsystem swerveSubsystem, double x, double y, double theta) {
        this.swerveSubsystem = swerveSubsystem;
        desiredX = x;
        desiredY = y;
        desiredRotation = theta;
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        LimelightHelpers.setPipelineIndex("limelight", 0);
    }

    @Override
    public void execute() {
        //Figure out distance and angle to apriltag
        double xSpeed = 0;
        double ySpeed = 0;
        double kPturning = 0.1;
        double kPdistance = 0.3;

        yDistance = ((LimelightHelpers.getTargetPose3d_RobotSpace("limelight").getZ()) / Math.sin(LimelightHelpers.getTargetPose3d_RobotSpace("limelight").getX())) - desiredY;
        xDistance = (LimelightHelpers.getTargetPose3d_RobotSpace("limelight").getZ()) - desiredX;

        //Set turning speed and y speed based off of apriltag
        angle = -kPturning * (LimelightHelpers.getTargetPose3d_RobotSpace("limelight").getX()) - desiredRotation;
        xSpeed = kPdistance * xDistance;
        ySpeed = kPdistance * yDistance;

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
        if(Math.abs(xDistance) < .25 && Math.abs(yDistance) < .25 && Math.abs(angle) < 1){
            return true;
        }
        return false;
    }
}
