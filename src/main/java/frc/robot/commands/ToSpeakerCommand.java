package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.SwerveSubsystem;

public class ToSpeakerCommand extends Command {

    private final SwerveSubsystem swerveSubsystem;

    public ToSpeakerCommand(SwerveSubsystem swerveSubsystem) {
        this.swerveSubsystem = swerveSubsystem;
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
        double ySpeed;
        double turningSpeed;
        double kPturning = 0.1;
        double KpDistance = 0.3;
        double distance =  LimelightHelpers.getTargetPose3d_RobotSpace("limelight").getZ();
        double distance_error = distance-2.137;

        //Set turning speed and y speed based off of apriltag
        turningSpeed = -kPturning*LimelightHelpers.getTargetPose3d_RobotSpace("limelight").getX();
        ySpeed = KpDistance*distance_error;

        //Set speeds to chassis
        ChassisSpeeds chassisSpeeds;
        chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
        
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
        return false;
    }
}
