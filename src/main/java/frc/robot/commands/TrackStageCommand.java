package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.SwerveSubsystem;

public class TrackStageCommand extends Command {

    private final SwerveSubsystem swerveSubsystem;
    private final Supplier<Double> xSpdFunction;

    public TrackStageCommand(SwerveSubsystem swerveSubsystem,
    Supplier<Double> xSpdFunction, Supplier<Double> ySpdFunction) {
        this.swerveSubsystem = swerveSubsystem;
        this.xSpdFunction = xSpdFunction;
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        LimelightHelpers.setPipelineIndex("limelight", 2);
    }

    @Override
    public void execute() {
        //Get real-time joystick inputs and angle from apriltag
        double xSpeed = xSpdFunction.get();
        double ySpeed = 0;
        double turningSpeed = 0;
        //double angle = swerveSubsystem.getHeading() % 360;

        //Set kps
        double KpDistance = 0.3;
        double kPturning = 0.1;

        //set distance error
        double distance =  LimelightHelpers.getTargetPose3d_RobotSpace("limelight").getZ();
        double distance_error = distance-2.137;
        
        //Apply deadband
        xSpeed = Math.abs(xSpeed) > OIConstants.kDeadband ? xSpeed : 0.0;

        //Make driving smoother

        xSpeed = Math.abs(xSpeed)*xSpeed;
        ySpeed = KpDistance*distance_error;

        if (ySpeed != 0) {
            turningSpeed = -kPturning*LimelightHelpers.getTargetPose3d_RobotSpace("limelight").getX();
        }

        //Construct desired chassis speeds
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
