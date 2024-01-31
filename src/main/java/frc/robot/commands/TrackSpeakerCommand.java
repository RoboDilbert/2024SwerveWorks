package frc.robot.commands;

import java.util.function.Supplier;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.SwerveSubsystem;

public class TrackSpeakerCommand extends Command {

    private final SwerveSubsystem swerveSubsystem;
    private final Supplier<Double> xSpdFunction, ySpdFunction;


    public TrackSpeakerCommand(SwerveSubsystem swerveSubsystem,
            Supplier<Double> xSpdFunction, Supplier<Double> ySpdFunction) {
        this.swerveSubsystem = swerveSubsystem;
        this.xSpdFunction = xSpdFunction;
        this.ySpdFunction = ySpdFunction;
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        LimelightHelpers.setPipelineIndex("limelight", 0);
    }

    @Override
    public void execute() {
        //Get real-time joystick inputs and angle from apriltag
        
        double xSpeed = xSpdFunction.get();
        double ySpeed = ySpdFunction.get();
        double turningSpeed;
        double kPturning = 0.1;
        
        //Apply deadband
        xSpeed = Math.abs(xSpeed) > OIConstants.kDeadband ? xSpeed : 0.0;
        ySpeed = Math.abs(ySpeed) > OIConstants.kDeadband ? ySpeed : 0.0;


        turningSpeed = -kPturning*LimelightHelpers.getTargetPose3d_RobotSpace("limelight").getX() * 4.5;
        xSpeed = Math.abs(xSpeed)*xSpeed;
        ySpeed = Math.abs(ySpeed)*ySpeed;
        

        //onstruct desired chassis speeds
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
