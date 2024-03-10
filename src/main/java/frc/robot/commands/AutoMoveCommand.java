package frc.robot.commands;

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
    private boolean straight;

    private double xDistance;
    private double yDistance;
    private double angle;

    private double adjustedHeading;

    public AutoMoveCommand(SwerveSubsystem swerveSubsystem, RotaterSubsystem rotaterSubsystem, double x, double y, boolean straight) {
        this.swerveSubsystem = swerveSubsystem;
        this.rotaterSubsystem = rotaterSubsystem;
        desiredX = x;
        desiredY = y;
        this.straight = straight;
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
        adjustedHeading = swerveSubsystem.getHeading() - SwerveSubsystem.gyroAngleAuto;
        angle = 0;
        double kPturning = 0.25;
        double kPX = 0.5;
        double kPY = 0.5;

        if(straight){
            yDistance = 0;
            angle = 0;
        }
        else{
            yDistance = ((LimelightHelpers.getTargetPose3d_RobotSpace("limelight").getZ()) * Math.tan(Math.toRadians((LimelightHelpers.getTargetPose3d_RobotSpace("limelight").getX()) + adjustedHeading))) + desiredY;
            
            //yDistance = 0;
            angle = -kPturning * (LimelightHelpers.getTargetPose3d_RobotSpace("limelight").getX());
        }

        xDistance = (LimelightHelpers.getTargetPose3d_RobotSpace("limelight").getZ()) - desiredX;

        xSpeed = kPX * xDistance;
        ySpeed = kPY * yDistance;

        SmartDashboard.putNumber("pre y", ySpeed);
        SmartDashboard.putNumber("pre x", xSpeed);
        SmartDashboard.putNumber("rotate", angle);
        SmartDashboard.putNumber("heading", adjustedHeading);
        SmartDashboard.putNumber("offset", SwerveSubsystem.gyroAngleAuto);
        SmartDashboard.putNumber("corrected angle", ((LimelightHelpers.getTargetPose3d_RobotSpace("limelight").getX()) + adjustedHeading));

        if(Math.abs(xSpeed) > .2){
            xSpeed = .2 * Math.signum(xSpeed);
        }
        if(Math.abs(ySpeed) > .15){
            ySpeed = .15 * Math.signum(ySpeed);
        }
        if(Math.abs(angle) > .3){
            angle = .3 * Math.signum(ySpeed);
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
        if(Math.abs(xDistance) < .05 && Math.abs(yDistance) < .05 && Math.abs(angle) < .05){
            return true;
        }
        return false;
    }
}
