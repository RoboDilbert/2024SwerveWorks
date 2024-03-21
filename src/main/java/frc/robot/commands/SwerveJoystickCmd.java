package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveJoystickCmd extends Command {

    private final SwerveSubsystem swerveSubsystem;
    private final Supplier<Double> xSpdFunction, ySpdFunction, turningSpdFunction, brakeFunction;
    private final Supplier<Boolean> fieldOrientedFunction;
    private final SlewRateLimiter turningLimiter;
    private ChassisSpeeds chassisSpeeds;

    private double adjustedHeading;


    public SwerveJoystickCmd(SwerveSubsystem swerveSubsystem,
            Supplier<Double> xSpdFunction, Supplier<Double> ySpdFunction, 
            Supplier<Double> turningSpdFunction, Supplier<Double> brakeFunction, Supplier<Boolean> fieldOrientedFunction) {
        this.swerveSubsystem = swerveSubsystem;
        this.xSpdFunction = xSpdFunction;
        this.ySpdFunction = ySpdFunction;
        this.turningSpdFunction = turningSpdFunction;
        this.brakeFunction = brakeFunction;
        this.fieldOrientedFunction = fieldOrientedFunction;
        this.turningLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        SwerveSubsystem.gyroAngleAuto = swerveSubsystem.getHeading();
    }

    @Override
    public void execute() {
        adjustedHeading = swerveSubsystem.getHeading() - SwerveSubsystem.gyroAngleAuto;

        SmartDashboard.putNumber("X Coord", LimelightHelpers.getTargetPose3d_RobotSpace("limelight").getZ());
        SmartDashboard.putNumber("Y Coord", LimelightHelpers.getTargetPose3d_RobotSpace("limelight").getZ() * Math.tan(LimelightHelpers.getTargetPose3d_RobotSpace("limelight").getX() * 1.13 + Math.toRadians(adjustedHeading)));
        SmartDashboard.putNumber("Rotate Coord", LimelightHelpers.getTargetPose3d_RobotSpace("limelight").getX());
        SmartDashboard.putNumber("Heading Auto", Math.toDegrees(LimelightHelpers.getTargetPose3d_RobotSpace("limelight").getX() * 1.13 + Math.toRadians(adjustedHeading)));

        //Get real-time joystick inputs
        double xSpeed = xSpdFunction.get();
        double ySpeed = ySpdFunction.get();
        double turningSpeed = turningSpdFunction.get();

        //Apply deadband
        xSpeed = Math.abs(xSpeed) > OIConstants.kDeadband ? xSpeed : 0.0;
        ySpeed = Math.abs(ySpeed) > OIConstants.kDeadband ? ySpeed : 0.0;
        turningSpeed = Math.abs(turningSpeed) > OIConstants.kDeadband ? turningSpeed : 0.0;

        //Make the driving smoother & applying smoothing curve (f(x) = x^2)
        turningSpeed = turningLimiter.calculate(turningSpeed*Math.abs(turningSpeed))* DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;
        xSpeed = Math.abs(xSpeed)*xSpeed;
        ySpeed = Math.abs(ySpeed)*ySpeed;
        
        SmartDashboard.putNumber("Limelight Z", LimelightHelpers.getTargetPose3d_RobotSpace("limelight").getZ());

        //Construct desired chassis speeds
        if (fieldOrientedFunction.get()) {
            // Relative to field
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeed, ySpeed, turningSpeed, swerveSubsystem.getRotation2d());

         } else {
            // Relative to robot
            chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
        }

        //Convert chassis speeds to individual module states
        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

        //Output each module states to wheels
        swerveSubsystem.setModuleStates(moduleStates, brakeFunction);
    }

    public ChassisSpeeds getCurrentSpeeds(){
        return chassisSpeeds;
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
