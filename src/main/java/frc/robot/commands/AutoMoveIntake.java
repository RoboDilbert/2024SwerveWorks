package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.FeederSubsystem.FeederState;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.RotaterSubsystem;
import frc.robot.subsystems.RotaterSubsystem.RotaterState;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.IntakeSubsystem.IntakeState;

public class AutoMoveIntake extends Command {

    private final SwerveSubsystem swerveSubsystem;
    private final RotaterSubsystem rotaterSubsystem;
    private final IntakeSubsystem intakeSubsystem;
    private final FeederSubsystem feederSubsystem;

    private double desiredX;
    private double desiredY;
    private boolean straight;

    private double xDistance;
    private double yDistance;
    private double angle;

    private double adjustedHeading;

    public AutoMoveIntake(SwerveSubsystem swerveSubsystem, RotaterSubsystem rotaterSubsystem, IntakeSubsystem intakeSubsystem, FeederSubsystem feederSubsystem, double x, double y, boolean straight) {
        this.swerveSubsystem = swerveSubsystem;
        this.rotaterSubsystem = rotaterSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.feederSubsystem = feederSubsystem;
        desiredX = x;
        desiredY = y;
        this.straight = straight;
        addRequirements(swerveSubsystem, rotaterSubsystem, intakeSubsystem, feederSubsystem);
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
        if(IntakeSubsystem.intakeState == IntakeState.INTAKE){
            IntakeSubsystem.intakeState = IntakeState.INTAKE;
            FeederSubsystem.feederState = FeederState.FEED;
            intakeSubsystem.run(.75);
            if(intakeSubsystem.getDistance() < 42){
                intakeSubsystem.run(0);
                feederSubsystem.feedStop();
                FeederSubsystem.feederState = FeederState.OFF;
                IntakeSubsystem.intakeState = IntakeState.OFF;
            }
        }
        
        //shooterLifterSubsystem.toPosition(4.0);

        //Figure out distance and angle to apriltag
        double xSpeed = 0;
        double ySpeed = 0;
        adjustedHeading = swerveSubsystem.getHeading() - SwerveSubsystem.gyroAngleAuto;
        angle = 0;
        double kPturning = 0.45;
        double kPX = 0.1875;
        double kPY = 0.125;

        if(straight){
            yDistance = 0;
        }
        else{
            yDistance = ((LimelightHelpers.getTargetPose3d_RobotSpace("limelight").getZ()) * Math.tan(Math.toRadians((LimelightHelpers.getTargetPose3d_RobotSpace("limelight").getX()) + adjustedHeading))) + desiredY;
            
            //yDistance = 0;
        }

        xDistance = (LimelightHelpers.getTargetPose3d_RobotSpace("limelight").getZ()) - desiredX;
        angle = -kPturning * (LimelightHelpers.getTargetPose3d_RobotSpace("limelight").getX());

        xSpeed = kPX * xDistance;
        ySpeed = kPY * yDistance;

        if(LimelightHelpers.getTargetPose3d_RobotSpace("limelight").getZ() == 0){
            xSpeed = 0;
            ySpeed = 0;
            angle = 0;
        }

        SmartDashboard.putNumber("pre y", ySpeed);
        SmartDashboard.putNumber("pre x", xSpeed);
        SmartDashboard.putNumber("rotate", angle);
        SmartDashboard.putNumber("heading", adjustedHeading);
        SmartDashboard.putNumber("offset", SwerveSubsystem.gyroAngleAuto);
        SmartDashboard.putNumber("corrected angle", ((LimelightHelpers.getTargetPose3d_RobotSpace("limelight").getX()) + adjustedHeading));

        if(Math.abs(xSpeed) > .45){
            xSpeed = .45 * Math.signum(xSpeed);
        }
        if(Math.abs(ySpeed) > .3){
            ySpeed = .3 * Math.signum(ySpeed);
        }
        if(Math.abs(angle) > .3){
            angle = .3 * Math.signum(ySpeed);
        }

        SmartDashboard.putNumber("y", ySpeed);

        //Set speeds to chassis
        ChassisSpeeds chassisSpeeds;
        chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, angle / 1.6);
        
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
        if(Math.abs(xDistance) < .3 && Math.abs(yDistance) < .3 && Math.abs(angle) < .3){
            return true;
        }
        return false;
    }
}
