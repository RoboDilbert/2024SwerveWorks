package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.SensorsSubsystem;

public class ToAmpCommand extends Command {

    private final SwerveSubsystem swerveSubsystem;
    private final SensorsSubsystem sensorsSubsystem;


    public ToAmpCommand(SwerveSubsystem swerveSubsystem, SensorsSubsystem sensorsSubsystem) {
        this.swerveSubsystem = swerveSubsystem;
        addRequirements(swerveSubsystem);

        this.sensorsSubsystem = sensorsSubsystem;
    }

    @Override
    public void initialize() {
        LimelightHelpers.setPipelineIndex("limelight", 1);
    }

    @Override
    public void execute() {
        //Sensor Speeds
        double xSpeed = 0;
        double ySpeed = 0;
        double turningSpeed = 0;
        //Kp values
        double KpTurning = 0.6;
        double KpDistance = 0.4;
        double KpStrafe = 0.6;
        // distance error
        double distance;
        double distance_error;


        
        if (LimelightHelpers.getTV("limelight")) {
            xSpeed = -KpStrafe*LimelightHelpers.getTargetPose3d_RobotSpace("limelight").getX();
        } else {
            xSpeed = 0;
        }

        sensorsSubsystem.setRangeMode(getName());
        //Distance Measurements
        distance = sensorsSubsystem.getDistance()/1000;
        distance_error =  distance - 0.3;
        
        //Setting y Speeds
        xSpeed = KpDistance * distance_error;

        //Limiting y Speed
        ChassisSpeeds chassisSpeeds;
           if(Math.abs(ySpeed) > 0.2){
               ySpeed = 0.2*Math.signum(ySpeed);
           }

        //Setting t Speeds
        turningSpeed = KpTurning*(sensorsSubsystem.getRightDistance()-sensorsSubsystem.getLeftDistance())/1000; 
                
        chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
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
