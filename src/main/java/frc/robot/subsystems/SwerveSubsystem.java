package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.ShooterCommand;

public class SwerveSubsystem extends SubsystemBase {

    double speed = 0;

    private final SwerveModule frontLeft = new SwerveModule(
            DriveConstants.kFrontLeftDriveMotorPort,
            DriveConstants.kFrontLeftTurningMotorPort,
            DriveConstants.kFrontLeftDriveEncoderReversed,
            DriveConstants.kFrontLeftTurningEncoderReversed,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderPort,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed);

    private final SwerveModule frontRight = new SwerveModule(
            DriveConstants.kFrontRightDriveMotorPort,
            DriveConstants.kFrontRightTurningMotorPort,
            DriveConstants.kFrontRightDriveEncoderReversed,
            DriveConstants.kFrontRightTurningEncoderReversed,
            DriveConstants.kFrontRightDriveAbsoluteEncoderPort,
            DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kFrontRightDriveAbsoluteEncoderReversed);

    private final SwerveModule backLeft = new SwerveModule(
            DriveConstants.kBackLeftDriveMotorPort,
            DriveConstants.kBackLeftTurningMotorPort,
            DriveConstants.kBackLeftDriveEncoderReversed,
            DriveConstants.kBackLeftTurningEncoderReversed,
            DriveConstants.kBackLeftDriveAbsoluteEncoderPort,
            DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kBackLeftDriveAbsoluteEncoderReversed);

    private final SwerveModule backRight = new SwerveModule(
            DriveConstants.kBackRightDriveMotorPort,
            DriveConstants.kBackRightTurningMotorPort,
            DriveConstants.kBackRightDriveEncoderReversed,
            DriveConstants.kBackRightTurningEncoderReversed,
            DriveConstants.kBackRightDriveAbsoluteEncoderPort,
            DriveConstants.kBackRightDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kBackRightDriveAbsoluteEncoderReversed);

    private final AHRS gyro = new AHRS(SPI.Port.kMXP);


    //private final Pigeon2 gyro_robot = new Pigeon2(13);


    private SwerveModulePosition[] modulePosition = new SwerveModulePosition[4];

    private final SwerveDriveOdometry odometer;

    public SwerveSubsystem() {
        modulePosition[0] = new SwerveModulePosition(frontLeft.getDrivePosition(), frontLeft.getState().angle);
        modulePosition[1] = new SwerveModulePosition(frontRight.getDrivePosition(), frontRight.getState().angle);
        modulePosition[2] = new SwerveModulePosition(backLeft.getDrivePosition(), backLeft.getState().angle);
        modulePosition[3] = new SwerveModulePosition(backRight.getDrivePosition(), backRight.getState().angle);
        
        odometer = new SwerveDriveOdometry(Constants.DriveConstants.kDriveKinematics,
        new Rotation2d(0), modulePosition);

        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading();
            } catch (Exception e) {
            }
        }).start();
    }

    public void zeroHeading() {
        gyro.reset();
    }

    public double getHeading() {
        return Math.IEEEremainder(gyro.getAngle(), 360);
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    public Pose2d getPose() {
        return odometer.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        odometer.resetPosition(getRotation2d(), modulePosition, pose);
    }

    @Override
    public void periodic() {
        odometer.update(getRotation2d(), modulePosition);
        SmartDashboard.putNumber("Robot Heading", getHeading());
        SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());

    }

    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {

        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);

        

        

        
        

        //Error Messages
        //double x = frontLeft.getAbsoluteEncoderDeg();
        //String message1 = "frontLeft - " + x;
        //DriverStation.reportWarning(message1, false);

        //double y = frontRight.getAbsoluteEncoderDeg();
        //String message2 = "frontRight - " + y;
        //DriverStation.reportWarning(message2, false);

        //double z = backLeft.getAbsoluteEncoderDeg();
        //String message3 = "backLeft - " + z;
        //DriverStation.reportWarning(message3, false);

        //double w = backRight.getAbsoluteEncoderDeg();
        //String message4 = "backRight - " + w;
        //DriverStation.reportWarning(message4, false);



        //Applying Brake Multiplier
        desiredStates[0].speedMetersPerSecond *= (1.05-RobotContainer.ps4_controller.getRawAxis(3));
        desiredStates[1].speedMetersPerSecond *= (1.05-RobotContainer.ps4_controller.getRawAxis(3));
        desiredStates[2].speedMetersPerSecond *= (1.05-RobotContainer.ps4_controller.getRawAxis(3));
        desiredStates[3].speedMetersPerSecond *= (1.05-RobotContainer.ps4_controller.getRawAxis(3));

        
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);


        speed += 0.01*RobotContainer.manipulator.getLeftY();
            if(speed >= 1){
               speed = 1;
            } else if(speed <= -1){
                speed = -1;
        }



        if(RobotContainer.manipulator.getRawButton(1)){
            ShooterCommand.run(speed);
        } else {
            ShooterCommand.stop();
        }

        


        //public double tx = LimelightHelpers.getTX("");
        //public double ty = LimelightHelpers.getTY("");
        //public double distance = LimelightHelpers.getTA("");



        //String error = " " + LimelightHelpers.getTX("");


        //DriverStation.reportError(error, false);

        
        
    }
}
