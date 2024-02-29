package frc.robot.subsystems;

import java.util.function.Supplier;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;


public class SwerveSubsystem extends SubsystemBase {

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

    private final SwerveDriveOdometry odometer;
    private final AHRS gyro = new AHRS(SPI.Port.kMXP);
    private SwerveModulePosition[] modulePosition = new SwerveModulePosition[4];
    private SwerveModuleState[] moduleStates = new SwerveModuleState[4]; 

    public SwerveSubsystem() {
        modulePosition[0] = new SwerveModulePosition(frontLeft.getDrivePosition(), frontLeft.getState().angle);
        modulePosition[1] = new SwerveModulePosition(frontRight.getDrivePosition(), frontRight.getState().angle);
        modulePosition[2] = new SwerveModulePosition(backLeft.getDrivePosition(), backLeft.getState().angle);
        modulePosition[3] = new SwerveModulePosition(backRight.getDrivePosition(), backRight.getState().angle);
        
        moduleStates[0] = new SwerveModuleState(frontLeft.getDriveVelocity(), frontLeft.getState().angle);
        moduleStates[1] = new SwerveModuleState(frontRight.getDriveVelocity(), frontLeft.getState().angle);
        moduleStates[2] = new SwerveModuleState(backLeft.getDriveVelocity(), frontLeft.getState().angle);
        moduleStates[3] = new SwerveModuleState(backRight.getDriveVelocity(), frontLeft.getState().angle);

        odometer = new SwerveDriveOdometry(Constants.DriveConstants.kDriveKinematics,
        new Rotation2d(0), modulePosition);

        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading();
            } catch (Exception e) {
            }
        }).start();

        configurePathPlanner();
    }

    public void zeroHeading() {
        gyro.reset();
    }

    public double getHeading() {
        return Math.IEEEremainder((gyro.getAngle()-90), 360);
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

    public void updateStates(SwerveModuleState[] moduleStatessss){
        moduleStatessss[0].speedMetersPerSecond = frontLeft.getDriveVelocity();
        moduleStatessss[1].speedMetersPerSecond = frontRight.getDriveVelocity();
        moduleStatessss[2].speedMetersPerSecond = backLeft.getDriveVelocity();
        moduleStatessss[3].speedMetersPerSecond = backRight.getDriveVelocity();

        moduleStatessss[0].angle = frontLeft.getState().angle;
        moduleStatessss[1].angle = frontRight.getState().angle;
        moduleStatessss[2].angle = backLeft.getState().angle;
        moduleStatessss[3].angle = backRight.getState().angle;
    }

    @Override
    public void periodic() {
        odometer.update(getRotation2d(), modulePosition);
        updateStates(moduleStates);
        SmartDashboard.putNumber("Robot Heading", getHeading());
        SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
    }

    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public void drive(
        double throttle,
        double strafe,
        double rotation,
        boolean isFieldRelative,
        boolean isOpenLoop) {

        // Applies a Deadband of 0.05 to the controllers input
        throttle = throttle * Constants.AutoConstants.kMaxSpeedMetersPerSecond;
        strafe = strafe * Constants.AutoConstants.kMaxSpeedMetersPerSecond;
        rotation = rotation * Constants.AutoConstants.kMaxAngularSpeedRadiansPerSecond;

        ChassisSpeeds chassisSpeeds = isFieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(
                throttle, strafe, rotation, 
                // Sets an offset if robot path doesn't start facing drive station
                Rotation2d.fromDegrees(getHeading()))
            : new ChassisSpeeds(throttle, strafe, rotation);
        moduleStates = Constants.DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
        setModuleStates(moduleStates);
    }

    public void drive(ChassisSpeeds chassisSpeeds){   
        moduleStates = Constants.DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
        setModuleStates(moduleStates);
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {

        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);

        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    } 

    public void setModuleStates(SwerveModuleState[] desiredStates, Supplier<Double> brakeFunction) {

        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);

        //String error1 = " " + sensorsSubsystem.getDistance();
        //String error2 = " " + sensorsSubsystem.getLeftDistance();
        //String error3 = " " + sensorsSubsystem.getRightDistance();
        //DriverStation.reportWarning(error1, false);
        //DriverStation.reportWarning(error2, false);
        //DriverStation.reportWarning(error3, false);
        //Applying Brake Multiplier

        desiredStates[0].speedMetersPerSecond *= (1.2 - brakeFunction.get());
        desiredStates[1].speedMetersPerSecond *= (1.2 - brakeFunction.get());
        desiredStates[2].speedMetersPerSecond *= (1.2 - brakeFunction.get());
        desiredStates[3].speedMetersPerSecond *= (1.2 - brakeFunction.get());

        desiredStates[0].speedMetersPerSecond *= 0.8;
        desiredStates[1].speedMetersPerSecond *= 0.8;
        desiredStates[2].speedMetersPerSecond *= 0.8;
        desiredStates[3].speedMetersPerSecond *= 0.8;

        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    } 

    public ChassisSpeeds getChassisSpeed() {
        return Constants.DriveConstants.kDriveKinematics.toChassisSpeeds(
            frontLeft.getState(),
            frontRight.getState(),
            backLeft.getState(),
            backRight.getState()
         );
     }

    public void configurePathPlanner() {
        AutoBuilder.configureHolonomic(
            this::getPose, // Robot pose supplier
            this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getChassisSpeed, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            this::drive, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                    new PIDConstants(0.001, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(0.001, 0.0, 0.0), // Rotation PID constants
                    0.5, // Max module speed, in m/s
                    0.4, // Drive base radius in meters. Distance from robot center to furthest module.
                    new ReplanningConfig() // Default path replanning config. See the API for the options here
            ),
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
        );
    }
}