package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public final class Constants {

    public class SparkIDs{
        public static final int shooter1SparkID = 17;
        public static final int shooter2SparkID = 18;

        public static final int intake1SparkID = 9;
        public static final int intake2SparkID = 10;

        public static final int shooterLifter1ID = 15;
        public static final int shooterLifter2ID = 16;

        public static final int lifter1ID = 19;
        public static final int lifter2ID = 20;

        public static final int shooterFeeder1ID = 13;
        public static final int shooterFeeder2ID = 14;

        public static final int rotaterID = 11;
    }

    public class PID{
        //PID Coefficients
        public static final double kP = .000125;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kIz = 0;
        public static final double kFF = 0.0000156;
        public static final double kMaxOutput = 1;
        public static final double kMinOutput = -1;
        public static final double maxRPM = 6800;

        //Smart Motion Coefficeints
        public static final double maxVel = 2000; //rpm
        public static final double minVel = 0; //rpm
        public static final double maxAcc = 1500; //rpm
        public static final double allowedErr = 0;

    }
    public static final class ModuleConstants {
        public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
        public static final double kDriveMotorGearRatio = 1 / 6.75;
        public static final double kTurningMotorGearRatio = 1 / 21.4285714;
        public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
        public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
        public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
        public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;
        public static final double kPTurning = 0.5;
    }

    public static final class DriveConstants {
        public static final double kTrackWidth = Units.inchesToMeters(21.5);
        // Distance between right and left wheels
        public static final double kWheelBase = Units.inchesToMeters(21.5);
        // Distance between front and back wheels
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2));

        public static final int kFrontLeftDriveMotorPort = 4;
        public static final int kBackLeftDriveMotorPort = 8;
        public static final int kFrontRightDriveMotorPort = 2;
        public static final int kBackRightDriveMotorPort = 6;

        public static final int kFrontLeftTurningMotorPort = 3;
        public static final int kBackLeftTurningMotorPort = 7;
        public static final int kFrontRightTurningMotorPort = 1;
        public static final int kBackRightTurningMotorPort = 5;

        public static final boolean kFrontLeftTurningEncoderReversed = false;
        public static final boolean kBackLeftTurningEncoderReversed = false;
        public static final boolean kFrontRightTurningEncoderReversed = false;
        public static final boolean kBackRightTurningEncoderReversed = false;

        public static final boolean kFrontLeftDriveEncoderReversed = false;
        public static final boolean kBackLeftDriveEncoderReversed = false;
        public static final boolean kFrontRightDriveEncoderReversed = true;
        public static final boolean kBackRightDriveEncoderReversed = true;

        public static final int kFrontLeftDriveAbsoluteEncoderPort = 12;
        public static final int kBackLeftDriveAbsoluteEncoderPort = 11;
        public static final int kFrontRightDriveAbsoluteEncoderPort = 9;
        public static final int kBackRightDriveAbsoluteEncoderPort = 10;

        public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = true;
        public static final boolean kBackLeftDriveAbsoluteEncoderReversed = true;
        public static final boolean kFrontRightDriveAbsoluteEncoderReversed = true;
        public static final boolean kBackRightDriveAbsoluteEncoderReversed = true;

        public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = Math.toRadians(311);
        public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = Math.toRadians(20.5); 
        public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = Math.toRadians(122);
        public static final double kBackRightDriveAbsoluteEncoderOffsetRad = Math.toRadians(226.5);

        /*public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = Math.toRadians(301.932 + - 290);
        public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = Math.toRadians(235.94292 - 210 ); 
        public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = Math.toRadians(42.36336 + 245 +(2* Math.PI));
        public static final double kBackRightDriveAbsoluteEncoderOffsetRad = Math.toRadians(35.94744 + 144.25 -180);*/

        public static final double kPhysicalMaxSpeedMetersPerSecond = 5;
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 1 * Math.PI;

        public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond / 4;
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = //
                kPhysicalMaxAngularSpeedRadiansPerSecond / 4;
        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;
    }

    public static final class SensorConstants {
        public static final int lidarID1 = 30;
        public static final int lidarID2 = 31;

        public static final int intake1ID = 34;
        public static final int intake2ID = 33;

        public static final int shooterSensor1ID = 32;
        public static final int shooterSensor2ID = 35;
    }

    public static final class LEDConstants {
        public static final int LED_PWM = 4;
    }


    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = DriveConstants.kPhysicalMaxSpeedMetersPerSecond / 4;
        public static final double kMaxAngularSpeedRadiansPerSecond =
                DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond;
        public static final double kMaxAccelerationMetersPerSecondSquared = 0.3;
        public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 4;
        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = .25;

        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = //
                new TrapezoidProfile.Constraints(
                        kMaxAngularSpeedRadiansPerSecond,
                        kMaxAngularAccelerationRadiansPerSecondSquared);
    }

    public static final class OIConstants {
        public static final int kDriverControllerPort = 1;

        public static final int kDriverYAxis = 1;
        public static final int kDriverXAxis = 0;
        public static final int kDriverRotAxis = 4;
        public static final int kDriverFieldOrientedButtonIdx = 1;

        public static final double kDeadband = 0.05;
    }

    public static final class TeleOpConstants {
        public static final double kRotaterIntakePosition = -13.8; //12.8
        public static final double kSubShootPosition = -1;
        public static final double kLineShootPosition = -38;
        public static final double kStageShootPosition = -11.5;
        public static final double kAmpShootPosition = -31;
    }

    public static final class ShooterConstants {
        public static final double kPowerDistanceMultiplier = 20;   // Power and Distance Directely Related
        public static final double kPowerSpeedMultiplier = -20;     // Power and Speed Inversely Related
        public static final double kAngleDistanceMultiplier = 20;   // Angle and Distance Directly Related                                                                                  
        public static final double kAngleSpeedMultiplier = -20;     // Angle and Speed Inversely Relatedd
        
        public static final double kGearRatio = 4.0/9.0;                 // Gear ratio 80/360 = 2/9

        public static final double kIdleSpeed = 20;                 // Idle Speed
        public static final double kHorizontalAngle = -30;           // Horizontal Angle 

        public static final double kVerticalAngle = -1;            // Vertical Angle     
        public static final double maxPower = 5800;                 // Max Power       
    }
}
