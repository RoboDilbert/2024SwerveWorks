package frc.robot;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SynchronousInterrupt.WaitResult;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.AutoMoveCommand;
import frc.robot.commands.FeederCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.LifterCommand;
import frc.robot.commands.RotaterCommand;
import frc.robot.commands.ShooterCommand;
import frc.robot.commands.ShooterLifterCommand;
import frc.robot.commands.ShooterMaxCommand;
import frc.robot.commands.ToAmpCommand;
import frc.robot.commands.ToSpeakerCommand;
import frc.robot.commands.TrackSpeakerCommand;
import frc.robot.commands.TrackRingCommand;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.FeederSubsystem.FeederState;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LifterSubsystem;
import frc.robot.subsystems.RotaterSubsystem;
import frc.robot.subsystems.SensorsSubsystem;
import frc.robot.subsystems.ShooterLifterSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class RobotContainer {

    private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
    private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
    private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    private final SensorsSubsystem sensorsSubsystem = new SensorsSubsystem();
    private final LifterSubsystem lifterSubsystem = new LifterSubsystem();
    private final RotaterSubsystem rotaterSubsystem = new RotaterSubsystem();
    private final FeederSubsystem feederSubsystem = new FeederSubsystem();
    private final ShooterLifterSubsystem shooterLifterSubsystem = new ShooterLifterSubsystem();
    

    CommandXboxController driver_controller = new CommandXboxController(0);
    CommandXboxController manipulator = new CommandXboxController(2); 

        private final SendableChooser<Command> autoChooser;


        public RobotContainer() {
                swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(swerveSubsystem,
                        () -> -driver_controller.getLeftX(),
                        () -> -driver_controller.getLeftY(),
                        () -> -driver_controller.getRawAxis(4),
                        () -> driver_controller.getRawAxis(3),
                        () -> true
                ));
                
                /*shooterSubsystem.setDefaultCommand(new ShooterCommand(shooterSubsystem, 
                        () -> 0
                ));*/
                
                feederSubsystem.setDefaultCommand(new FeederCommand(feederSubsystem));

                //rotaterSubsystem.setDefaultCommand(new RotaterCommand(rotaterSubsystem));

                intakeSubsystem.setDefaultCommand(new IntakeCommand(intakeSubsystem));
                
                lifterSubsystem.setDefaultCommand(new LifterCommand(lifterSubsystem, () -> manipulator.getLeftY(), () -> manipulator.getRightY()));
                
                shooterLifterSubsystem.setDefaultCommand(new ShooterLifterCommand(shooterLifterSubsystem, () -> manipulator.getRawAxis(3), ()-> manipulator.getRawAxis(2)));

                sensorsSubsystem.setRangeMode("Long");

                autoChooser = AutoBuilder.buildAutoChooser();

                autoChooser.setDefaultOption("mid", mid);
                autoChooser.addOption("left",  left);
                autoChooser.addOption("right", right);

                SmartDashboard.putData("Auto Chooser", autoChooser);

                configureButtonBindings(); 
        }
        

    private void configureButtonBindings() {
        driver_controller.y().onTrue((new InstantCommand(swerveSubsystem::zeroHeading)));
        
        driver_controller.a().whileTrue(new ToSpeakerCommand(swerveSubsystem));
        driver_controller.b().whileTrue(new TrackSpeakerCommand(swerveSubsystem, 
                () -> -driver_controller.getLeftX(),
                () -> -driver_controller.getLeftY() 
        ));
        driver_controller.x().whileTrue(new TrackRingCommand(swerveSubsystem,
                () -> -driver_controller.getLeftX(),
                () -> -driver_controller.getLeftY()
        ));
        //[TEMP] going to the amp is the left bumper
        driver_controller.leftBumper().whileTrue(new ToAmpCommand(swerveSubsystem, sensorsSubsystem));
        driver_controller.rightBumper().whileTrue(new TrackSpeakerCommand(swerveSubsystem, 
                () -> -driver_controller.getLeftX(), null
        ));

        driver_controller.x().onTrue(rotaterSubsystem.reset());

        manipulator.a().onTrue(intakeSubsystem.toggleIntake());
        manipulator.rightBumper().onTrue(intakeSubsystem.reverse());
        manipulator.b().whileTrue(new ShooterMaxCommand(shooterSubsystem));
        manipulator.y().onTrue(new InstantCommand(rotaterSubsystem::resetPosition));
        manipulator.x().onTrue(feederSubsystem.back());
        
        

        manipulator.pov(270).onTrue(new InstantCommand(shooterSubsystem::setSub));
        manipulator.pov(0).onTrue(new InstantCommand(shooterSubsystem::setLine));
        manipulator.pov(90).onTrue(new InstantCommand(shooterSubsystem::setStage));

        
        manipulator.button(7).onTrue(new InstantCommand(rotaterSubsystem::setAuto));
        
        if(FeederSubsystem.feederState != FeederState.FEED){
                manipulator.pov(180).onTrue(feederSubsystem.shootUpOBlock());
        }

        manipulator.leftBumper().onTrue(feederSubsystem.shootUpAmp());

        DriverStation.reportError("Intake State: " + IntakeSubsystem.intakeState, true);

    }

    /*public Command getAutonomousCommand() {
        //PathPlannerPath path = PathPlannerPath.fromPathFile("Multiple Points");
        //PathPlannerPath path = PathPlannerPath.fromPathFile("backnforth");

        swerveSubsystem.zeroHeading();
        //PathPlannerPath path = PathPlannerPath.fromPathFile("straight as gavin");

        List<Translation2d> points = PathPlannerPath.bezierFromPoses(
                new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0)),
                new Pose2d(0.5, 0, Rotation2d.fromDegrees(0))
        );      

        PathPlannerPath path = new PathPlannerPath(
                points,
                new PathConstraints(.25, .25, 2 * Math.PI, 4 * Math.PI),
                new GoalEndState(0.0, Rotation2d.fromDegrees(0))
        );
        
        return AutoBuilder.followPath(path);
        return autoChooser.getSelected();
    }*/

        public Command getAutonomousCommand(){
                return autoChooser.getSelected();
        }

        /*public Command getAutonomousCommand() {
        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                        .setKinematics(DriveConstants.kDriveKinematics);

        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
                new Pose2d(-0.01, 0, new Rotation2d(0)),
                List.of(
                        new Translation2d(-0.25, 0),
                        new Translation2d(-0.5, 0),
                        new Translation2d(-0.75, 0)
                ),
                new Pose2d(-1,0, Rotation2d.fromDegrees(0)),
                trajectoryConfig);

        PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
        PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
        ProfiledPIDController thetaController = new ProfiledPIDController(
                AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        
        return new SequentialCommandGroup(
                new InstantCommand(() -> swerveSubsystem.resetOdometry(trajectory.getInitialPose())),
                new InstantCommand(intakeSubsystem::toggleIntake),
                getDriveCommand(trajectory, xController, yController, thetaController),
                new InstantCommand(() -> swerveSubsystem.stopModules()));
        
        }

        public SwerveControllerCommand getDriveCommand(Trajectory traj, PIDController xContr, PIDController yContr, ProfiledPIDController thetaCont){
                return new SwerveControllerCommand(
                traj,
                swerveSubsystem::getPose,
                DriveConstants.kDriveKinematics,
                xContr,
                yContr,
                thetaCont,
                swerveSubsystem::setModuleStates,
                swerveSubsystem);
        }*/

        
        SequentialCommandGroup mid = new SequentialCommandGroup(
                new AutoMoveCommand(swerveSubsystem, 1, 1, 1)
        );
        SequentialCommandGroup left = new SequentialCommandGroup(
                        
        );
        SequentialCommandGroup right = new SequentialCommandGroup(
                        
        );
        

}
