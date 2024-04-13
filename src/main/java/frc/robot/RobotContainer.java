package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.AmpCommand;
import frc.robot.commands.AutoMoveCommand;
import frc.robot.commands.AutoMoveIntake;
import frc.robot.commands.AutoShoot;
import frc.robot.commands.AutoShootClose;
import frc.robot.commands.AutoShootFeed;
import frc.robot.commands.FeedBackCommand;
import frc.robot.commands.FeederCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.IntakeShooterAuto;
import frc.robot.commands.LEDCommand;
import frc.robot.commands.LifterCommand;
import frc.robot.commands.RotaterCommand;
import frc.robot.commands.ShooterCommand;
import frc.robot.commands.ShooterLifterCommand;
import frc.robot.commands.ShooterMaxCommand;
import frc.robot.commands.ShooterMaxCommandTeleop;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.commands.ToSpeakerCommand;
import frc.robot.commands.TrackSpeakerCommand;
import frc.robot.commands.TrapCommand;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.FeederSubsystem.FeederState;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.LifterSubsystem;
import frc.robot.subsystems.RotaterSubsystem;
import frc.robot.subsystems.SensorsSubsystem;
import frc.robot.subsystems.ShooterLifterSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;


public class RobotContainer {

    private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
    private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
    private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    private final SensorsSubsystem sensorsSubsystem = new SensorsSubsystem();
    private final LifterSubsystem lifterSubsystem = new LifterSubsystem();
    private final RotaterSubsystem rotaterSubsystem = new RotaterSubsystem();
    private final FeederSubsystem feederSubsystem = new FeederSubsystem();
    private final LEDSubsystem ledSubsystem = new LEDSubsystem();
    private final ShooterLifterSubsystem shooterLifterSubsystem = new ShooterLifterSubsystem();
    

    CommandXboxController driver_controller = new CommandXboxController(0);
    CommandXboxController manipulator = new CommandXboxController(2); 

        private final SendableChooser<Command> autoChooser2;


        public RobotContainer() {
                swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(swerveSubsystem,
                        () -> -driver_controller.getLeftX(),
                        () -> -driver_controller.getLeftY(),
                        () -> -driver_controller.getRawAxis(4),
                        () -> driver_controller.getRawAxis(3),
                        () -> true
                ));
                
                shooterSubsystem.setDefaultCommand(new ShooterCommand(shooterSubsystem, 
                        () -> 0
                ));
                
                feederSubsystem.setDefaultCommand(new FeederCommand(feederSubsystem));

                rotaterSubsystem.setDefaultCommand(new RotaterCommand(rotaterSubsystem));

                intakeSubsystem.setDefaultCommand(new IntakeCommand(intakeSubsystem));
                
                lifterSubsystem.setDefaultCommand(new LifterCommand(lifterSubsystem, () -> manipulator.getLeftY(), () -> manipulator.getRightY(), () -> manipulator.getRightX()));
                
                shooterLifterSubsystem.setDefaultCommand(new ShooterLifterCommand(shooterLifterSubsystem, () -> manipulator.getRawAxis(3), ()-> manipulator.getRawAxis(2)));

                sensorsSubsystem.setRangeMode("Long");

                ledSubsystem.setDefaultCommand(new LEDCommand(ledSubsystem));

                autoChooser2 = AutoBuilder.buildAutoChooser();

                SmartDashboard.putData("Auto Chooser ", autoChooser2);
                autoChooser2.addOption("MStage4", AutoBuilder.buildAuto("MStage4"));
                SmartDashboard.putData("Selected: ", autoChooser2.getSelected());

                NamedCommands.registerCommand("intakeDefault", new IntakeCommand(intakeSubsystem));
                NamedCommands.registerCommand("rotater", new RotaterCommand(rotaterSubsystem));
                NamedCommands.registerCommand("intakeOn", intakeSubsystem.autoIntake());
                NamedCommands.registerCommand("rotaterIntake", rotaterSubsystem.autoIntake());
                NamedCommands.registerCommand("shooterMax", new ShooterMaxCommand(shooterSubsystem));
                NamedCommands.registerCommand("feed", feederSubsystem.feedPlease());
                NamedCommands.registerCommand("intakeOff", intakeSubsystem.intakeOff());
                NamedCommands.registerCommand("feedOff", feederSubsystem.feedOff());
                NamedCommands.registerCommand("shooterOff", new InstantCommand(shooterSubsystem::coast));
                NamedCommands.registerCommand("feedBack", new FeedBackCommand(feederSubsystem));
                NamedCommands.registerCommand("feedBack2", new FeedBackCommand(feederSubsystem));
                NamedCommands.registerCommand("shootStageCenter", new AutoShootFeed(feederSubsystem, shooterSubsystem, intakeSubsystem, rotaterSubsystem, -3));
                NamedCommands.registerCommand("shootAmp", new AutoShootFeed(feederSubsystem, shooterSubsystem, intakeSubsystem, rotaterSubsystem, -3));
                NamedCommands.registerCommand("shoot", new AutoShootClose(feederSubsystem, shooterSubsystem, intakeSubsystem, rotaterSubsystem, -3.5));
                NamedCommands.registerCommand("shoot2", new AutoShootClose(feederSubsystem, shooterSubsystem, intakeSubsystem, rotaterSubsystem, -8));
                NamedCommands.registerCommand("shoot3", new AutoShootClose(feederSubsystem, shooterSubsystem, intakeSubsystem, rotaterSubsystem, -8.8));


                configureButtonBindings(); 
        }
        

    private void configureButtonBindings() {
        driver_controller.y().onTrue((new InstantCommand(swerveSubsystem::zeroHeading)));
        
        driver_controller.a().whileTrue(new TrapCommand(shooterLifterSubsystem));  
        driver_controller.rightBumper().whileTrue(new TrackSpeakerCommand(swerveSubsystem, 
                () -> -driver_controller.getLeftX(),
                () -> -driver_controller.getLeftY() 
        ));
        /*driver_controller.x().whileTrue(new TrackRingCommand(swerveSubsystem,
                () -> -driver_controller.getLeftX(),
                () -> -driver_controller.getLeftY()
        ));*/
        //[TEMP] going to the amp is the left bumper
        /*driver_controller.leftBumper().whileTrue(new ToAmpCommand(swerveSubsystem, sensorsSubsystem));
        driver_controller.rightBumper().whileTrue(new TrackSpeakerCommand(swerveSubsystem, 
                () -> -driver_controller.getLeftX(), null
        ));*/

        driver_controller.leftBumper().onTrue(new InstantCommand(rotaterSubsystem::setAuto));
        driver_controller.x().onTrue(rotaterSubsystem.reset());
        driver_controller.b().onTrue(shooterLifterSubsystem.resetShooterLifter());

        manipulator.a().onTrue(intakeSubsystem.toggleIntake());
        manipulator.rightBumper().onTrue(intakeSubsystem.reverse());
        manipulator.b().whileTrue(new ShooterMaxCommandTeleop(shooterSubsystem));
        manipulator.y().onTrue(rotaterSubsystem.resetRotater());
        manipulator.x().onTrue(new AmpCommand(shooterLifterSubsystem));
        

        manipulator.pov(270).onTrue(new InstantCommand(shooterSubsystem::setSub));
        manipulator.pov(0).onTrue(lifterSubsystem.lifterUpTrap());
        manipulator.pov(90).onTrue(new InstantCommand(shooterSubsystem::setStage));

                
        if(FeederSubsystem.feederState != FeederState.FEED){
                manipulator.pov(180).onTrue(feederSubsystem.shootUpOBlock());
        }

        manipulator.leftBumper().onTrue(feederSubsystem.shootUpAmp());

        DriverStation.reportError("Intake State: " + IntakeSubsystem.intakeState, true);

        }

        /*public Command getAutonomousCommand(){
                return autoChooser.getSelected();
        }*/

        public Command getAutonomousCommand(){
                //swerveSubsystem.setPose(new Rotation2d(), new Pose2d(new Translation2d(0.71, 4.41), new Rotation2d())); //L1 and stagecenter
                swerveSubsystem.setPose(new Rotation2d(), new Pose2d(new Translation2d(1.37, 5.56), new Rotation2d())); //M12
                //swerveSubsystem.setPose(new Rotation2d(), new Pose2d(new Translation2d(1.47, 7), new Rotation2d())); //muktest
                //swerveSubsystem.setPose(new Rotation2d(), new Pose2d(new Translation2d(0.79, 6.67), new Rotation2d())); //AmpRing5
                //if(autoChooser.getSelected() == )

                return AutoBuilder.buildAuto("MStageMid");
        }    
}
