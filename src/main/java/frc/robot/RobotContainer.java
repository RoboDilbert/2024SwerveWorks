package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
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
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.commands.AmpCommand;
import frc.robot.commands.AutoMoveCommand;
import frc.robot.commands.AutoMoveIntake;
import frc.robot.commands.AutoShoot;
import frc.robot.commands.AutoShootClose;
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
import frc.robot.commands.ToSpeakerCommand;
import frc.robot.commands.TrackSpeakerCommand;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.FeederSubsystem.FeederState;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LifterSubsystem;
import frc.robot.subsystems.RotaterSubsystem;
import frc.robot.subsystems.SensorsSubsystem;
import frc.robot.subsystems.ShooterLifterSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.LEDSubsystem;


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

        private final SendableChooser<Command> autoChooser;


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

                autoChooser = AutoBuilder.buildAutoChooser();

                autoChooser.setDefaultOption("M2", M2);
                autoChooser.addOption("L1",  L1);
                autoChooser.addOption("R3", R3);
                autoChooser.addOption("M1", M1);
                autoChooser.addOption("M3", M3);
                autoChooser.addOption("M32", M32);
                autoChooser.addOption("M12", M12);
                autoChooser.addOption("M123", M123);
                autoChooser.addOption("TEST", TEST);

                SmartDashboard.putData("Auto Chooser", autoChooser);

                NamedCommands.registerCommand("intakeDefault", new IntakeCommand(intakeSubsystem));
                NamedCommands.registerCommand("rotater", new RotaterCommand(rotaterSubsystem));
                NamedCommands.registerCommand("intakeOn", intakeSubsystem.autoIntake());
                NamedCommands.registerCommand("rotaterIntake", rotaterSubsystem.autoIntake());
                NamedCommands.registerCommand("shooterMax", new ShooterMaxCommand(shooterSubsystem));
                NamedCommands.registerCommand("feed", feederSubsystem.feedPlease());
                NamedCommands.registerCommand("shooterOff", new InstantCommand(shooterSubsystem::coast));
                NamedCommands.registerCommand("shoot", new AutoShootClose(feederSubsystem, shooterSubsystem, intakeSubsystem, rotaterSubsystem));
                NamedCommands.registerCommand("shoot2", new AutoShootClose(feederSubsystem, shooterSubsystem, intakeSubsystem, rotaterSubsystem));
                NamedCommands.registerCommand("shoot3", new AutoShootClose(feederSubsystem, shooterSubsystem, intakeSubsystem, rotaterSubsystem));
                NamedCommands.registerCommand("intake1", new IntakeShooterAuto(feederSubsystem, shooterSubsystem, intakeSubsystem, rotaterSubsystem));
                NamedCommands.registerCommand("intake2", new IntakeShooterAuto(feederSubsystem, shooterSubsystem, intakeSubsystem, rotaterSubsystem));
                NamedCommands.registerCommand("intake3", new IntakeShooterAuto(feederSubsystem, shooterSubsystem, intakeSubsystem, rotaterSubsystem));


                configureButtonBindings(); 
        }
        

    private void configureButtonBindings() {
        driver_controller.y().onTrue((new InstantCommand(swerveSubsystem::zeroHeading)));
        
        driver_controller.a().whileTrue(new ToSpeakerCommand(swerveSubsystem));
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
        manipulator.pov(0).onTrue(new InstantCommand(shooterSubsystem::setLine));
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
                swerveSubsystem.setPose(new Rotation2d(), new Pose2d(new Translation2d(2, 7), new Rotation2d()));

                return AutoBuilder.buildAuto("test");
        }
        
        SequentialCommandGroup M2 = new SequentialCommandGroup(
                swerveSubsystem.setStaticHeading(0),
                new ShooterMaxCommand(shooterSubsystem),
                new WaitCommand(1),
                feederSubsystem.feedPlease(),
                new WaitCommand(.25),
                intakeSubsystem.autoIntake(),
                rotaterSubsystem.autoIntake(),
                new InstantCommand(shooterSubsystem::coast),
                new AutoMoveCommand(swerveSubsystem, rotaterSubsystem, shooterLifterSubsystem, 2.7, 0, true),
                new AutoShoot(feederSubsystem, shooterSubsystem, intakeSubsystem, rotaterSubsystem, 200, -11.5)
        );
        SequentialCommandGroup L1 = new SequentialCommandGroup(
                swerveSubsystem.setStaticHeading(40),
                new ShooterMaxCommand(shooterSubsystem),
                new WaitCommand(1),
                feederSubsystem.feedPlease(),
                new WaitCommand(.25),
                intakeSubsystem.autoIntake(),
                rotaterSubsystem.autoIntake(),
                new InstantCommand(shooterSubsystem::coast),
                new AutoMoveCommand(swerveSubsystem, rotaterSubsystem, shooterLifterSubsystem, 2.4, -1.4, false),
                new AutoMoveCommand(swerveSubsystem, rotaterSubsystem, shooterLifterSubsystem, 3, -1.4, false),
                new AutoShoot(feederSubsystem, shooterSubsystem, intakeSubsystem, rotaterSubsystem, 200, -11.9),
                new AutoMoveCommand(swerveSubsystem, rotaterSubsystem, shooterLifterSubsystem, 3, -1.4, false)
        );
        SequentialCommandGroup R3 = new SequentialCommandGroup(
                swerveSubsystem.setStaticHeading(-40),
                new ShooterMaxCommand(shooterSubsystem),
                new WaitCommand(1),
                feederSubsystem.feedPlease(),
                new WaitCommand(.25),
                intakeSubsystem.autoIntake(),
                rotaterSubsystem.autoIntake(),
                new InstantCommand(shooterSubsystem::coast),
                new AutoMoveCommand(swerveSubsystem, rotaterSubsystem, shooterLifterSubsystem, 2.4, 1, false),
                new AutoMoveCommand(swerveSubsystem, rotaterSubsystem, shooterLifterSubsystem, 3, 1.3, false),
                new AutoShoot(feederSubsystem, shooterSubsystem, intakeSubsystem, rotaterSubsystem, 200, -11.9),
                new AutoMoveCommand(swerveSubsystem, rotaterSubsystem, shooterLifterSubsystem, 3, 1.3, false)
        );
        SequentialCommandGroup M3 = new SequentialCommandGroup(
                swerveSubsystem.setStaticHeading(0),
                new ShooterMaxCommand(shooterSubsystem),
                new WaitCommand(1),
                feederSubsystem.feedPlease(),
                new WaitCommand(.25),
                intakeSubsystem.autoIntake(),
                rotaterSubsystem.autoIntake(),
                new InstantCommand(shooterSubsystem::coast),
                new AutoMoveCommand(swerveSubsystem, rotaterSubsystem, shooterLifterSubsystem, 3, 1.3, false),
                new AutoShoot(feederSubsystem, shooterSubsystem, intakeSubsystem, rotaterSubsystem, 200, -11.9),
                new AutoMoveCommand(swerveSubsystem, rotaterSubsystem, shooterLifterSubsystem, 3, 1.3, false)
        );

        SequentialCommandGroup M1 = new SequentialCommandGroup(
                swerveSubsystem.setStaticHeading(0),
                new ShooterMaxCommand(shooterSubsystem),
                new WaitCommand(1),
                feederSubsystem.feedPlease(),
                new WaitCommand(.25),
                intakeSubsystem.autoIntake(),
                rotaterSubsystem.autoIntake(),
                new InstantCommand(shooterSubsystem::coast),
                new AutoMoveCommand(swerveSubsystem, rotaterSubsystem, shooterLifterSubsystem, 3, -1.4, false),
                new AutoShoot(feederSubsystem, shooterSubsystem, intakeSubsystem, rotaterSubsystem, 200, -11.9),
                new AutoMoveCommand(swerveSubsystem, rotaterSubsystem, shooterLifterSubsystem, 3, -1.4, false)
        );
        SequentialCommandGroup M32 = new SequentialCommandGroup(
                swerveSubsystem.setStaticHeading(0),
                new ShooterMaxCommand(shooterSubsystem),
                new WaitCommand(.75),
                feederSubsystem.feedPlease(),
                new WaitCommand(.25),
                intakeSubsystem.autoIntake(),
                rotaterSubsystem.autoIntake(),
                new InstantCommand(shooterSubsystem::coast),
                new AutoMoveCommand(swerveSubsystem, rotaterSubsystem, shooterLifterSubsystem, 3, 1.3, false),
                new AutoShoot(feederSubsystem, shooterSubsystem, intakeSubsystem, rotaterSubsystem, 100, -11.9),
                new AutoMoveCommand(swerveSubsystem, rotaterSubsystem, shooterLifterSubsystem, 2, 0.2, false),
                intakeSubsystem.autoIntake(),
                feederSubsystem.feedPlease(),
                rotaterSubsystem.autoIntake(),
                new AutoMoveCommand(swerveSubsystem, rotaterSubsystem, shooterLifterSubsystem, 2.7, 0, false),
                new AutoShoot(feederSubsystem, shooterSubsystem, intakeSubsystem, rotaterSubsystem, 100, -11.5)
        );
        SequentialCommandGroup M12 = new SequentialCommandGroup(
                swerveSubsystem.setStaticHeading(0),
                new ShooterMaxCommand(shooterSubsystem),
                new WaitCommand(.75),
                feederSubsystem.feedPlease(),
                new WaitCommand(.25),
                intakeSubsystem.autoIntake(),
                rotaterSubsystem.autoIntake(),
                new InstantCommand(shooterSubsystem::coast),
                new AutoMoveCommand(swerveSubsystem, rotaterSubsystem, shooterLifterSubsystem, 3, -1.3, false),
                new AutoShoot(feederSubsystem, shooterSubsystem, intakeSubsystem, rotaterSubsystem, 100, -11.9),
                new AutoMoveCommand(swerveSubsystem, rotaterSubsystem, shooterLifterSubsystem, 2, -0.2, false),
                intakeSubsystem.autoIntake(),
                feederSubsystem.feedPlease(),
                rotaterSubsystem.autoIntake(),
                new AutoMoveCommand(swerveSubsystem, rotaterSubsystem, shooterLifterSubsystem, 2.7, 0, false),
                new AutoShoot(feederSubsystem, shooterSubsystem, intakeSubsystem, rotaterSubsystem, 100, -11.5)
        );
        SequentialCommandGroup M123 = new SequentialCommandGroup(
                swerveSubsystem.setStaticHeading(0),
                new ShooterMaxCommand(shooterSubsystem),
                new WaitCommand(.25),
                feederSubsystem.feedPlease(),
                new WaitCommand(.25),
                intakeSubsystem.autoIntake(),
                rotaterSubsystem.autoIntake(),
                new InstantCommand(shooterSubsystem::coast),
                new AutoMoveIntake(swerveSubsystem, rotaterSubsystem, intakeSubsystem, feederSubsystem, 3.6, -2, false),
                new AutoMoveIntake(swerveSubsystem, rotaterSubsystem, intakeSubsystem, feederSubsystem, 1.75, 0, false),
                new AutoShootClose(feederSubsystem, shooterSubsystem, intakeSubsystem, rotaterSubsystem),
                feederSubsystem.feedPlease(),
                intakeSubsystem.autoIntake(),
                rotaterSubsystem.autoIntake(),
                new InstantCommand(shooterSubsystem::coast),
                new AutoMoveIntake(swerveSubsystem, rotaterSubsystem, intakeSubsystem, feederSubsystem, 3.5, 0, false),
                new AutoMoveIntake(swerveSubsystem, rotaterSubsystem, intakeSubsystem, feederSubsystem, 1.75, 0, false),
                new AutoShootClose(feederSubsystem, shooterSubsystem, intakeSubsystem, rotaterSubsystem),
                feederSubsystem.feedPlease(),
                intakeSubsystem.autoIntake(),
                rotaterSubsystem.autoIntake(),
                new InstantCommand(shooterSubsystem::coast),
                new AutoMoveIntake(swerveSubsystem, rotaterSubsystem, intakeSubsystem, feederSubsystem, 3.5, 1.8, false),
                new AutoMoveIntake(swerveSubsystem, rotaterSubsystem, intakeSubsystem, feederSubsystem, 1.75, 0, false),
                new AutoShootClose(feederSubsystem, shooterSubsystem, intakeSubsystem, rotaterSubsystem),
                new InstantCommand(shooterSubsystem::coast)
        );
        SequentialCommandGroup TEST = new SequentialCommandGroup(
                swerveSubsystem.setStaticHeading(0),
                new ShooterMaxCommand(shooterSubsystem),
                new WaitCommand(.25),
                feederSubsystem.feedPlease(),
                new WaitCommand(.25),
                intakeSubsystem.autoIntake(),
                rotaterSubsystem.autoIntake(),
                new InstantCommand(shooterSubsystem::coast),
                new AutoMoveIntake(swerveSubsystem, rotaterSubsystem, intakeSubsystem, feederSubsystem, 3.3, 0, false),
                new AutoMoveIntake(swerveSubsystem, rotaterSubsystem, intakeSubsystem, feederSubsystem, 1.75, 0, false),
                new AutoShootClose(feederSubsystem, shooterSubsystem, intakeSubsystem, rotaterSubsystem),
                new InstantCommand(shooterSubsystem::coast)
        );
}
