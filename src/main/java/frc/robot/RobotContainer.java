package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathPlannerAuto;
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
    public final LEDSubsystem ledSubsystem = new LEDSubsystem();
    private final ShooterLifterSubsystem shooterLifterSubsystem = new ShooterLifterSubsystem();
    

    CommandXboxController driver_controller = new CommandXboxController(0);
    CommandXboxController manipulator = new CommandXboxController(2); 

    private AutoCommandManager m_autoManager = new AutoCommandManager(swerveSubsystem, shooterSubsystem, intakeSubsystem, rotaterSubsystem, feederSubsystem);

        public RobotContainer() {
                swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(swerveSubsystem,
                        () -> -driver_controller.getLeftX(),
                        () -> -driver_controller.getLeftY(),
                        () -> -driver_controller.getRawAxis(4),
                        () -> driver_controller.getRawAxis(3),
                        () -> driver_controller.getRawAxis(2),
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
                
                configureButtonBindings(); 
        }
        

    private void configureButtonBindings() {
        driver_controller.y().onTrue((new InstantCommand(swerveSubsystem::zeroHeading)));
        
        driver_controller.a().whileTrue(new TrapCommand(shooterLifterSubsystem));  
        driver_controller.rightBumper().whileTrue(new TrackSpeakerCommand(swerveSubsystem, 
                () -> -driver_controller.getLeftX(),
                () -> -driver_controller.getLeftY() 
        ));

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

        }

        /*public Command getAutonomousCommand(){
                return autoChooser.getSelected();
        }*/

        public Command getAutonomousCommand(){
                Command autoCommand = m_autoManager.getAutoManagerSelected();

                //MStage4 com.pathplanner.lib.commands.PathPlannerAuto@12a209c
                //MStage3 com.pathplanner.lib.commands.PathPlannerAuto@66af20
                //MAmp3 com.pathplanner.lib.commands.PathPlannerAuto@93b025
                //Amp2 com.pathplanner.lib.commands.PathPlannerAuto@7ddf94
                //Stage2 com.pathplanner.lib.commands.PathPlannerAuto@13ac989
                //M2 com.pathplanner.lib.commands.PathPlannerAuto@505305
                //Center21 com.pathplanner.lib.commands.PathPlannerAuto@12ea799
                //Center31 com.pathplanner.lib.commands.PathPlannerAuto@c03695
                //AmpRing5 com.pathplanner.lib.commands.PathPlannerAuto@9754d8

                if(m_autoManager.getAutoManagerSelected().toString().equals("com.pathplanner.lib.commands.PathPlannerAuto@66af20") || m_autoManager.getAutoManagerSelected().toString().equals("com.pathplanner.lib.commands.PathPlannerAuto@12a209c") || m_autoManager.getAutoManagerSelected().toString().equals("com.pathplanner.lib.commands.PathPlannerAuto@93b025") || m_autoManager.getAutoManagerSelected().toString().equals("com.pathplanner.lib.commands.PathPlannerAuto@505305")){ //MStage4, MStage3, MAmp3, M2
                        swerveSubsystem.setPose(new Rotation2d(), new Pose2d(new Translation2d(1.37, 5.56), new Rotation2d())); 
                }
                else if(m_autoManager.getAutoManagerSelected().toString().equals("com.pathplanner.lib.commands.PathPlannerAuto@7ddf94")){ //Amp2
                        swerveSubsystem.setPose(new Rotation2d(), new Pose2d(new Translation2d(0.76, 6.78), new Rotation2d())); 
                }
                else if(m_autoManager.getAutoManagerSelected().toString().equals("com.pathplanner.lib.commands.PathPlannerAuto@13ac989")){ //Stage2
                        swerveSubsystem.setPose(new Rotation2d(), new Pose2d(new Translation2d(0.74, 4.41), new Rotation2d()));
                }
                else if(m_autoManager.getAutoManagerSelected().toString().equals("com.pathplanner.lib.commands.PathPlannerAuto@12ea799") || m_autoManager.getAutoManagerSelected().toString().equals("com.pathplanner.lib.commands.PathPlannerAuto@c03695")){ //Center21 and Center31
                        swerveSubsystem.setPose(new Rotation2d(), new Pose2d(new Translation2d(0.71, 4.41), new Rotation2d()));
                }
                else if(m_autoManager.getAutoManagerSelected().toString().equals("com.pathplanner.lib.commands.PathPlannerAuto@9754d8")){ //AmpRing5
                        swerveSubsystem.setPose(new Rotation2d(), new Pose2d(new Translation2d(0.79, 6.61), new Rotation2d()));
                }
                
                SmartDashboard.putString("Auto Selected", m_autoManager.getAutoManagerSelected().toString());
                return autoCommand;
        }    
}
