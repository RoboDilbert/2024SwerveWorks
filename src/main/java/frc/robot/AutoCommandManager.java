package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.AutoShootClose;
import frc.robot.commands.AutoShootFeed;
import frc.robot.commands.FeedBackCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.RotaterCommand;
import frc.robot.commands.ShooterMaxCommand;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.RotaterSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class AutoCommandManager {

    SendableChooser<Command> m_chooser = new SendableChooser<>();

    public AutoCommandManager(SwerveSubsystem swerveSubsystem, 
        ShooterSubsystem shooterSubsystem,
        IntakeSubsystem intakeSubsystem,
        RotaterSubsystem rotaterSubsystem,
        FeederSubsystem feederSubsystem) {
        configureNamedCommands(swerveSubsystem, shooterSubsystem, intakeSubsystem, rotaterSubsystem, feederSubsystem);
            
        PathPlannerAuto mstage4 = new PathPlannerAuto("MStage4");
        PathPlannerAuto mstage3 = new PathPlannerAuto("MStage3");
        PathPlannerAuto mamp3 = new PathPlannerAuto("MAmp3");
        PathPlannerAuto amp2 = new PathPlannerAuto("Amp2");
        PathPlannerAuto stage2 = new PathPlannerAuto("Stage2");
        PathPlannerAuto m2 = new PathPlannerAuto("M2");
        PathPlannerAuto center21 = new PathPlannerAuto("Center21");
        PathPlannerAuto center31 = new PathPlannerAuto("Center31");
        PathPlannerAuto ampring5 = new PathPlannerAuto("AmpRing5");

        m_chooser.setDefaultOption("None", null);
        m_chooser.addOption("MStage4", mstage4);
        m_chooser.addOption("MStage3", mstage3);
        m_chooser.addOption("MAmp3", mamp3);
        m_chooser.addOption("Amp2", amp2);
        m_chooser.addOption("Stage2", stage2);
        m_chooser.addOption("M2", m2);
        m_chooser.addOption("Center21", center21);
        m_chooser.addOption("Center31", center31);
        m_chooser.addOption("AmpRing5", ampring5);

        SmartDashboard.putData("SelectAuto", m_chooser);
    }

    public SendableChooser<Command> getChooser() {
        return m_chooser;
    }

    public Command getAutoManagerSelected(){
        return m_chooser.getSelected();
    }

    public void configureNamedCommands(SwerveSubsystem swerveSubsystem, 
        ShooterSubsystem shooterSubsystem,
        IntakeSubsystem intakeSubsystem,
        RotaterSubsystem rotaterSubsystem,
        FeederSubsystem feederSubsystem) { 
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
        NamedCommands.registerCommand("shoot2", new AutoShootClose(feederSubsystem, shooterSubsystem, intakeSubsystem, rotaterSubsystem, -7.8));
        NamedCommands.registerCommand("shoot3", new AutoShootClose(feederSubsystem, shooterSubsystem, intakeSubsystem, rotaterSubsystem, -8.9));
        NamedCommands.registerCommand("rotaterShoot", intakeSubsystem.rotaterShoot());

    }
}