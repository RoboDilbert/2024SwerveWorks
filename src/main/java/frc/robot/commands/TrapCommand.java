package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.RotaterSubsystem;
import frc.robot.subsystems.ShooterLifterSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.RotaterSubsystem.RotaterState;
import frc.robot.subsystems.ShooterLifterSubsystem.ShooterLifterState;
import frc.robot.subsystems.ShooterSubsystem.ShooterState;

public class TrapCommand extends Command{

    private boolean end;

    public TrapCommand(ShooterLifterSubsystem lift){
        end  = true;
        addRequirements(lift);
    }

    public void initialize(){
        if(ShooterLifterSubsystem.shooterLifterState == ShooterLifterState.MANUAL){
            RotaterSubsystem.rotaterState = RotaterState.SHOOT;
            ShooterSubsystem.shooterState = ShooterState.TRAP;
            ShooterLifterSubsystem.shooterLifterState = ShooterLifterState.TRAP;
        }
        else if(ShooterLifterSubsystem.shooterLifterState == ShooterLifterState.TRAP){
            RotaterSubsystem.rotaterState = RotaterState.INTAKE;
            ShooterSubsystem.shooterState = ShooterState.TRAP;
            ShooterLifterSubsystem.shooterLifterState = ShooterLifterState.DOWN;
        } 
    }

    public void execute(){
        end = true;
    }

    public boolean isFinished(){
        return end;
    }
    
}
