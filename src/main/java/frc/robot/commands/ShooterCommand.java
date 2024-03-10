package frc.robot.commands;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterSubsystem.ShooterSpeedState;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterCommand extends Command{

    private final ShooterSubsystem m_shooterSubsystem;
    public double power = 0;

    public ShooterCommand(ShooterSubsystem shooter, DoubleSupplier power){
        m_shooterSubsystem = shooter;
        addRequirements(shooter);
    }

    public void initialize(){
    }

    public double evalPower(){
        power = Constants.ShooterConstants.kIdleSpeed;
        power += Constants.ShooterConstants.kPowerDistanceMultiplier;   // *LIDAR_DISTANCE_VALUE
        power += Constants.ShooterConstants.kPowerSpeedMultiplier;      // *ROBOT_SPEED_Y_VALUE
        return power;
    }

    public boolean powerReasonable() {
        if(power < Constants.ShooterConstants.maxPower) {
            return true;
        } else {
            return false;
        }
    }

    public void execute(){
        if(ShooterSubsystem.speedState == ShooterSpeedState.OFF){
            m_shooterSubsystem.coast();
        }
        else if(ShooterSubsystem.speedState == ShooterSpeedState.MAX){
            m_shooterSubsystem.maxSpeed();
        }
    }

    public void execute_auto_shooting(){
        if(powerReasonable()){
            m_shooterSubsystem.run(() -> evalPower());
        } else {
            m_shooterSubsystem.coast();
        }
    }

    public boolean isFinished(){
        return false;
    }
    
}
