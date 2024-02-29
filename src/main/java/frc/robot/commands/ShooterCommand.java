package frc.robot.commands;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterCommand extends Command{

    private final ShooterSubsystem m_shooterSubsystem;
    private final DoubleSupplier m_power;
    public double power = 0;

    public ShooterCommand(ShooterSubsystem shooter, DoubleSupplier power){
        m_shooterSubsystem = shooter;
        m_power = power;
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
        if(Math.abs(m_power.getAsDouble()) > .1)
            m_shooterSubsystem.run(m_power);
        else
            m_shooterSubsystem.coast();
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
