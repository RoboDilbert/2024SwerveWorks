package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.LEDSubsystem.LEDMode;

public class LEDCommand extends Command{

    private final LEDSubsystem m_ledSubsystem;

    public LEDCommand(LEDSubsystem nitin){
        m_ledSubsystem = nitin;
        addRequirements(nitin);
        //m_ledSubsystem.setLEDPWM(0.93);

    }

    public void initialize(){
    }

    public void execute(){
        //m_ledSubsystem.setLEDMode(LEDMode.RING_IN);
        m_ledSubsystem.setLEDMode(LEDMode.OFF);
    }    

    public boolean isFinished(){
        return false;
    }

    public void periodic() {
        m_ledSubsystem.setLEDPWM(0.93);

    }
    
}
