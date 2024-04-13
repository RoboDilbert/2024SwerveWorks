package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.LEDSubsystem.LEDMode;
import frc.robot.subsystems.LEDSubsystem.LEDState;


public class LEDCommand extends Command{

    private final LEDSubsystem m_ledSubsystem;

    public LEDCommand(LEDSubsystem nitin){
        m_ledSubsystem = nitin;
        addRequirements(nitin);
    }

    public void initialize(){
    }

    public void execute(){
        if(LEDSubsystem.ledState == LEDState.OFF){
            m_ledSubsystem.setLEDMode(LEDMode.OFF);
        }
        if(LEDSubsystem.ledState == LEDState.INTAKE){
            m_ledSubsystem.setLEDMode(LEDMode.INTAKE);
        }
         if(LEDSubsystem.ledState == LEDState.RING){
            m_ledSubsystem.setLEDMode(LEDMode.RING);
        }
         if(LEDSubsystem.ledState == LEDState.SHOOT){
            m_ledSubsystem.setLEDMode(LEDMode.SHOOT);
        }
    }    

    public boolean isFinished(){
        return false;
    }

    public void periodic() {
        m_ledSubsystem.setLEDPWM(0.93);

    }
    
}
