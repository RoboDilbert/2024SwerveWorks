package frc.robot.subsystems;

import frc.robot.Constants;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

//https://usermanual.wiki/Pdf/REV20Blinkin20LED20Driver20Users20Manual.1683507547/html#pf8 has all of the PWM color IDs

public class LEDSubsystem extends SubsystemBase {

  private Spark ledPWMController;

  public LEDSubsystem() {
    ledPWMController = new Spark(Constants.LEDConstants.LED_PWM);
    //sets the PWM port it's wired to on the rio
  }

  public void setLEDMode(LEDMode ledMode) {
    // Sets a LED mode
    ledPWMController.set(ledMode.pwmSignal);
  }
  
  public void setLEDPWM(double PWM) {
    //Sets the PWM signal manually
    ledPWMController.set(PWM);
  }

  // Declare the preset LED modes we want to use
  public enum LEDMode {
    RING_IN(0.3),
    NO_RING(0.77),
    //generic ones idk
    TELEOP(-0.75),
    AUTO(-0.75);
  
    public double pwmSignal;
    LEDMode(double pwmSignal) {
      this.pwmSignal = pwmSignal;
    }
  }
}