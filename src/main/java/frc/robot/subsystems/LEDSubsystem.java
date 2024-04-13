package frc.robot.subsystems;

import frc.robot.Constants;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

//https://usermanual.wiki/Pdf/REV20Blinkin20LED20Driver20Users20Manual.1683507547/html#pf8 has all of the PWM color IDs

public class LEDSubsystem extends SubsystemBase {

  private Spark ledPWMController;

  public enum LEDState{
    OFF,
    INTAKE,
    RING,
    SHOOT
  }

  public static LEDState ledState = LEDState.OFF;

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
    RING(0.87),
    PROBLEMA(-0.11),
    SHOOT(0.77),
    INTAKE(0.69),
    OFF(.99);
    //red: 0.61 green: 0.77 blue: 0.87

    public double pwmSignal;
    LEDMode(double pwmSignal) {
      this.pwmSignal = pwmSignal;
    }
  }
}