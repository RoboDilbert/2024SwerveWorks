package frc.robot.subsystems;

import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SensorsSubsystem extends SubsystemBase{
    // init tof sensors
    public static TimeOfFlight lidar1 = new TimeOfFlight(Constants.SensorConstants.lidarID1);
    public static TimeOfFlight lidar2 = new TimeOfFlight(Constants.SensorConstants.lidarID2);
    
    public double leftDistance = 0;
    public double rightDistance = 0;
    public double avgDistance = 0;
    public boolean isSeeing = false;
    public boolean byAmp = false;



    //Time of Flight Sensors (Outside of Swerve)

    public boolean isSeeing() {
        leftDistance = lidar1.getRange();
        rightDistance = lidar2.getRange();
        avgDistance = (rightDistance + leftDistance) / 2;

        if(lidar1.getRange() >  0 || lidar2.getRange() > 0){
            isSeeing = true;
        } else {
            isSeeing = false;
        }
        return isSeeing;
    }

    public void seeingAmp() {
        if (byAmp) {
            byAmp = false;
        } else {
            byAmp = true;
        }
    }

    public boolean getIfByAmp() {
        return byAmp;
    }

    public double getDistance() {
        avgDistance = (lidar1.getRange() + lidar2.getRange()) / 2;
        return avgDistance;
    }

    public double  getLeftDistance() {
        return lidar1.getRange();
    }

    public double  getRightDistance() {
        return lidar2.getRange();
    }
   
   public boolean isCloseEnough(boolean sight) {
       if(lidar1.isRangeValid() && lidar2.isRangeValid()){
           return true;
       }
       else{
           return false;
       }
   }

   public void setRangeMode(String mode) {
       if(mode.equals("Long")) {
           lidar1.setRangingMode(RangingMode.Long, 100);
           lidar2.setRangingMode(RangingMode.Long, 100);
       }
       else if(mode.equals("Medium")) {
           lidar1.setRangingMode(RangingMode.Medium, 100);
           lidar2.setRangingMode(RangingMode.Medium, 100);
       }
       else if(mode.equals("Short")) {
           lidar1.setRangingMode(RangingMode.Short, 100);
           lidar2.setRangingMode(RangingMode.Short, 100);
       }
   }

   public boolean areTOFEqual() {
       if(lidar1.getRange() > 100 && lidar1.getRange() < 120 && lidar2.getRange() < 100 && lidar2.getRange() > 120) {
           return true;
       }
       else{
           return false;
       }
   }

   public void periodic() {
       double leftFlightSensor = lidar1.getRange();
       double rightFlightSensor = lidar2.getRange();
       double avgDistance = (leftFlightSensor + rightFlightSensor)/2;
       
       
       //SmartDashboard.putNumber("Left TOF", leftFlightSensor);
       SmartDashboard.putBoolean("Left TOF isSeeing?", isSeeing());
       //SmartDashboard.putNumber("Left TOF", lidar1.getRange());
       SmartDashboard.putNumber("Right TOF", getDistance());
       SmartDashboard.updateValues();
   }

}