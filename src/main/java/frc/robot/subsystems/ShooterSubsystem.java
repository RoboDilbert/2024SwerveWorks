package frc.robot.subsystems;


import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase{
    /* 
    public static CANSparkMax shooterMotor = new CANSparkMax(Constants.SparkIDs.shooter1SparkID, MotorType.kBrushless);
    public static CANSparkMax shooterMotor2 = new CANSparkMax(Constants.SparkIDs.shooter2SparkID, MotorType.kBrushless);
    public double speed = 0;
    public ShooterSubsystem(){
        shooterMotor.restoreFactoryDefaults();
        shooterMotor.setIdleMode(IdleMode.kBrake);
        shooterMotor.setSmartCurrentLimit(20);
        shooterMotor2.restoreFactoryDefaults();
        shooterMotor2.setIdleMode(IdleMode.kBrake);
        shooterMotor2.setSmartCurrentLimit(20);

        setDefaultCommand(new ShooterCommand(this));
    }
    */
    public void periodic(){

        // NEED TO FIX!!!


        //ShooterCommand.run(0.1);

        //speed += 0.01*RobotContainer.manipulator.getLeftY();
        //    if(speed >= 1){
        //       speed = 1;
        //    } else if(speed <= -1){
        //        speed = -1;
        //    }

        //if(RobotContainer.manipulator.getRawButton(1)){
        //    ShooterCommand.run(speed);
        //} else {
        //    ShooterCommand.run(0);
        //}

        
    }
}
