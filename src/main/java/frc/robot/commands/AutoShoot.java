package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.FeederSubsystem.FeederState;
import frc.robot.subsystems.IntakeSubsystem.IntakeState;
import frc.robot.subsystems.RotaterSubsystem;
import frc.robot.subsystems.RotaterSubsystem.RotaterState;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class AutoShoot extends Command{

    private final FeederSubsystem m_feederSubsystem;
    private final ShooterSubsystem m_shooterSubsystem;
    private final IntakeSubsystem m_intakeSubsystem;
    private final RotaterSubsystem m_rotaterSubsystem;
    private final double shootTimer;
    private final double shooterPos;

    private boolean backSpin;
    private boolean shoot;
    private boolean shooterSpin;
    private double initialPosBack;
    private double initialPosShoot;
    private boolean endShoot;
    private boolean end;
    private double wait;


    public AutoShoot(FeederSubsystem feed, ShooterSubsystem shooter, IntakeSubsystem intake, RotaterSubsystem rotaterSubsystem, double shootTimer, double shooterPos){
        m_feederSubsystem = feed;
        m_shooterSubsystem = shooter;
        m_intakeSubsystem = intake;
        m_rotaterSubsystem = rotaterSubsystem;
        this.shootTimer = shootTimer;
        this.shooterPos = shooterPos;
        backSpin = false;
        shoot = false;
        shooterSpin = false;
        initialPosBack = 0;
        initialPosShoot = 0;
        endShoot = false;
        wait = 0;
        end = false;
        addRequirements(feed);
    }

    public void initialize(){
    }

    public void execute(){
        if(RotaterSubsystem.rotaterState == RotaterState.INTAKE && !shooterSpin){
            m_rotaterSubsystem.toPosition(Constants.TeleOpConstants.kRotaterIntakePosition);
        } 
        else if(RotaterSubsystem.rotaterState == RotaterState.INTAKE && shooterSpin){
            m_rotaterSubsystem.toPosition(shooterPos);
        } 

        if(m_intakeSubsystem.getDistance() < 45 && !shooterSpin){
            m_intakeSubsystem.run(0);
            FeederSubsystem.feederState = FeederState.BACK;
            IntakeSubsystem.intakeState = IntakeState.OFF;
            m_intakeSubsystem.run(0);
            m_feederSubsystem.feed(() -> 0);
        }
        if(FeederSubsystem.feederState == FeederState.BACK){
            if(!backSpin){
                backSpin = true;
                initialPosBack = m_feederSubsystem.getFeederPosition();
                m_feederSubsystem.feed(() -> .125);
            }
            if(backSpin){
                m_feederSubsystem.feed(() -> .125);
                if(m_feederSubsystem.getFeederPosition() > initialPosBack){
                    backSpin = false;
                    m_feederSubsystem.feed(() -> 0);
                    FeederSubsystem.feederState = FeederState.OFF;
                    shooterSpin = true;
                    m_shooterSubsystem.maxSpeed();
                }
            }
        }
        if(FeederSubsystem.feederState == FeederState.OFF && shooterSpin && !shoot){
            wait+=1;
            m_shooterSubsystem.maxSpeed();
        }
        if(FeederSubsystem.feederState == FeederState.OFF && wait > shootTimer && !shoot){
            shoot = true;
            initialPosShoot = m_feederSubsystem.getFeederPosition();
            m_feederSubsystem.feed(() -> -1);
        }    
        if(shoot){
            m_feederSubsystem.feed(() -> -1);
            m_shooterSubsystem.maxSpeed();
            if(m_feederSubsystem.getFeederPosition() < initialPosShoot - 200){
                shoot = false;
                m_feederSubsystem.feed(() -> 0);
                FeederSubsystem.feederState = FeederState.OFF;
                shooterSpin = false;
                endShoot = true;
            }
        }
        if(FeederSubsystem.feederState == FeederState.OFF && wait > shootTimer && shoot == false && endShoot){
            m_shooterSubsystem.coast();
            end = true;
        }
    }

    public boolean isFinished(){
        return end;
    }
    
}
