package frc.robot.Mechanisms;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Solenoid;
//import frc.robot.Constants.IntakeHopperConstants;
//import frc.robot.Constants.ElevatorArmConstants.GamePiece;

//Right Bumber cone,Left Bumber cube
public class Intake{

    private Solenoid Intakeout;
    private CANSparkMax RollerBottom;
    private CANSparkMax RollerTop;
    private Encoder IntakeEncoder;

    private boolean isFinished;
    private boolean currentIntakeState;

    public enum moveIntake{
        UP, DOWN
    }
    public static enum autoIntakeState{
        INIT, RUN
    }
    public moveIntake intakeState = moveIntake.UP;
    private autoIntakeState m_AutoIntakeState;

    public Intake(){
        Intakeout = new Solenoid(PneumaticsModuleType.REVPH, 1);
        RollerBottom = new CANSparkMax(9, MotorType.kBrushless);
        RollerTop = new CANSparkMax(10, MotorType.kBrushless);
        isFinished = false;
        //IntakeEncoder = new Encoder(0, 0);
        //IntakeEncoder.reset();
        
    }

    public void TriggerIntake() {
        Intakeout.toggle();
    }

    
    public void run(moveIntake intakeState){
        this.intakeState = intakeState;
        switch(intakeState){
            case UP:
                Intakeout.set(false);
                RollerTop.set(0);
                RollerBottom.set(0);
                break;
            case DOWN:
                Intakeout.set(true);
                RollerTop.set(-0.5);
                RollerBottom.set(0.6);
                break;
        }
    }

    public void autoRun(){
        //m_AutoIntakeState = autoIntakeState.INIT;
        switch(m_AutoIntakeState){
            case INIT:
                isFinished = false;
                System.out.println("Init intake");
                TriggerIntake();
                if(!currentIntakeState){
                    isFinished = true;
                }
                //m_AutoIntakeState = autoIntakeState.RUN;
                //currentIntakeState = Intakeout.get();
                break;
            case RUN:
                TriggerIntake();
                if(!currentIntakeState){
                    isFinished = true;
                }
                break;

        }
        
    }

    public void setAutoState(autoIntakeState state){
        m_AutoIntakeState = state;

    }

    public boolean isDone(){
        return isFinished;
    }
}