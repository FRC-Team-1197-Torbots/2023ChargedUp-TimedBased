package frc.robot.Mechanisms;


import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;

public class Claw{
    private Solenoid clawSolenoid;
    private CANSparkMax clawMotor;
    private boolean clawState;
    private boolean clawTarget;
    private boolean isFinished;
    private double speed;
    private double ConeIntakeSpeed = 0.3;
    private double CubeIntakeSpeed = 0.5;
    private double CubeEjectSpeed = -0.4;
    private double IdleSpeed = 0.02;
    private autoClawState m_aClawState;

    public static enum autoClawState{
        OPEN, CLOSE
    }  

    public Claw(){
        clawSolenoid = new Solenoid(PneumaticsModuleType.REVPH, 2);
        clawMotor = new CANSparkMax(14, MotorType.kBrushless);
    }

    public static enum RunClaw{
        OPEN,CLOSE,NOSOLENOID
    }

    public void SetSpeed(double speed) {
        SetClawSpeed(speed);
    }

    public void TriggerSolenoid() {
        clawSolenoid.toggle();
    }

    public void run(RunClaw ClawState, double speed){
        switch(ClawState){
            case OPEN:
                clawSolenoid.set(true);
                
            case CLOSE:
                clawSolenoid.set(false);
                
            case NOSOLENOID:
                SetClawSpeed(speed);
        }
    }

    public void autoRun(){
        setAutoClawState(autoClawState.OPEN);
        System.out.println("running claw");
        
        
    }

    public void setAutoClawState(autoClawState autoState){
        m_aClawState = autoState;
        isFinished = false;
        switch(m_aClawState){
            case OPEN:
                clawTarget = false;
                break;
            case CLOSE:
                clawTarget = true;
                break;

        }
        
    }
    public boolean getTarget(){
        return clawTarget;
    }

    public boolean isDone(){
        return isFinished;
    }

    public void resetFinished(){
        isFinished = false;
    }

    public void SetClawSpeed(double speed){
        clawMotor.set(speed);
    }

    public void dropClaw(){
        //clawSolenoid.set(!clawSolenoid.get());
        clawSolenoid.toggle();
        //System.out.println("Claw scolenoid: " + clawSolenoid.get());
    }

    public boolean getSolenoidValue(){
        return clawSolenoid.get();
    }
    
    public double getCurrent(){
        return clawMotor.getOutputCurrent();
    }
}