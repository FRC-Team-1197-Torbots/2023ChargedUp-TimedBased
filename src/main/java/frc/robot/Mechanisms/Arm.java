package frc.robot.Mechanisms;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Mechanisms.LED.setLED;
import edu.wpi.first.math.controller.PIDController;

public class Arm{

    private CANSparkMax armMotor1;
    private AnalogPotentiometer armPot;
    private double target;
    public RunArm m_state;
    public boolean ontarget;
    private double currentPosition;
    private double speed;
    private boolean isFinished;
    //new pot value for mid 0.0484
    //0.992
    //0.614
    public final double STOREDBL = 0.970f;//0.463f;//0.805f;0.486;0.992
    public final double SCOREDBL = 0.645;//0.112f;//0.441f;0.132
    public final double SCOREMID = 0.0484;
    public final double DOWNDBL = 0.121f;
    private final double MAX_ARM_OUTPUT = 0.2;
    private AutoArmDirection m_ArmDirection;
    public static enum RunArm { 
        STORE, DOWN, HORIZONTAL, IDLE,ANGLEDUP
    }
    public static enum AutoArmDirection{
        UP, DOWN
    }


    public Arm(){
        //armMotor = new CANSparkMax(ElevatorArmConstants.ArmID, MotorType.kBrushless);
        //armSwitch1 = new DigitalInput(ElevatorArmConstants.armSwitch1Port);
        //armSwitch2 = new DigitalInput(ElevatorArmConstants.armSwitch2Port);
        armMotor1 = new CANSparkMax(11, MotorType.kBrushless);
        armPot = new AnalogPotentiometer(0);
        //armPID = new PIDController(1, 0, 0);
        armMotor1.setIdleMode(IdleMode.kBrake);
        SetState(RunArm.IDLE);     
    }

    public double getMaxOutput(){
        return MAX_ARM_OUTPUT;
    }


    public void SetState(RunArm state) {
        m_state = state;

        switch(m_state) {
            case STORE:
                target = STOREDBL;
            break;

            case DOWN:
                target = DOWNDBL;
            break;

            case HORIZONTAL:
                target = SCOREDBL;
            break;
            case ANGLEDUP:
                target = SCOREMID;

            case IDLE:
                target = GetPotValue();
            break;
        }


        ontarget = false;
    }

    //positive motor power goes towards robot
    //store = 0.803
    //horizontal = 0.441
    //straight down = 0.695
    public void run(){
        currentPosition = GetPotValue();
        //System.out.println("Pot value" + currentPosition);
        //System.out.println("Current Angle: " + currentPosition);
        speed = 0;

       if(!ontarget) {                   

            switch(m_state){
                case STORE:

                    if(currentPosition < STOREDBL) {
                        speed = 0.25;//0.45;
                    } else {
                        speed = -0.2;
                    }
                
                break;

                case DOWN:
                    if(currentPosition < DOWNDBL) {
                        speed = 0.25;//0.37;
                    } else {
                        speed = -0.2;
                    }
                break;

                case HORIZONTAL:
                    if(currentPosition < SCOREDBL) {
                        speed = 0.2;//0.2;//0.30;
                    } else {
                        speed = -0.2;
                    }
                break;
                case ANGLEDUP:
                    if(currentPosition < SCOREMID) {
                        speed = 0.2;//0.2;//0.30;
                    } else {
                        speed = -0.2;
                    }
                break;
            }
            
        } 

        if(Math.abs(target - currentPosition) < 0.005f) {
            ontarget = true;
            //speed = 0;
        }

        if(ontarget && m_state == RunArm.HORIZONTAL) {
            speed = -0.005f;
        }
        if(ontarget && m_state == RunArm.ANGLEDUP) {
            speed = -0.005f;
        }

        SetArmSpeed(speed * 3.9f);

        SmartDashboard.putNumber("CurrentPosition Arm", currentPosition);
        //SmartDashboard.putBoolean("OnTarget Arm", ontarget);
        //SmartDashboard.putNumber("Arm Error", (target - currentPosition));
        //SmartDashboard.putNumber("Arm target", target);

    }

    public void autoRun(AutoArmDirection armDirection){
        currentPosition = GetPotValue();
        speed = 0;
        switch(m_ArmDirection){
             
            case UP:
                target = SCOREDBL;
                if(currentPosition < target) {
                    speed = 0.2;
                } else {
                    speed = -0.2;
                }
                break;
            case DOWN:
                target = STOREDBL;
                if(currentPosition < target) {
                    speed = 0.2;
                } else {
                    speed = -0.2;
                }
                break;
        }
        if(Math.abs(target - currentPosition) < 0.005f) {
            ontarget = true;
            //isFinished = true;
            speed = 0;
        }
        else{
            isFinished = false;
        }

    }
    
    public boolean isDone(){
        return ontarget;
    }

    public void resetTaget(){
        ontarget = false;
    }

    /*
    public void SetArmState(STATE state){
        m_armState = state;
    }*/
    public void SetArmSpeed(double speed){
        armMotor1.set(speed);

    }

    public double GetError() {
        return target - GetPotValue();
    }
    

    public double GetPotValue(){
        return armPot.get();
    }
    public double getArmVoltage(){
        return armMotor1.get();
    }
}