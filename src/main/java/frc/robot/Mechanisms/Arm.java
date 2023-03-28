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
    private RunArm m_state;
    private boolean ontarget;

    private final double STOREDBL = 0.805f;
    private final double SCOREDBL = 0.441f;
    private final double DOWNDBL = 0.655f;

    public static enum RunArm { 
        STORE, DOWN, HORIZONTAL
    }


    public Arm(){
        //armMotor = new CANSparkMax(ElevatorArmConstants.ArmID, MotorType.kBrushless);
        //armSwitch1 = new DigitalInput(ElevatorArmConstants.armSwitch1Port);
        //armSwitch2 = new DigitalInput(ElevatorArmConstants.armSwitch2Port);
        armMotor1 = new CANSparkMax(11, MotorType.kBrushless);
        armPot = new AnalogPotentiometer(0);
        //armPID = new PIDController(1, 0, 0);
        armMotor1.setIdleMode(IdleMode.kBrake);
        SetState(RunArm.STORE);     
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
        }


        ontarget = false;
    }

    //positive motor power goes towards robot
    //store = 0.803
    //horizontal = 0.441
    //straight down = 0.695
    public void run(){
        double currentPosition = GetPotValue();
        double speed = 0;

        if(!ontarget) {                   

            switch(m_state){
                case STORE:
                    if(currentPosition < STOREDBL) {
                        speed = 0.2;
                    } else {
                        speed = -0.2;
                    }
                
                break;

                case DOWN:
                    if(currentPosition < DOWNDBL) {
                        speed = 0.2;
                    } else {
                        speed = -0.2;
                    }
                break;

                case HORIZONTAL:
                    if(currentPosition < SCOREDBL) {
                        speed = 0.2;
                    } else {
                        speed = -0.2;
                    }
                break;
            }
        } 

        if(Math.abs(target - currentPosition) < 0.005f) {
            ontarget = true;
            speed = 0;
        }

        if(ontarget && m_state == RunArm.HORIZONTAL) {
            speed = -0.01f;
        }

        SetArmSpeed(speed * 1.5f);

        SmartDashboard.putNumber("CurrentPosition Arm", currentPosition);
        SmartDashboard.putBoolean("OnTarget Arm", ontarget);
        SmartDashboard.putNumber("Arm Error", (target - currentPosition));
        SmartDashboard.putNumber("Arm target", target);

    }

    /*
    public void SetArmState(STATE state){
        m_armState = state;
    }*/
    public void SetArmSpeed(double speed){
        armMotor1.set(speed);

    }
    

    public double GetPotValue(){
        return armPot.get();
    }
}