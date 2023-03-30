package frc.robot.Mechanisms;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Mechanisms.Arm.RunArm;
import frc.robot.PID_Tools.TorDerivative;


public class Elevator {
    private CANSparkMax elMotor1;
    private CANSparkMax elMotor2;
    private DigitalInput botlimitSwitch;
    private DigitalInput toplimitSwitch;
    private Encoder elEncoder;    
    private PIDController elevatorPID;
    private double target;
    private int currentPosition;
    private double elevatorOutput;
    private boolean OnTarget = false;

    private boolean isFinished;
    //private STATE m_Elstate;
    //private TARGET m_Eltarget;
    private RunElevator m_runElevator;
    private AutoElevatorDirection m_ElevatorDirection;
    private double timeInterval = 0.005;
    private double dt = timeInterval;
    private double pidDerivativeResult;
    private double pidIntegral;
    private TorDerivative pidDerivative;
    private double speed;
    private Arm ARM_REF;
    
    private final double kPTOP = 0.0000270;
    private final double kITOP = 0.0;
    private final double kDTOP = 0.000000015;


   // private final double kPMID = 0.000017;
    //private final double kIMID = 0.000000037;
   // private final double kDMID = 0.0000012;

    private boolean armmoved;

    public static enum RunElevator{
        INTAKE,SCORE,IDLE, STORE
    }

    public static enum AutoElevatorDirection{
        UP, DOWN
    }
    
    public Elevator(Arm arm){
        
        elMotor1 = new CANSparkMax(7, MotorType.kBrushless);
        elMotor2 = new CANSparkMax(6, MotorType.kBrushless);
        elMotor1.setIdleMode(IdleMode.kCoast);
        elMotor2.setIdleMode(IdleMode.kCoast);
        //botlimitSwitch = new DigitalInput(ElevatorArmConstants.limitSwitch1Port);
        //toplimitSwitch = new DigitalInput(ElevatorArmConstants.limitSwitch2Port);
        elEncoder = new Encoder(4, 5, false, Encoder.EncodingType.k4X);//Input correct channels later
        //elevatorPID = new PIDController(0.0000284, 0, 0.00000001);
        //%ResetEncoder();
        pidDerivative = new TorDerivative(dt);
        pidIntegral = 0;

        //status variables        
        isFinished = false;
        armmoved = false;
        m_runElevator = RunElevator.IDLE;
        elEncoder.reset();
        

        ARM_REF = arm;
    }  

    //function for setting the state
    public void SetState(RunElevator State) {
        /*if(m_runElevator == RunElevator.SCORE && State == RunElevator.STORE) {
            fromtop = true;
        } else if(m_runElevator == RunElevator.INTAKE && State == RunElevator.STORE) {
            fromtop = false;
        }*/
        
        m_runElevator = State;
        armmoved = false;
        OnTarget = false;
    }

    public void run(){  
        //currentPosition = GetElevatorPos();
        //m_runElevator = ElevatorState;
        // if(!OnTarget) {
        //     currentPosition = GetElevatorPos();
        //     if(currentPosition < target && target > 30000) {
        //         speed = 0.24;
        //     } if(currentPosition < target && target > 20000 && target < 30000){
        //         speed = 0.17;
        //     }else if (target < 1000){
        //         speed = -0.24;
        //     }
            

        
        /* 
    
            switch(m_runElevator){
                case INTAKE:     
                    //m_intake.SetSolenoid(true);
                    target = 22108;
                    if(!armmoved && currentPosition > (25108/2.5)) {
                        ARM_REF.SetState(RunArm.HORIZONTAL);
                        armmoved = true;
                    }
                    break;
                case SCORE:
                    target = 35096;

                    if(!armmoved && currentPosition > (35096/3)) {
                        ARM_REF.SetState(RunArm.HORIZONTAL);
                        armmoved = true;
                    } 
                    break;
                case IDLE://comment out if PID doesn't work
                    target = 0;
                    break;
                case STORE:
                    target = 250;
                    if(!armmoved) {
                        ARM_REF.SetState(RunArm.STORE);
                        armmoved = true;
                    }
                    
                    break;
            }
        

        if(Math.abs(target - currentPosition) < 600) {
            OnTarget = true;
            speed = 0;
        }
        */
        
        
        SmartDashboard.putNumber("Output", elevatorOutput);
        SmartDashboard.putNumber("Current Pos", currentPosition);
        //SmartDashboard.putNumber("Error", target - currentPosition);
        SmartDashboard.putBoolean("ontarget", OnTarget);
        //System.out.println(currentPosition);


        
        currentPosition = GetElevatorPos();
        
        pidDerivativeResult = pidDerivative.estimate(target - currentPosition);
        SetElevatorSpeed(elevatorOutput);
        
        if(Math.abs(target - currentPosition) > 1000 || (currentPosition < 1000 && m_runElevator == RunElevator.STORE)) {
            pidIntegral = 0;
        } else {
            pidIntegral += (target - currentPosition);
        }

        //elevatorOutput = elevatorPID.calculate(target - currentPosition);

        switch(m_runElevator){
            case IDLE:

            break;

            case INTAKE:
                target = 23108;
                elevatorOutput = (kPTOP * (target - currentPosition)) + (kITOP * pidIntegral) + (kDTOP * pidDerivativeResult);
                break;

            case SCORE:
                target = 35096;
                elevatorOutput = (kPTOP * (target - currentPosition)) + (kITOP * pidIntegral) + (kDTOP * pidDerivativeResult);
                break;

            case STORE:
                target = 5000;
                elevatorOutput = ((kPTOP * 0.5) * (target - currentPosition)) + (kITOP * pidIntegral) + (kDTOP * pidDerivativeResult);
                
        }
        
        


        
    }

    public void autoRun(AutoElevatorDirection elDirection){

        m_ElevatorDirection = elDirection;
        currentPosition = GetElevatorPos();
        pidDerivativeResult = pidDerivative.estimate(target - currentPosition);
        if(Math.abs(target - currentPosition) > 1000 || (currentPosition < 1000 && m_runElevator == RunElevator.STORE)) {
            pidIntegral = 0;
        } else {
            pidIntegral += (target - currentPosition);
        }
        switch(m_ElevatorDirection){
            case UP:
                target = 35096;
                elevatorOutput = (kPTOP * (target - currentPosition)) + (kITOP * pidIntegral) + (kDTOP * pidDerivativeResult);
                break;
            case DOWN:
                target = 1000;
                elevatorOutput = ((kPTOP * 0.5) * (target - currentPosition)) + (kITOP * pidIntegral) + (kDTOP * pidDerivativeResult);
                break;
        }
        SetElevatorSpeed(elevatorOutput);
        if(Math.abs(target - currentPosition) < 600){
            isFinished = true;
            elevatorOutput = 0;
        }
        else{
            isFinished = false;
        }

    }

    public boolean isDone(){
        return isFinished;
    }

    public void SetElevatorSpeed(double speed){
        elMotor1.set(speed);
        elMotor2.set(speed);

    }
    public int GetElevatorPos(){
        return elEncoder.getRaw();
    }
    public double GetEncoderRate(){
        return elEncoder.getRate();
    }
    public void ResetEncoder(){
        elEncoder.reset();
    }
}
