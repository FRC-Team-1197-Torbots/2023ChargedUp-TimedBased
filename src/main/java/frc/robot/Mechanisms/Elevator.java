package frc.robot.Mechanisms;

import java.lang.annotation.Target;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.PID_Tools.TorDerivative;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.Trajectory.State;


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
    //private STATE m_Elstate;
    //private TARGET m_Eltarget;
    private RunElevator m_runElevator;
    private double timeInterval = 0.005;
    private double dt = timeInterval;
    private double pidDerivativeResult;
    private double pidIntegral;
    private TorDerivative pidDerivative;
    
    private final double kPTOP = 0.000020;
    private final double kITOP = 0.0;
    private final double kDTOP = 0.0000002;


    private final double kPMID = 0.000017;
    private final double kIMID = 0.000000037;
    private final double kDMID = 0.0000012;

    private boolean fromtop;

    public static enum RunElevator{
        INTAKE,SCORE,IDLE, STORE
    }
    
    public Elevator(){
        
        elMotor1 = new CANSparkMax(7, MotorType.kBrushless);
        elMotor2 = new CANSparkMax(6, MotorType.kBrushless);
        elMotor1.setIdleMode(IdleMode.kBrake);
        elMotor2.setIdleMode(IdleMode.kBrake);
        //botlimitSwitch = new DigitalInput(ElevatorArmConstants.limitSwitch1Port);
        //toplimitSwitch = new DigitalInput(ElevatorArmConstants.limitSwitch2Port);
        elEncoder = new Encoder(4, 5, false, Encoder.EncodingType.k4X);//Input correct channels later
        //elevatorPID = new PIDController(0.0000284, 0, 0.00000001);
        //%ResetEncoder();
        pidDerivative = new TorDerivative(dt);
        pidIntegral = 0;
        
        m_runElevator = RunElevator.IDLE;
    }  

    //function for setting the state
    public void SetState(RunElevator State) {
        if(m_runElevator == RunElevator.SCORE && State == RunElevator.STORE) {
            fromtop = true;
        } else if(m_runElevator == RunElevator.INTAKE && State == RunElevator.STORE) {
            fromtop = false;
        }
        
        m_runElevator = State;


    }

    public void run(){  
        //m_runElevator = ElevatorState;

        switch(m_runElevator){
            case INTAKE:     
                //m_intake.SetSolenoid(true);
                target = 25108;
                break;
            case SCORE:
                target = 35096;
                break;
            case IDLE://comment out if PID doesn't work
                target = 0;
                break;
            case STORE:
                target = 1000;
                break;
        }

        
        currentPosition = GetElevatorPos();
        pidDerivativeResult = pidDerivative.estimate(target - currentPosition);
        
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
                elevatorOutput = (kPMID * (target - currentPosition)) + (kIMID * pidIntegral) + (kDMID * pidDerivativeResult);
                break;

            case SCORE:
                elevatorOutput = (kPTOP * (target - currentPosition)) + (kITOP * pidIntegral) + (kDTOP * pidDerivativeResult);
                break;

            case STORE:
                if(fromtop) {
                    elevatorOutput = ((kPTOP * 0.5) * (target - currentPosition)) + (kITOP * pidIntegral) + (kDTOP * pidDerivativeResult);
                } else {
                    elevatorOutput = ((kPMID * 0.4) * (target - currentPosition)) + (kIMID * pidIntegral) + (kDMID * pidDerivativeResult);
                }
                break;
        }


        SetElevatorSpeed(elevatorOutput);
        //SmartDashboard.putNumber("Output", elevatorOutput);
        //SmartDashboard.putNumber("Current Pos", currentPosition);
        //SmartDashboard.putNumber("Error", target - currentPosition);
        //SmartDashboard.putBoolean("fromtop", fromtop);
        //System.out.println(currentPosition);
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
