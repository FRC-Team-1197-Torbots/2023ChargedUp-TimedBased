package frc.robot.Autos;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Mechanisms.Arm;
import frc.robot.Mechanisms.Arm.RunArm;
import frc.robot.Mechanisms.Claw;
import frc.robot.Mechanisms.DriveTrain;
import frc.robot.Mechanisms.Elevator;
import frc.robot.Mechanisms.Elevator.RunElevator;
import frc.robot.PID_Tools.TorDerivative;
import frc.robot.Mechanisms.Intake;

public class AutoBalance {
    public static enum autoBalance1 {
        INIT, MoveElevatorUp, OuttakeRoller, Linear1, MoveElevatorDown, Linear2, Linear3, DONE, TIMEOUT1, Balance
    }
    private LinearTrajectoryTimed linear1;
    private LinearTrajectory linear3;
    private LinearTrajectoryTimed linear2;
    private LinearTrajectoryTimed linearDone;
    private double currentTime;
    private Elevator m_Elevator;
    private Arm m_Arm;
    private Claw m_Claw;
    private DriveTrain m_DriveTrain;
    private autoBalance1 m_AutoBalance = autoBalance1.INIT;
    private boolean hasrun;
    private double endtime;
    private double balanceOutput;

    //PID variables
    private TorDerivative pidDerivative;
    private double pidIntegral;
    private double pidDerivativeResult;

    private final double dt = 0.005f;
    private final double balancekP = 0.01;
    private final double balancekI = 0;
    private final double balancekD = 0;

    public AutoBalance(DriveTrain driveTrain, Intake intake, Elevator elevator, Arm arm, Claw claw){
        pidIntegral = 0;
        m_Elevator = elevator;
        m_DriveTrain = driveTrain;
        m_Arm = arm;
        m_Claw = claw;
        linear1 = new LinearTrajectoryTimed(driveTrain, 0.1, 1);
        linear2 = new LinearTrajectoryTimed(driveTrain, -0.1, 1);
        linear3 = new LinearTrajectory(driveTrain, -0.35, -6.5, 2.5);
        linearDone = new LinearTrajectoryTimed(driveTrain, 0, 10);
        pidDerivative = new TorDerivative(dt);
        hasrun = false;
        endtime = -1;
    }

    public void run() {
        m_Claw.SetClawSpeed(0.04f);
        //SmartDashboard.putString("Arm State", m_Arm.m_state.toString());
    
        currentTime = Timer.getFPGATimestamp();
        switch(m_AutoBalance) {
            case INIT:                
                m_AutoBalance = autoBalance1.MoveElevatorUp;
                break;
            /* 
            case CloseClaw:
                m_Claw.setAutoClawState(autoClawState.CLOSE);
                m_Claw.dropClaw();
                Timer.delay(2);
                if(m_Claw.getSolenoidValue() == true){
                    autoState = autoRun1.MoveElevatorUp;
                }
                break*/
            case MoveElevatorUp:
                if(endtime == -1) {
                    endtime = currentTime + 2.5f;
                }

                if(!hasrun) {
                    m_Elevator.SetState(RunElevator.SCORE);
                    //m_Arm.SetState(RunArm.HORIZONTAL);
                    hasrun = true;
                }

                if(m_Elevator.GetElevatorPos() >= (23108/1.87) && m_Arm.m_state == RunArm.IDLE) {
                    m_Arm.SetState(RunArm.HORIZONTAL);
                }

                if(m_Elevator.isDone() || currentTime > endtime){
                    //Timer.delay(2);
                    hasrun = false;
                    m_AutoBalance = autoBalance1.TIMEOUT1;
                    endtime = currentTime + 2.0f;
                }
                break;

            case TIMEOUT1:
                if(currentTime > endtime) {
                    m_AutoBalance = autoBalance1.Linear1;
                }
            break;

            case OuttakeRoller:                
                m_Claw.dropClaw();
                Timer.delay(2);
                m_AutoBalance = autoBalance1.Linear2;   
                hasrun = false;             
                break;

            case Linear2:
                if(!hasrun) {
                    linear2.init();
                    hasrun = true;
                }

                linear2.run();
                if(linear2.isDone()) {
                    m_AutoBalance = autoBalance1.MoveElevatorDown;
                    hasrun = false;
                }
                break;

            case Linear1:
                if(!hasrun) {
                    linear1.init();
                    hasrun = true;
                }

                linear1.run();
                if(linear1.isDone()) {
                    m_AutoBalance = autoBalance1.OuttakeRoller;
                    hasrun = false;
                }
                break;   

            case MoveElevatorDown:
                if(!hasrun) {
                    m_Elevator.SetState(RunElevator.STORE);
                    m_Arm.SetState(RunArm.STORE);
                    hasrun = true;
                    endtime = currentTime + 2.0f;
                }  

                if(m_Arm.isDone() || currentTime > endtime){
                    m_AutoBalance = m_AutoBalance.Linear3;
                    hasrun = false;
                }
                break;

            case Linear3:
                if(!hasrun) {
                    linear3.init();
                    hasrun = true;
                }

                linear3.run();
                if(linear3.isDone()) {
                    m_AutoBalance = m_AutoBalance.DONE; //m_AutoBalance.Balance
                }
                break;
                
            case Balance:
                double error = m_DriveTrain.getPitch();
                pidIntegral += error;
                pidDerivativeResult = pidDerivative.estimate(error);
                balanceOutput = CalculateBalancePID(error);
                m_DriveTrain.setMotorSpeeds(balanceOutput, balanceOutput);
                break;

            case DONE:
                linearDone.run();
                break;
        }    
    }

    // Ideal Max amount of error is 15 degrees for the pitch
    public double CalculateBalancePID(double error){
        return (error * balancekP) + (pidIntegral * balancekI) + (pidDerivativeResult * balancekD);

    }
}
    

