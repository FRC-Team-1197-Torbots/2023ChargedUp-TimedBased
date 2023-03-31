package frc.robot.Autos;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Mechanisms.Arm;
import frc.robot.Mechanisms.Claw;
import frc.robot.Mechanisms.DriveTrain;
import frc.robot.Mechanisms.Elevator;
import frc.robot.Mechanisms.Intake;
import frc.robot.Mechanisms.Arm.RunArm;
import frc.robot.Mechanisms.Claw.autoClawState;
import frc.robot.Mechanisms.Elevator.RunElevator;


public class ScoreConeDriveBack {
    public static enum autoRun1 {
        INIT, CloseClaw, MoveElevatorUp, OpenClaw, Linear1, MoveElevatorDown, Linear2, Linear3, DONE, TIMEOUT1
    }
    private LinearTrajectoryTimed linear1;
    private LinearTrajectoryTimed linear3;
    private LinearTrajectoryTimed linear2;
    private LinearTrajectoryTimed linearDone;
    private double currentTime;
    private Elevator m_Elevator;
    private Arm m_Arm;
    private Claw m_Claw;
    private autoRun1 autoState = autoRun1.INIT;
    private boolean hasrun;
    private double endtime;

    public ScoreConeDriveBack(DriveTrain driveTrain, Intake intake, Elevator elevator, Arm arm, Claw claw){
        m_Elevator = elevator;
        m_Arm = arm;
        m_Claw = claw;
        linear1 = new LinearTrajectoryTimed(driveTrain, 0.1, 1);
        linear2 = new LinearTrajectoryTimed(driveTrain, -0.1, 1);
        linear3 = new LinearTrajectoryTimed(driveTrain, -0.35, 2);
        linearDone = new LinearTrajectoryTimed(driveTrain, 0, 10);
        hasrun = false;
        endtime = -1;
    }

    public void run() {
        m_Claw.SetClawSpeed(0.04f);
        //SmartDashboard.putString("Arm State", m_Arm.m_state.toString());
    
        currentTime = Timer.getFPGATimestamp();
        switch(autoState) {
            case INIT:                
                autoState = autoRun1.MoveElevatorUp;
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
                    autoState = autoRun1.TIMEOUT1;
                    endtime = currentTime + 2.0f;
                }
                break;

            case TIMEOUT1:
                if(currentTime > endtime) {
                    autoState = autoRun1.Linear1;
                }
            break;

            case OpenClaw:                
                m_Claw.dropClaw();
                Timer.delay(2);
                autoState = autoRun1.Linear2;   
                hasrun = false;             
                break;

            case Linear2:
                if(!hasrun) {
                    linear2.init();
                    hasrun = true;
                }

                linear2.run();
                if(linear2.isDone()) {
                    autoState = autoRun1.MoveElevatorDown;
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
                    autoState = autoRun1.OpenClaw;
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
                    autoState = autoRun1.Linear3;
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
                    autoState = autoRun1.DONE;
                }
                break;

            case DONE:
                linearDone.run();
                break;
        }    
    }
}
