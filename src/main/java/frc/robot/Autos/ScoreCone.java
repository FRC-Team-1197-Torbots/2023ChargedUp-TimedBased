package frc.robot.Autos;

import frc.robot.Mechanisms.Arm;
import frc.robot.Mechanisms.Claw;
import frc.robot.Mechanisms.Elevator;
import frc.robot.Mechanisms.Intake;
import frc.robot.Mechanisms.Arm.AutoArmDirection;
import frc.robot.Mechanisms.Claw.autoClawState;
import frc.robot.Mechanisms.Elevator.AutoElevatorDirection;
import frc.robot.Mechanisms.Intake.autoIntakeState;

public class ScoreCone {
    public enum scoreSequence {
        ClawIn, IntakeOut, ElevatorUp, ArmUp, ClawOpen, 
        ArmDown, ElevatorDown, IntakeIn, DONE
    }
    private Intake m_Intake;
    private Elevator m_Elevator;
    private Arm m_Arm;
    private Claw m_Claw;
    private scoreSequence m_Sequence = scoreSequence.ClawIn;
    private boolean isFinished;

    public ScoreCone(Intake intake, Elevator elevator, Arm arm, Claw claw){
        m_Intake = intake;
        m_Elevator = elevator;
        m_Arm = arm;
        m_Claw = claw;
    }

    public void init(){
        isFinished = false;
    }

    public void run(){
        System.out.println(m_Sequence);
        switch(m_Sequence){
            case ClawIn:
                m_Claw.autoRun();
                if(m_Claw.isDone()){
                    System.out.println("Going to INtake");
                    m_Sequence = scoreSequence.IntakeOut;
                    //System.out.println(m_Sequence);
                }
                break;
            case IntakeOut:
                System.out.println("2222222222222222222222");
                m_Intake.setAutoState(autoIntakeState.INIT);
                m_Intake.autoRun();
                System.out.println("2222222222222222222222");
                if(m_Intake.isDone()){
                    m_Sequence = scoreSequence.ElevatorUp;
                }
                break;
            case ElevatorUp:
                m_Elevator.autoRun(AutoElevatorDirection.UP);
                if(m_Elevator.isDone()){
                    m_Sequence = scoreSequence.ArmUp;
                }
                break;
            case ArmUp:
                m_Arm.autoRun(AutoArmDirection.UP);
                if(m_Arm.isDone()){
                    m_Sequence = scoreSequence.ClawOpen;
                }
                break;
            case ClawOpen:
                m_Claw.autoRun();
                if(m_Claw.isDone()){
                    m_Sequence = scoreSequence.ArmDown;
                }
                break;
            case ArmDown:
                m_Arm.autoRun(AutoArmDirection.DOWN);
                if(m_Arm.isDone()){
                    m_Sequence = scoreSequence.ElevatorDown;
                }
                break;
            case ElevatorDown:
                m_Elevator.autoRun(AutoElevatorDirection.DOWN);
                if(m_Elevator.isDone()){
                    m_Sequence = scoreSequence.ArmUp;
                }
                break;
            case IntakeIn:
                m_Intake.autoRun();
                if(m_Intake.isDone()){
                    m_Sequence = scoreSequence.ElevatorUp;
                }
                break;
            case DONE:
                isFinished = true;
                break;            
           }
    }

    public boolean isDone(){
        return isFinished;
    }
    
}
