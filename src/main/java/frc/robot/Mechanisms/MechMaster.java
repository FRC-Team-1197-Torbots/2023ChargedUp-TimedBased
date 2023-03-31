package frc.robot.Mechanisms;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Mechanisms.Arm.RunArm;
import frc.robot.Mechanisms.Claw.RunClaw;
import frc.robot.Mechanisms.Elevator.RunElevator;
import frc.robot.Mechanisms.LED.setLED;

public class MechMaster {
    
    private Arm m_arm;
    private Claw m_claw;
    private DriveTrain m_driveTrain;
    private Elevator m_elevator;
    private Intake m_intake;
    private double armThrottle;
    private double armOutput;
    private LED m_led;

    private XboxController m_player1;
    private XboxController m_player2;
    private double ConeIntakeSpeed = 0.7;
    private double CubeIntakeSpeed = 0.5;
    private double CubeEjectSpeed = -0.7;
    private boolean claw = false;
    private RunClaw clawsolenoid;

    
    public MechMaster(XboxController player1, XboxController player2, Elevator elevator, Arm arm, Claw claw, Intake intake, LED led){
        m_player1 = player1;
        m_player2 = player2;
        m_elevator = elevator;
        m_arm = arm;
        m_claw = claw;
        m_intake = intake;
        m_led = led;
        
    }



    public void mechRun(){

        /** Need to integrate into elevator code, but arm code is working */
        m_arm.run();
        
        if(m_player2.getAButton()){
            
            m_arm.SetState(RunArm.STORE);
           
        }
        if(m_player2.getBButton()){
            m_arm.SetState(RunArm.HORIZONTAL);
            
        }
        if(m_player2.getYButton()){
           
            m_arm.SetState(RunArm.HORIZONTAL);
            
        }
        if(m_arm.GetPotValue() > m_arm.STOREDBL && m_arm.GetPotValue() < m_arm.SCOREDBL && m_arm.getArmVoltage() < 0.01){
            armThrottle = -m_player2.getLeftY();
            armOutput = armThrottle * m_arm.getMaxOutput();
            double sign = Math.signum(armOutput);
            if(Math.abs(armOutput) > m_arm.getMaxOutput()){
                armOutput = sign * m_arm.getMaxOutput();
            }
        }
    

        
        SmartDashboard.putBoolean("Claw Open/Close", m_claw.getSolenoidValue());

        if(m_player1.getAButtonReleased()){
            m_intake.TriggerIntake();
        }
        
        if(m_player1.getRightBumper()){
            m_claw.SetClawSpeed(ConeIntakeSpeed);
        }
        else if(m_player1.getRightTriggerAxis()>0.2){
            m_claw.SetClawSpeed(CubeEjectSpeed);
        }
         else {
            m_claw.SetClawSpeed(0.07f);
        }

       /*  if(m_player1.getRightTriggerAxis()>0.2){
            m_claw.SetClawSpeed(CubeEjectSpeed);
        } else if(m_player1.getRightTriggerAxis()< 0.2) {
            m_claw.SetClawSpeed(0);
        }*/

        if(m_player1.getBButtonPressed()){
            m_claw.TriggerSolenoid();
        }

        /*MANUAL CONTROL FOR ARM */
        
        
        


        /* 
        if(m_player2.getAButton()){
            
            m_elevator.SetState(RunElevator.STORE);
           m_arm.SetState(RunArm.STORE);
        }
        if(m_player2.getBButton()){
            m_elevator.SetState(RunElevator.INTAKE);
            
        }
        if(m_player2.getYButton()){
           
            m_elevator.SetState(RunElevator.SCORE);
            
        }
        */

        if(( (m_arm.m_state == RunArm.STORE || m_arm.m_state == RunArm.IDLE) 
            && m_elevator.GetElevatorPos() >= (23108/2)) && 
            (m_elevator.m_runElevator == RunElevator.SCORE || m_elevator.m_runElevator == RunElevator.INTAKE)) {
                m_arm.SetState(RunArm.HORIZONTAL);
            }

        m_elevator.run();
         
        if(m_player2.getLeftBumperPressed()){
            m_led.run(setLED.CONE);
        }
        else if(m_player2.getRightBumperPressed()){
            m_led.run(setLED.CUBE);
        }
        else{
            m_led.run(setLED.NONE);
        }         
    }
}