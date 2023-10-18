package frc.robot.Autos;

import java.util.PriorityQueue;
import java.util.Queue;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.LimelightHelpers;
import frc.robot.Mechanisms.Arm;
import frc.robot.Mechanisms.Claw;
import frc.robot.Mechanisms.DriveTrain;
import frc.robot.Mechanisms.Elevator;
import frc.robot.Mechanisms.Intake;
import frc.robot.Mechanisms.Arm.RunArm;
import frc.robot.Mechanisms.Claw.autoClawState;
import frc.robot.Mechanisms.Elevator.RunElevator;

public class ScoreHighTaxi {
    public static enum autorun2{
        INIT, ALIGN,SCAN, ELEVATORUP, FORWARD, OPENCLAW, ELEVATORDOWN, BACKUPANDTAXI, DONE
    }
    private LinearTrajectory Forward1;
    private LinearTrajectoryTimed Backwards1;
    private LinearTrajectoryTimed linearDone;
    private double currentTime;
    private Elevator m_Elevator;
    private Arm m_Arm;
    private Claw m_Claw;
    private DriveTrain m_DriveTrain;
    private autorun2 autoState = autorun2.INIT;
    private boolean hasrun;
    private double endtime;
    private NetworkTable table;
    private Queue<Double> area_data;
    private int samples = 10;
    private double goal_distance;
    
    public ScoreHighTaxi(DriveTrain driveTrain, Elevator elevator, Claw claw, Arm arm){
        m_DriveTrain=driveTrain;
        m_Elevator = elevator;
        m_Arm = arm;
        m_Claw = claw;
        Backwards1 = new LinearTrajectoryTimed(driveTrain, -0.35, 2.25);
        linearDone = new LinearTrajectoryTimed(driveTrain, 0, 8);
        hasrun = false;
        endtime=-1;


    }
    public void run(){
        m_Claw.SetClawSpeed(0.25f);
        currentTime = Timer.getFPGATimestamp();
        area_data = new PriorityQueue<Double>();

        m_Arm.SetState(RunArm.IDLE);
        switch(autoState){
            case INIT:
                autoState = autorun2.SCAN;
                break;
            /*case FORWARDTOALIGN:
                if(!hasrun) {
                    Forward1.init();
                    hasrun = true;
                }
                Forward1.run();
                if(Forward1.isDone()){
                    autoState = autorun2.ALIGN;
                    hasrun=false;
                }
                break;*/
            case SCAN:
                double[] botpose = LimelightHelpers.getBotPose_TargetSpace("limelight");
                goal_distance = (-botpose[2]*3.28)-2;
                System.out.println("goal: " + goal_distance);
                autoState=autorun2.FORWARD;
                Forward1 = new LinearTrajectory(m_DriveTrain, 0.1, goal_distance, 4);
                m_DriveTrain.setMotorSpeeds(0, 0);
                Forward1.init();
                break;

            case ALIGN:
                if(endtime==-1){
                    endtime=currentTime+1;
                }
                 double kp = 0.5;
                 double motorspeed=0;
                 double offset = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
                 System.out.println(offset);
                // align to tag
                if(Math.abs(offset)<3){
                    m_DriveTrain.setMotorSpeeds(0, 0);
                }
                else if(offset > 0){
                    motorspeed = offset*kp;
                    m_DriveTrain.setMotorSpeeds(motorspeed, -motorspeed);
                }
                else{
                    m_DriveTrain.setMotorSpeeds(-motorspeed, motorspeed);
                }
                if(currentTime>endtime){
                    autoState= autorun2.FORWARD;
                    hasrun=false;
                    Forward1 = new LinearTrajectory(m_DriveTrain, 0.1, goal_distance, 4);
                    Forward1.init();
                }
                break;

            case FORWARD:
                // System.out.println("Forward state");

                Forward1.run();

                if(Forward1.isDone()){
                    hasrun=false;
                    m_Elevator.SetState(RunElevator.SCORE);
                    m_DriveTrain.setMotorSpeeds(0, 0);
                    endtime = currentTime + 2.5f;
                    autoState = autorun2.ELEVATORUP;
                }
                break;

            case ELEVATORUP:
                System.out.println("Elevator up");
                m_Elevator.run();
                m_Arm.run();

                if(m_Elevator.GetElevatorPos() >= (23108/1.87) && m_Arm.m_state == RunArm.IDLE) {
                    m_Arm.SetState(RunArm.HORIZONTAL);
                }
                if (currentTime>endtime || m_Elevator.isDone()){
                    autoState = autorun2.OPENCLAW;
                }
            break;   
   
            case OPENCLAW:
            System.out.println("open claw");

                
            break;                

        }       
    }
}
