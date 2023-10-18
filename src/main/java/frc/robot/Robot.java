// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.List;
import java.util.PriorityQueue;
import java.util.Queue;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CameraServerCvJNI;
import edu.wpi.first.cscore.VideoSource;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Autos.AutoBalance;
import frc.robot.Autos.LinearTrajectory;
import frc.robot.Autos.ScoreConeDriveBack;
import frc.robot.Autos.ScoreHighTaxi;
import frc.robot.LimelightHelpers.LimelightResults;
import frc.robot.Mechanisms.Arm;
import frc.robot.Mechanisms.Claw;
import frc.robot.Mechanisms.DriveTrain;
import frc.robot.Mechanisms.Elevator;
import frc.robot.Mechanisms.Intake;
import frc.robot.Mechanisms.LED;
import frc.robot.Mechanisms.MechMaster;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Do Nothing";
  private static final String kAuto1 = "Score then Drive Back";
  private static final String kAuto2 = "Auto Balance";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  private XboxController player1;
  private XboxController player2;
  private DriveTrain driveTrain;
  private MechMaster mechMaster;
  private Elevator elevator;
  private Arm arm;
  private Claw claw;
  private Intake intake;
  private LED led;
  private NetworkTable table;

  private ScoreHighTaxi m_ScoreHighTaxi;
  private ScoreConeDriveBack m_ScoreDriveBack;
  private AutoBalance m_AutoBalance;
  //private CameraServer cameraServer;
  private double[] data;
  private Queue<Double> area_data;
  private int goalmet_counter;
  private int counter;
  private int samples = 10;

  private LinearTrajectory Linear1;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {

    m_chooser.setDefaultOption("Do Nothing", kDefaultAuto);
    m_chooser.addOption("Score and Drive Back", kAuto1);
    m_chooser.addOption("Auto Balance", kAuto2);
    SmartDashboard.putData("Auto choices", m_chooser);
    table = NetworkTableInstance.getDefault().getTable("limelight");
    player1 = new XboxController(0);
    player2 = new XboxController(1);
    //intake = new Intake();
    arm = new Arm();
    claw = new Claw();
    led = new LED();
    elevator = new Elevator(arm);
    mechMaster = new MechMaster(player1, player2, elevator, arm, claw, led);
    driveTrain = new DriveTrain(player1, player2, elevator);
    m_ScoreDriveBack = new ScoreConeDriveBack(driveTrain, intake, elevator, arm, claw);
    m_ScoreHighTaxi = new ScoreHighTaxi(driveTrain, elevator, claw, arm);
    m_AutoBalance = new AutoBalance(driveTrain, intake, elevator, arm, claw);
    elevator.ResetEncoder();
    //driveTrain.setBrakeMode();
    //claw.TriggerSolenoid();
    
    //CameraServer.addServer("http://10.11.97.210:5800", 0);
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {}

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    //driveTrain.setBrakeMode();
    elevator.ResetEncoder();
    //claw.TriggerSolenoid();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    //System.out.println("Auto selected: " + m_autoSelected);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kDefaultAuto:
        driveTrain.setMotorSpeeds(0, 0);
        break;
        
      case kAuto1:
        m_ScoreDriveBack.run();
        arm.run();
        elevator.run();

        break;
      case kAuto2:
        m_AutoBalance.run();
        arm.run();
        elevator.run();
        
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    SmartDashboard.putNumber("Potentiometer Value", arm.GetPotValue());
    mechMaster.mechRun();
    driveTrain.TeleDrive();
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
    driveTrain.setMotorState(IdleMode.kCoast);
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
    data = new double[100];
    counter = 0;

    area_data = new PriorityQueue<Double>();

    Linear1 = new LinearTrajectory(driveTrain, 0.1, 2, 100);
    Linear1.init();
    driveTrain.setBrakeMode();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    m_ScoreHighTaxi.run();
    
     
     /*if(!Linear1.isDone()){
      Linear1.run();
     }else{
      driveTrain.setMotorSpeeds(0, 0);
     }*/
     
     
    // //SmartDashboard.putNumber("Pot Value", arm.GetPotValue());
    //System.out.println("Pot value" + arm.GetPotValue());
    // System.out.println("Elevator value " + elevator.GetElevatorPos());
    // double kp = 0.5;
    // double motorspeed=0;
    // double offset = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
    // System.out.println(offset);
    // // align to pole
    // if(Math.abs(offset)<3){
    //   driveTrain.setMotorSpeeds(0, 0);
    // }
    // else if(offset > 0){
    //   motorspeed = offset*kp;
    //   driveTrain.setMotorSpeeds(-motorspeed, motorspeed);
    // }
    // else{
    //   driveTrain.setMotorSpeeds(motorspeed, -motorspeed);
    // }

    // // distance calculation for pole
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    // NetworkTableEntry tx = table.getEntry("tx");
    // double targetOffsetAngle_Vertical = -tx.getDouble(0.0);

    // // // how many degrees back is your limelight rotated from perfectly vertical?
    // double limelightMountAngleDegrees = 0.0;

    // // // distance from the center of the Limelight lens to the floor
    // double limelightLensHeightInches = 22.75;

    // // // distance from the target to the floor
    // double goalHeightInches = 47.75;

    // double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
    // double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

    // // //calculate distance
    // double distance = (goalHeightInches - limelightLensHeightInches)/Math.tan(angleToGoalRadians);
    // System.out.println(distance);

    //25in = 0.71% area
    //28.5in = 0.46% area
    //45in = 0.26% area
    //double[] botpose = LimelightHelpers.getBotPose_TargetSpace("limelight");
    //System.out.println("distance: " + -botpose[2]);


    double goalArea = 0.3338977572321892;//0.23;//0.4;//0.630587891638279;

    
    // System.out.println(poleArea);

    
    // if(poleArea >= goalArea){
    //   System.out.println("goal met");
    // }
    // else{
    //   System.out.println("not close enough");
    // }
    // if(goalmet_counter < 20){
    // }

    /*if(area_data.size() < samples) {
      area_data.add(poleArea);
    } else {
      area_data.remove();
      area_data.add(poleArea);
    }

    double sum = 0;

    for(Double d : area_data) {
      sum += d;
      //System.out.println(d);
    }

    double area_avg = sum/area_data.size();
    double goal_area = 0;*/

    // if(area_avg > goal_area){
    //   //stop
    // }
    // else{
    //   //move fwd
    //   //double drive_spd = 0.2;
    //   //driveTrain.setMotorSpeeds(drive_spd , drive_spd);
    // }



    
    //System.out.println("area average: " + area_avg + "   " + area_data.size() + "   " + sum);
    //System.out.println(area_data.peek());
    //socal avg: 0.6930251628160476


    // average: 0.630587891638279%

      // if(counter < 100) {
      //   data[counter] = poleArea;
      //   counter++;
      // } else {
      //   double sum = 0;
      //   for(int i = 0; i < data.length; i++) {
      //     sum += data[i];
      //   }

      //   System.out.println("Average is " + sum/data.length);
      // }



  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
