// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Autos.AutoBalance;
import frc.robot.Autos.LinearTrajectory;
import frc.robot.Autos.LinearTrajectoryTimed;
import frc.robot.Autos.ScoreConeDriveBack;
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

  private ScoreConeDriveBack m_ScoreDriveBack;
  private AutoBalance m_AutoBalance;
  

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
    intake = new Intake();
    arm = new Arm();
    claw = new Claw();
    led = new LED();
    elevator = new Elevator(arm);
    mechMaster = new MechMaster(player1, player2, elevator, arm, claw, intake, led);
    driveTrain = new DriveTrain(player1, player2, elevator);
    m_ScoreDriveBack = new ScoreConeDriveBack(driveTrain, intake, elevator, arm, claw);
    m_AutoBalance = new AutoBalance(driveTrain, intake, elevator, arm, claw);
    elevator.ResetEncoder();
    
  
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
  public void teleopInit() {}

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
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
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
