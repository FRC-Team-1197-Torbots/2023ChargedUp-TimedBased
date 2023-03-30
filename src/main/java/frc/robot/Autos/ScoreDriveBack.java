package frc.robot.Autos;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Mechanisms.Arm;
import frc.robot.Mechanisms.Claw;
import frc.robot.Mechanisms.DriveTrain;
import frc.robot.Mechanisms.Elevator;
import frc.robot.Mechanisms.Intake;
import frc.robot.Mechanisms.Claw.autoClawState;

public class ScoreDriveBack {
    public static enum autoRun1 {
        INIT, Score1, Linear1, DONE
    }
    private LinearTrajectoryTimed linear1;
    private LinearTrajectoryTimed linearDone;
    private ScoreCone score1;
    private double currentTime;
    private autoRun1 autoState = autoRun1.INIT;

    public ScoreDriveBack(DriveTrain driveTrain, Intake intake, Elevator elevator, Arm arm, Claw claw){
        score1 = new ScoreCone(intake, elevator, arm, claw);
        linear1 = new LinearTrajectoryTimed(driveTrain, -0.25, 4.0);
        linearDone = new LinearTrajectoryTimed(driveTrain, 0, 10);

    }
    public void run() {
        currentTime = Timer.getFPGATimestamp();
        switch(autoState) {
            case INIT:
                score1.init();
                System.out.println("PHASE 1");
                autoState = autoRun1.Score1;
                break;
            case Score1:
                System.out.println("PHASE 2");
                score1.run();
                if(score1.isDone()){
                    autoState = autoRun1.Linear1;
                }
            case Linear1:
                linear1.run();
                if(linear1.isDone()) {
                    autoState = autoRun1.DONE;
                }
                break;
            case DONE:
                linearDone.run();
                break;
        }
    
    }
}
