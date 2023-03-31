package frc.robot.Autos;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Mechanisms.DriveTrain;

public class LinearTrajectoryTimed {
    private double driveSpeed;
    private double startTime;
    private double currentTime;
    private double timeDifference;
    private double endTime;
    private boolean isFinished;
    private boolean isDone;
    private DriveTrain m_DriveTrain;
    public LinearTrajectoryTimed(DriveTrain driveTrain, double speed, double time){
        driveSpeed = speed;
        endTime = time;
        m_DriveTrain = driveTrain;
    }

    public void init() {
		isFinished = false;
		startTime = Timer.getFPGATimestamp();
	}

    public void run(){
        currentTime = Timer.getFPGATimestamp();
        timeDifference = currentTime - startTime;
        m_DriveTrain.setMotorSpeeds(-driveSpeed, -driveSpeed);
        
        if(timeDifference > endTime){
            isFinished = true;
        }
    }

    public boolean isDone(){
        return isFinished;
    }
    
}
