package frc.robot.Autos;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Mechanisms.DriveTrain;

public class LinearTrajectory {
    private double startTime;
    private double timeInit;
    private double currentTime;
    private double elapsedTime;
    private double targetTime;
    private DriveTrain m_DriveTrain;
    private double driveSpeed;

    private final double kEncoderTicksPerRevolution = 2048.0f;
    private final double Wheel_Circumference = (6.0f * Math.PI) / 12.0; // Circumference in Feet
    private final double EncoderTicksPerFoot = kEncoderTicksPerRevolution / Wheel_Circumference;
    private double currentPosTicks; //Measures position in encoder ticks
    private double m_distance;
    private boolean isFinished;
    

    public LinearTrajectory(DriveTrain driveTrain, double speed, double distance, double Timeout){
        //elapsedTime = 0;
        driveSpeed = speed;
        m_DriveTrain = driveTrain;
        m_distance = distance;
        targetTime = Timeout;
    }

    public void init() {
		isFinished = false;
		startTime = Timer.getFPGATimestamp();
	}

    public void run(){
        currentTime = Timer.getFPGATimestamp();
        m_DriveTrain.setMotorSpeeds(-driveSpeed, -driveSpeed);
        currentPosTicks = m_DriveTrain.getAverageEncoder();
        
        if(((currentTime - startTime) > targetTime) || (TickstoFeet(currentPosTicks) > m_distance)){
            isFinished = true;
        }
    }

    private double TickstoFeet(double ticks){
        return ticks / EncoderTicksPerFoot;
    }



    public boolean isDone(){
        return isFinished;
    }
    
}
