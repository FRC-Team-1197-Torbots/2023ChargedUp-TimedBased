package frc.robot.Autos;

public class LinearTrajectory {
    private double timeInit;
    private double currentTime;
    private double elapsedTime;
    private double targetTime;
    //private DriveTrain driveSubsystem;
    private double driveSpeed;
    private double m_distance;

    public LinearTrajectory(double speed, double distance){
        //elapsedTime = 0;
        driveSpeed = speed;
        //driveSubsystem = driveSub;
        m_distance = distance;
        //targetTime = runTime;
    }
    
}
