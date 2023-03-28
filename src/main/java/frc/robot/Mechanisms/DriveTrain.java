package frc.robot.Mechanisms;

import com.ctre.phoenix.sensors.Pigeon2;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.math.controller.PIDController;
public class DriveTrain {
    private CANSparkMax LeftTop;
    private CANSparkMax LeftBottom1;
    private CANSparkMax LeftBottom2;

    private CANSparkMax RightTop;
    private CANSparkMax RightBottom1;
    private CANSparkMax RightBottom2;
    private Pigeon2 pigeon;

    private Encoder leftEncoder;
    private Encoder rightEncoder;
    private XboxController m_player1;
    private XboxController m_player2;

    private double maxThrottle, maxSteer;

    private boolean low;

    public DriveTrain(XboxController player1, XboxController player2) {
        //navx_sim = new SimDevice(dev);
        m_player1 = player1;
        m_player2 = player2;

        LeftTop = new CANSparkMax(3, MotorType.kBrushless);
        LeftBottom1 = new CANSparkMax(4, MotorType.kBrushless);
        LeftBottom2 = new CANSparkMax(5, MotorType.kBrushless); 
    
        RightTop = new CANSparkMax(13, MotorType.kBrushless); 
        RightBottom1 = new CANSparkMax(12, MotorType.kBrushless);		
        RightBottom2 = new CANSparkMax(2, MotorType.kBrushless);
        
        
        LeftTop.setIdleMode(IdleMode.kBrake);
        LeftBottom1.setIdleMode(IdleMode.kBrake);
        LeftBottom2.setIdleMode(IdleMode.kBrake);

        RightTop.setIdleMode(IdleMode.kBrake);
        RightBottom1.setIdleMode(IdleMode.kBrake);
        RightBottom2.setIdleMode(IdleMode.kBrake);

        pigeon = new Pigeon2(1);
    
        //pidDrive = new PIDController(TeleopDriveConstants.velocitykP, TeleopDriveConstants.velocitykI, TeleopDriveConstants.velocitykD);
        //m_gyro = new AnalogGyro(0);
    
        //driveSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 0);
    
        leftEncoder = new Encoder(8, 9, false, Encoder.EncodingType.k4X);
        rightEncoder = new Encoder(0, 1, false, Encoder.EncodingType.k4X);
        
        resetEncoder();
        resetGyro();

        pigeon.configFactoryDefault();
        pigeon.setYaw(0);

        setMotorState(IdleMode.kBrake);
        //gyroSim = new AnalogGyroSim(m_gyro);
        
      }
    
      public void Drive(){
        double throttle = m_player1.getLeftY();
        double steer = m_player1.getLeftX();

        /*if(m_player1.getRawButtonReleased(1)) {
            low = !low;
        }*/
      
          //SmartDashboard.putBoolean("low", low);
      
        if(!low){
          maxThrottle = 0.9;
          maxSteer = 0.9;
        } else if(low){
          maxThrottle = 0.5;
          maxSteer = 0.5;
        }
        
        double sign = Math.signum(throttle);
        throttle = sign * Math.pow(throttle, 2) * maxThrottle;
        sign = Math.signum(steer);
        steer = sign * Math.pow(steer, 2) * maxSteer;
      
        //steer *= -1;
        
        if(Math.abs(throttle) < 0.025f) {
          throttle = 0;
        }
      
        if(Math.abs(steer) < 0.035f) {
         steer = 0;
        }
      
        double leftSpeed, rightspeed;
      
        if(throttle > 0) {
          if(steer > 0) {
              leftSpeed = throttle - steer;
              rightspeed = Math.max(throttle, steer);
          } else {
              leftSpeed = Math.max(throttle, -steer);
              rightspeed = throttle + steer;
          }
        } else {
            if(steer > 0) {
                leftSpeed = -Math.max(-throttle, steer);
                rightspeed = throttle + steer;
            } else {
                leftSpeed = throttle - steer;
                rightspeed = -Math.max(-throttle, -steer);
            }
        }
      
    
        setMotorSpeeds(leftSpeed, rightspeed);
        
      }
    
      public void setMotorState(IdleMode mode){
        LeftTop.setIdleMode(mode);
        LeftBottom1.setIdleMode(mode);
        LeftBottom2.setIdleMode(mode);
        RightTop.setIdleMode(mode);
        RightBottom1.setIdleMode(mode);
        RightBottom2.setIdleMode(mode);
      }
    
      public void setMotorSpeeds(double leftOutput, double rightOutput){
        SetLeft(leftOutput);
        SetRight(rightOutput);
      }
      
      public double getRightVelocity() {
        return rightEncoder.getRate();
      }
      
      public double getLeftVelocity() {
        return leftEncoder.getRate();
      }
      
      public double getLeftEncoder() {
        return leftEncoder.getRaw();
      }
      
      public double getRightEncoder() {
        return rightEncoder.getRaw();
      }
    
      public double getAverageEncoder(){
        return ((getLeftEncoder() + getRightEncoder()) / 2.0);
      }
      
      // Method to reset the encoder values
        public void resetEncoder() {
            leftEncoder.reset();
            rightEncoder.reset();
        }
    
        // Method to reset the spartan board gyro values
        public void resetGyro() {
            pigeon.setYaw(0);
        }
    
      public void SetLeft(double speed) {
            LeftTop.set(speed);
            LeftBottom1.set(speed);
            LeftBottom2.set(speed);
        }
    
        // Setting the right master Talon's speed to the given parameter
        public void SetRight(double speed) {
            RightTop.set(-speed); 
            RightBottom1.set(-speed);
            RightBottom2.set(-speed);
        }    
}
