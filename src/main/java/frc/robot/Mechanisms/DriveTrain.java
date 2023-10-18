package frc.robot.Mechanisms;

import com.ctre.phoenix.sensors.Pigeon2;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.PID_Tools.TorDerivative;
import edu.wpi.first.math.controller.PIDController;
public class DriveTrain {
    private CANSparkMax LeftTop;
    private CANSparkMax LeftBottom1;
    private CANSparkMax LeftBottom2;
//kjnjfvnfivnlivzfsivzivhzignsz
    private CANSparkMax RightTop;
    private CANSparkMax RightBottom1;
    private CANSparkMax RightBottom2;
    private Pigeon2 pigeon;

    private Encoder leftEncoder;
    private Encoder rightEncoder;
    private XboxController m_player1;
    private XboxController m_player2;
    private double lefttargetSpeed, righttargetSpeed;
    private double currentSpeed;
    private double currentError;
    
    private double velocitykP = 0.000066;
    private double velocitykI = 0;
    private double velocitykD = 0;
    private TorDerivative pidDerivative;
    private double pidDerivativeResult;
    private double LeftPIDSum;
    private double RightPIDSum;
    private double pidIntegral;
    private double timeInterval = 0.005;
    private double dt = timeInterval;

    public static enum SIDE {
      LEFT, RIGHT
    }

    private double leftOutput;
    private double rightOutput;

    private final double MAX_VELOCITY = 15000f;

    private Elevator m_Elevator;

    private double maxThrottle, maxSteer;

    private boolean low;

    public DriveTrain(XboxController player1, XboxController player2, Elevator elevator) {
        //navx_sim = new SimDevice(dev);
        m_player1 = player1;
        m_player2 = player2;

        LeftTop = new CANSparkMax(3, MotorType.kBrushless);
        LeftBottom1 = new CANSparkMax(4, MotorType.kBrushless);
        LeftBottom2 = new CANSparkMax(5, MotorType.kBrushless); 
    
        RightTop = new CANSparkMax(13, MotorType.kBrushless); 
        RightBottom1 = new CANSparkMax(12, MotorType.kBrushless);		
        RightBottom2 = new CANSparkMax(2, MotorType.kBrushless);
        
        pidDerivative = new TorDerivative(dt);
        

        pigeon = new Pigeon2(1);

        m_Elevator = elevator;
    
        //pidDrive = new PIDController(TeleopDriveConstants.velocitykP, TeleopDriveConstants.velocitykI, TeleopDriveConstants.velocitykD);
        //m_gyro = new AnalogGyro(0);
    
        //driveSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 0);
    
        leftEncoder = new Encoder(8, 9, false, Encoder.EncodingType.k4X);
        rightEncoder = new Encoder(0, 1, false, Encoder.EncodingType.k4X);
        
        resetEncoder();
        resetGyro();

        pigeon.configFactoryDefault();
        pigeon.setYaw(0);

        setBrakeMode();
        //gyroSim = new AnalogGyroSim(m_gyro);
        
      }
    
      public void TeleDrive(){
        double throttle = m_player1.getLeftY();
        double steer = m_player1.getLeftX();
        //System.out.println(getRightEncoder());

        /*if(m_player1.getRawButtonReleased(1)) {
            low = !low;
        }*/
      
          //SmartDashboard.putBoolean("low", low);
      
        
        if(m_Elevator.GetElevatorPos() > 5000){
          maxThrottle = 0.35;
          maxSteer = 0.35;
        }
        else if(!low){
          maxThrottle = 0.9;//0.8
          maxSteer = 0.55;
        } 
        else{
          maxThrottle = 0.9;//0.8
      
          maxSteer = 0.55;
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


        //leftOutput = PID(getLeftVelocity(), leftSpeed * MAX_VELOCITY, SIDE.LEFT);
        //rightOutput = PID(getRightVelocity(), rightspeed * MAX_VELOCITY, SIDE.RIGHT);
        /* 
        if (Math.abs(leftOutput) < 0.01 && Math.abs(rightOutput) < 0.01) 
            setMotorSpeeds(0, 0);
       else 
            setMotorSpeeds(leftOutput, rightOutput);
          */
          setMotorSpeeds(leftSpeed, rightspeed);

        //lefttargetSpeed = leftSpeed * MAX_VELOCITY;
        //righttargetSpeed = rightspeed * MAX_VELOCITY;

      
    
        //setMotorSpeeds(leftOutput, rightOutput);
        
      }

      public double getPitch(){
        return pigeon.getPitch();
      }

      public void setBrakeMode(){
        LeftTop.setIdleMode(IdleMode.kBrake);
        LeftBottom1.setIdleMode(IdleMode.kBrake);
        LeftBottom2.setIdleMode(IdleMode.kBrake);

        RightTop.setIdleMode(IdleMode.kBrake);
        RightBottom1.setIdleMode(IdleMode.kBrake);
        RightBottom2.setIdleMode(IdleMode.kBrake);
      }

      public double PID(double currentSpeed, double targetSpeed, SIDE side) {
        
        currentError = targetSpeed - currentSpeed;

        // SmartDashboard.putNumber("currentError:", currentError);
        pidDerivativeResult = pidDerivative.estimate(currentError);
        pidIntegral += currentError;

        if(currentError < 20) {
        pidIntegral = 0;
        }

        if(pidIntegral * velocitykI > 0.5) {
        pidIntegral = 0.5 / velocitykI;
        } else if(pidIntegral * velocitykI < -0.5) {
        pidIntegral = -0.5 / velocitykI;
        }

        if(side == SIDE.LEFT) {
            LeftPIDSum = ((currentError * velocitykP) +
            (pidIntegral * velocitykI) +
            (pidDerivativeResult * velocitykD)); //+ FeedForward;
    
            return LeftPIDSum;
        } else if(side == SIDE.RIGHT) {
            RightPIDSum = ((currentError * velocitykP) +
            (pidIntegral * velocitykI) +
            (pidDerivativeResult * velocitykD)); //+ FeedForward;
    
            return RightPIDSum;
        } else {
            return 0;            
        }


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

      public void setHalfSpeed(){}
      
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
