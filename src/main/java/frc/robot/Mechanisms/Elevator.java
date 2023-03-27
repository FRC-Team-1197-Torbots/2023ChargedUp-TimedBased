package frc.robot.Mechanisms;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;

public class Elevator {
    private CANSparkMax elMotor1;
    private CANSparkMax elMotor2;
    private DigitalInput botlimitSwitch;
    private DigitalInput toplimitSwitch;
    private Encoder elEncoder;
    //private STATE m_Elstate;
    //private TARGET m_Eltarget;
    
    public Elevator(){
        elMotor1 = new CANSparkMax(7, MotorType.kBrushless);
        elMotor2 = new CANSparkMax(6, MotorType.kBrushless);
        //botlimitSwitch = new DigitalInput(ElevatorArmConstants.limitSwitch1Port);
        //toplimitSwitch = new DigitalInput(ElevatorArmConstants.limitSwitch2Port);
        elEncoder = new Encoder(4, 5, false, Encoder.EncodingType.k4X);//Input correct channels later
        //ResetEncoder();
        
    }
    
    switch(m_IntakeorScore){
        case INTAKE:
            m_intake.SetSolenoid(true);
            target = 5083;
            break;
        case SCORE:
            m_intake.SetSolenoid(true);
            target = 9400;
            break;
        case IDLE://comment out if PID doesn't work
            m_intake.SetSolenoid(true);
            target = 0;
            break;
    }

    
}
