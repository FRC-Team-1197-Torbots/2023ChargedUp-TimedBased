import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.AnalogInput;
import edu.wpi.first.math.controller.PIDController;
public class Arm{
    private CANSparkMax armMotor;
    private DigitalInput armSwitch1;
    private DigitalInput armSwitch2;
    private CANSparkMax armMotor1;
    // private STATE m_armState;
    // private TARGET m_armTarget;
    private boolean runArm = true;
    private double finalTarget;
    private AnalogPotentiometer armPot;
    private PIDController armPID;
    private double target;
    private double current;
    private double output;
    public Arm(){
        //armMotor = new CANSparkMax(ElevatorArmConstants.ArmID, MotorType.kBrushless);
        //armSwitch1 = new DigitalInput(ElevatorArmConstants.armSwitch1Port);
        //armSwitch2 = new DigitalInput(ElevatorArmConstants.armSwitch2Port);
        armMotor1 = new CANSparkMax(11, MotorType.kBrushless);
        armPot = new AnalogPotentiometer(0);
        armPID = new PIDController(0, 0, 0);
        armMotor1.setIdleMode(IdleMode.kBrake);
    }
    public static enum RunArm { 
        UP,DOWN
    }
    // public static RunArm ArmState;
    @Override
    public void run(RunArm ArmState){
        current = GetPotValue();
        switch(ArmState){
            case UP:
                target = 0.6285;
                output = armPID.calculate(target-current);
                SetArmSpeed(output);
            
            break;
            case DOWN:
                target = 0.98;
                output = armPID.calculate(target-current);
                SetArmSpeed(output);
            break;
        }
        
    }

    /*
    public void SetArmState(STATE state){
        m_armState = state;
    }*/
    public void SetArmSpeed(double speed){
        armMotor1.set(speed);

    }
    

    public double GetPotValue(){
        return armPot.get();
    }
}