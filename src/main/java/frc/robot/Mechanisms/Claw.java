package frc.robot.Mechanisms;


import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;

public class Claw{
    private Solenoid clawSolenoid;
    private CANSparkMax clawMotor;
    private boolean clawState;
    private double speed;
    private double ConeIntakeSpeed = 0.3;
    private double CubeIntakeSpeed = 0.5;
    private double CubeEjectSpeed = -0.4;
    private double IdleSpeed = 0.02;
    public Claw(){
        clawSolenoid = new Solenoid(PneumaticsModuleType.REVPH, 2);
        clawMotor = new CANSparkMax(14, MotorType.kBrushless);
    }

    public void run()
    public void SetClawSpeed(double speed){
        clawMotor.set(speed);
    }
    public void dropClaw(){
        //clawSolenoid.set(!clawSolenoid.get());
        clawSolenoid.toggle();
        //System.out.println("Claw scolenoid: " + clawSolenoid.get());
    }
    public boolean getSolenoidValue(){
        return clawSolenoid.get();
    }
    public double getCurrent(){
        return clawMotor.getOutputCurrent();
    }
}