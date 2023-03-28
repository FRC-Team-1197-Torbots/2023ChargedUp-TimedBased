package frc.robot.Mechanisms;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Solenoid;
//import frc.robot.Constants.IntakeHopperConstants;
//import frc.robot.Constants.ElevatorArmConstants.GamePiece;

//Right Bumber cone,Left Bumber cube
public class Intake{

    private Solenoid Intakeout;
    private CANSparkMax RollerBottom;
    private CANSparkMax RollerTop;
    private Encoder IntakeEncoder;

    public enum moveIntake{
        UP, DOWN
    }
    public moveIntake intakeState = moveIntake.UP;

    public Intake(){
        Intakeout = new Solenoid(PneumaticsModuleType.REVPH, 1);
        RollerBottom = new CANSparkMax(9, MotorType.kBrushless);
        RollerTop = new CANSparkMax(10, MotorType.kBrushless);
        //IntakeEncoder = new Encoder(0, 0);
        //IntakeEncoder.reset();
        
    }

    public void TriggerIntake() {
        Intakeout.toggle();
    }

    
    public void run(moveIntake intakeState){
        this.intakeState = intakeState;
        switch(intakeState){
            case UP:
                Intakeout.set(false);
                RollerTop.set(0);
                RollerBottom.set(0);
                break;
            case DOWN:
                Intakeout.set(true);
                RollerTop.set(-0.5);
                RollerBottom.set(0.6);
                break;
        }
    }
}