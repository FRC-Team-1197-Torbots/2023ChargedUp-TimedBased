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
    public static final int Intake1ID = 9;
    public static final int Intake2ID = 10;
    private Solenoid Intakeout;
    private CANSparkMax RollerBottom;
    private CANSparkMax RollerTop;
    private Encoder IntakeEncoder;
    public Intake(){
        Intakeout = new Solenoid(PneumaticsModuleType.REVPH, 1);
        RollerBottom = new CANSparkMax(Intake1ID, MotorType.kBrushless);
        RollerTop = new CANSparkMax(Intake2ID, MotorType.kBrushless);
        //IntakeEncoder = new Encoder(0, 0);
        //IntakeEncoder.reset();
    }

}