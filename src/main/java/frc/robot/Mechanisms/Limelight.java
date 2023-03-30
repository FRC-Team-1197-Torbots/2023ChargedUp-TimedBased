package frc.robot.Mechanisms;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Limelight {
    private NetworkTableInstance networkTableInstance;
        private NetworkTable table;// for limelight
        private NetworkTableEntry tx;
        private NetworkTableEntry ty;
        private NetworkTableEntry ta;
        private NetworkTableEntry tv;
        private NetworkTableEntry ts;
        private double x;
        private double y;
        private double area;
        private double v;
        private double s;
        

        public Limelight(){
                table = NetworkTableInstance.getDefault().getTable("limelight");
                tx = table.getEntry("tx");
                ty = table.getEntry("ty");
                ta = table.getEntry("ta");
                tv = table.getEntry("tv");
                ts = table.getEntry("ts");

                


                
        }
    
}
