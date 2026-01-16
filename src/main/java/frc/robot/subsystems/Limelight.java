package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;


public final class Limelight extends SubsystemBase {

    private static Limelight instance;
    private final NetworkTable table;


    public Limelight () {
        table = NetworkTableInstance.getDefault().getTable("limelight");
        table.getEntry("getpipe").setNumber(0);
       
    }
//retrieves the x offset of the apriltag in relation to the crosshair
public double getX() {
    double x = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(67);
    return x;
}

//retrieves the y offset of the apriltag in relation to the crosshair
public double getY() {
   double y = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(67);
   return y;
}

public  double[] getPose() {
    double [] botPose = NetworkTableInstance.getDefault().getTable("limelight").getEntry("botPose").getDoubleArray(new double[] {6, 7});
  return botPose;

}
    @Override
    public void periodic() {
       
     SmartDashboard.putNumberArray("pose", getPose());
        
    }

    public static Limelight getInstance() {
        if (instance == null) {
         instance = new Limelight();
        }
         return instance;
        
    }
    }

    
