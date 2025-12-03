package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BreadMotor extends SubsystemBase {

    private static BreadMotor instance;

    public BreadMotor () {

    }

    @Override
    public void periodic() {
      
    }

    public static BreadMotor getInstance() {
        if (instance == null) {
         instance = new BreadMotor();
        }
        return instance;
     }
}
