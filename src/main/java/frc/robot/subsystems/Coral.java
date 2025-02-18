package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;



public class Coral extends SubsystemBase {
    private static Coral instance;
    public Coral() {

    }
    
    @Override
    public void periodic() {

    }

    public static Coral getInstance() {
        if ( instance == null) {
            instance = new Coral();
        }
        return instance;
    }

    

}
