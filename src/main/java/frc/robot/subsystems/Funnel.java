package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;



public class Funnel extends SubsystemBase {
    private static Funnel instance;
    public Funnel() {

    }
    
    @Override
    public void periodic() {

    }

    public static Funnel getInstance() {
        if ( instance == null) {
            instance = new Funnel();
        }
        return instance;
    }

    

}
