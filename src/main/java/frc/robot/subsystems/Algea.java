package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;



public class Algea extends SubsystemBase {
    private static Algea instance;
    public Algea() {

    }
    
    @Override
    public void periodic() {

    }

    public static Algea getInstance() {
        if ( instance == null) {
            instance = new Algea();
        }
        return instance;
    }

    

}
