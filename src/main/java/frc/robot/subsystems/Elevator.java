package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;



public class Elevator extends SubsystemBase {
    private static Elevator instance;
    public Elevator() {

    }
    
    @Override
    public void periodic() {

    }

    public static Elevator getInstance() {
        if ( instance == null) {
            instance = new Elevator();
        }
        return instance;
    }

    

}
