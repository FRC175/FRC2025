// package frc.robot.subsystems;

// import java.util.PrimitiveIterator;

// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Subsystem;
// import frc.robot.Constants.DriveConstants;
// // import frc.robot.subsystems.Drive.Drive;
// import frc.robot.subsystems.Cage;
// import frc.robot.subsystems.Drive.Drive;

// public final class Shuckleboard implements Subsystem {

//     // These variables are final because they only need to be instantiated once (after all, you don't need to create a
//     // new left master TalonSRX).
    
    
//     /**
//      * The single instance of {@link Drive} used to implement the "singleton" design pattern. A description of the
//      * singleton design pattern can be found in the JavaDoc for {@link Drive::getInstance()}.
//      */
//     public static Shuckleboard instance;
//     private Cage cage;

   

//     private Shuckleboard() {
// 		this.cage = cage;
		
// 	}

// 	public static Shuckleboard getInstance() {
// 		if (instance == null) {
// 			instance = new Shuckleboard();
// 		}

// 		return instance;
// 	}

//     @Override
//     public void periodic() {
        
//     }
    
// // 
//     public void logTargeted() {
//         SmartDashboard.putBoolean("HallSesnor", cage.getSensor());
//     }
    
  

// }
