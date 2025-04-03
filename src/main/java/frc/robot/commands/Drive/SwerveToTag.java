// package frc.robot.commands.Drive;


// import java.lang.annotation.Target;

// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.wpilibj.smartdashboard.Field2d;
// import edu.wpi.first.wpilibj2.command.Command;

// import frc.robot.subsystems.Drive.Drive;
// import frc.robot.subsystems.PhotonVision;


// public class SwerveToTag extends Command{
//    private Drive drive;
//    private PhotonVision photonVision;
//    private Pose2d targetPos;
   
//     public SwerveToTag(Drive drive) {
//         this.drive = drive;
//         photonVision = drive.setupPhotonVisionObject();
        

      
     
//     }

//     @Override
//     public void execute() {
//         int target = photonVision.getPrimaryTargetId();
//         if (!(target == -1)) {
            
//             Double dist = photonVision.getX();
//             Double heading = photonVision.getY();
//             new SwerveToDist(drive, .28, heading, drive.getYaw(), dist);
//         //need to tune!
//         }
      
//     }
    
    
//     @Override
//     public boolean isFinished() {
//       return(swerveSubsystem.getPose() == targetPos);
//     }

// }
