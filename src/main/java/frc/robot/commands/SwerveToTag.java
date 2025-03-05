// package frc.robot.commands;

// import java.lang.annotation.Target;

// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.wpilibj.smartdashboard.Field2d;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.SwerveSubsystem;
// import frc.robot.subsystems.PhotonVision;


// public class SwerveToTag extends Command{
//    private SwerveSubsystem swerveSubsystem;
//    private PhotonVision photonVision;
//    private Pose2d targetPos;
   
//     public SwerveToTag(SwerveSubsystem swerveSubsystem) {
//         this.swerveSubsystem = swerveSubsystem;
//         photonVision = swerveSubsystem.setupPhotonVisionObject();
        

      
     
//     }

//     @Override
//     public void execute() {
//         int target = photonVision.getPrimaryTargetId();
//         if (!(target == -1)) {
            
//             targetPos = PhotonVision.getAprilTagPose(target, null);
//             swerveSubsystem.driveToPose(targetPos);
//         //need to tune!
//         }
      
//     }
    
    
//     @Override
//     public boolean isFinished() {
//       return(swerveSubsystem.getPose() == targetPos);
//     }

// }
