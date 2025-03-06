package frc.robot.subsystems;

import java.lang.annotation.Target;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import javax.naming.spi.DirStateFactory.Result;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import swervelib.SwerveDrive;
import swervelib.telemetry.SwerveDriveTelemetry;
/** 
@param currentPose Current pose supplier, should reference {@link SwerveDrive#getPose()}
* @param field Current field, should be {@link SwerveDrive#field}
*/
public class PhotonVision extends SubsystemBase {
    
    private static PhotonVision instance;
    private final PhotonCamera camera;
    private PhotonTrackedTarget target;

    public static final AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
    private Supplier<Pose2d> currentPose;
  /**
   * Field from {@link swervelib.SwerveDrive#field}
   */
  private Field2d field2d;



    public PhotonVision(Supplier<Pose2d> currentPose, Field2d field2d) {
        this.currentPose = currentPose;
        this.field2d = field2d;
        camera = new PhotonCamera("Microsoft_LifeCam_HD-3000");
        camera.setPipelineIndex(0);
       
    }

    

    
    

      /**
   * Calculates a target pose relative to an AprilTag on the field.
   *
   * @param aprilTag    The ID of the AprilTag.
   * @param robotOffset The offset {@link Transform2d} of the robot to apply to the pose for the robot to position
   *                    itself correctly.
   * @return The target pose of the AprilTag.
   */
  public static Pose2d getAprilTagPose(int aprilTag, Transform2d robotOffset)
  {
    Optional<Pose3d> aprilTagPose3d = fieldLayout.getTagPose(aprilTag);
    if (aprilTagPose3d.isPresent())
    {
      return aprilTagPose3d.get().toPose2d().transformBy(robotOffset);
    } else
    {
      throw new RuntimeException("Cannot get AprilTag " + aprilTag + " from field " + fieldLayout.toString());
    }

  }

  public PhotonTrackedTarget getTargetFromId(int id, PhotonCamera camera)
  {
    PhotonTrackedTarget target = null;
    for (PhotonPipelineResult result : camera.getAllUnreadResults())
    {
      if (result.hasTargets())
      {
        for (PhotonTrackedTarget i : result.getTargets())
        {
          if (i.getFiducialId() == id)
          {
            return i;
          }
        }
      }
    }
    return target;

  }

  public int getPrimaryTargetId() {
    boolean targetVisible = false;
    var results = camera.getAllUnreadResults();
    if (!results.isEmpty()) {
        var result = results.get(results.size() - 1);
        PhotonTrackedTarget target = result.getBestTarget();
        return target.getFiducialId();
    } else {
        return -1;
    }
  }


     
    @Override
    public void periodic() {

        // Read in relevant data from the Camera
        boolean targetVisible = false;
        double targetYaw = 0.0;
        var results = camera.getAllUnreadResults();
        if (!results.isEmpty()) {
            // Camera processed a new frame since last
            // Get the last one in the list.
            var result = results.get(results.size() - 1);
            if (result.hasTargets()) {
                // At least one AprilTag was seen by the camera
                for (var target : result.getTargets()) {
                    if (target.getFiducialId() == 586) {
                        // Found Tag 7, record its information
                        targetYaw = target.getYaw();
                        targetVisible = true;
                    }
                }
            }
        }

    
        SmartDashboard.putBoolean("Vision Target Visible", targetVisible);


    }
        
}

