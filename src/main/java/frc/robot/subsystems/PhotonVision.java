package frc.robot.subsystems;

import java.lang.annotation.Target;
import java.util.List;

import javax.naming.spi.DirStateFactory.Result;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class PhotonVision extends SubsystemBase {
    
    private static PhotonVision instance;
    private final PhotonCamera camera;
    private PhotonTrackedTarget target;


    public PhotonVision() {

        camera = new PhotonCamera("Microsoft_LifeCam_HD-3000");
        camera.setPipelineIndex(0);
       
    }

    
    

    public static PhotonVision getInstance() {
		if (instance == null) {
			instance = new PhotonVision();
		}

		return instance;
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

