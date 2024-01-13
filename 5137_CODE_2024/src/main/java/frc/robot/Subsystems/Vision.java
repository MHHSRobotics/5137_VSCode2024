package frc.robot.Subsystems;

import java.io.IOException;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase{
    
    private final PhotonCamera camera = new PhotonCamera("cameraNameHere");
    private AprilTagFieldLayout aprilTagFieldLayout;
    private PhotonPoseEstimator visionPoseEstimator;

    public Vision()
    {
        try {
        aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
        } 
        catch (IOException e) {
        }
    }

     public Optional<EstimatedRobotPose> getPoseFromARCamCamera(Pose2d referencePose) 
    {
      visionPoseEstimator.setReferencePose(referencePose);
      return visionPoseEstimator.update();
    }
    
}

