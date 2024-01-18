package frc.robot.Subsystems;

import java.io.IOException;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Vision_Constants;

public class Vision extends SubsystemBase{
    
    private final PhotonCamera camera = new PhotonCamera("SPCA2688_AV_Camera");
    private AprilTagFieldLayout aprilTagFieldLayout;
    private PhotonPoseEstimator visionPoseEstimator;

    public Vision()
    {
        try {
        aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
        } 
        catch (IOException e) {
        }

        visionPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.LOWEST_AMBIGUITY, camera, Vision_Constants.robotToCamera);
        SmartDashboard.putString("Robot Pose", getEstimatedVisionPose().get().estimatedPose.toString());
        SmartDashboard.putNumber("Distance from Origin", getEstimatedVisionPose().get().estimatedPose.getTranslation().toTranslation2d().getDistance(new Translation2d(0,0)));
    }

     public Optional<EstimatedRobotPose> getEstimatedVisionPose() 
    {
      //TODO: Add swerve pose estimator into method once added
      return visionPoseEstimator.update();
    }
    
}

