package frc.robot.Subsystems;

import java.io.IOException;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Vision_Constants;

public class Vision extends SubsystemBase{
    
  private AprilTagFieldLayout aprilTagFieldLayout;

  /* 
    private final PhotonCamera ar1Camera = new PhotonCamera("AR1");
    */

    private final PhotonCamera ar2Camera = new PhotonCamera("AR2");
    private final PhotonCamera ov1Camera = new PhotonCamera("OV1");
    private final PhotonCamera ov2Camera = new PhotonCamera("OV2");

    private final PhotonCamera objectCamera;
    private double latestMetersToNote;
    private Translation2d latestTranslationToNote;



    //private PhotonPoseEstimator ar1PoseEstimator;
    private PhotonPoseEstimator ar2PoseEstimator;
    private PhotonPoseEstimator ov1PoseEstimator;
    private PhotonPoseEstimator ov2PoseEstimator;



    public Vision(){
        try {
        aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
        } 
        catch (IOException e) {
        }
        /* 
        ar1PoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, ar1Camera, Vision_Constants.robotToAR1);
        */
        ar2PoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, ar2Camera, Vision_Constants.robotToAR2);
        ov1PoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, ov1Camera, Vision_Constants.robotToOV1);
        ov2PoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, ov2Camera, Vision_Constants.robotToOV2);

        objectCamera = new PhotonCamera("objectCamera");
        latestMetersToNote = 0;
        latestTranslationToNote = new Translation2d();
    }
    /* 
     public Optional<EstimatedRobotPose> getEstimatedAR1Pose(Pose2d referencePose) {
      ar1PoseEstimator.setReferencePose(referencePose);
      return ar1PoseEstimator.update();
    }
    */
    public Optional<EstimatedRobotPose> getEstimatedAR2Pose(Pose2d referencePose) {      
      ar2PoseEstimator.setReferencePose(referencePose);
      return ar2PoseEstimator.update();
    }

    public Optional<EstimatedRobotPose> getEstimatedOV1Pose(Pose2d referencePose) {      
      ov1PoseEstimator.setReferencePose(referencePose);
      return ov1PoseEstimator.update();
    }
    
    public Optional<EstimatedRobotPose> getEstimatedOV2Pose(Pose2d referencePose) {      
      ov2PoseEstimator.setReferencePose(referencePose);
      return ov2PoseEstimator.update();
    }

  public double getMetersToNote()
    {
      Transform3d robotToCamera = Vision_Constants.objectCameraToRobot;
      var result = objectCamera.getLatestResult();
      if(result.hasTargets()){
        var target = result.getBestTarget();
        double distance = PhotonUtils.calculateDistanceToTargetMeters(robotToCamera.getZ(), Vision_Constants.noteDetectionHeight, robotToCamera.getRotation().getY(), Units.degreesToRadians(target.getPitch()));
        latestMetersToNote = distance;
        return distance;
      
      }
      return latestMetersToNote;
    }
    

    public Translation2d getTranslationToNote(){
      Transform3d robotToCamera = Vision_Constants.objectCameraToRobot;
      var result = objectCamera.getLatestResult();
      if(result.hasTargets()){
        var target = result.getBestTarget();
        double distance = PhotonUtils.calculateDistanceToTargetMeters(robotToCamera.getZ(), Vision_Constants.noteDetectionHeight, robotToCamera.getRotation().getY(), Units.degreesToRadians(target.getPitch()));
        Translation2d translation = PhotonUtils.estimateCameraToTargetTranslation(distance, Rotation2d.fromDegrees(-target.getYaw()));
        latestTranslationToNote = translation;
        return translation;
      
      }
      return latestTranslationToNote;
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Yaw to Object (Deg)", getTranslationToNote().getAngle().getDegrees());
        SmartDashboard.putNumber("Distance to Object (m)", getMetersToNote());

    }
}
