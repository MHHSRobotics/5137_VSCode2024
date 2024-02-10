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
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Vision_Constants;

public class Vision extends SubsystemBase{
    
    private final PhotonCamera ar1Camera = new PhotonCamera("AR1");
    private final PhotonCamera ar2Camera = new PhotonCamera("AR1");
    private AprilTagFieldLayout aprilTagFieldLayout;
    private PhotonPoseEstimator ar1PoseEstimator;
    private PhotonPoseEstimator ar2PoseEstimator;
    private Optional<EstimatedRobotPose> ar1Pose = Optional.of(new EstimatedRobotPose(new Pose3d(0,0,0, new Rotation3d(0,0,0)), 0, null, null));
    private Optional<EstimatedRobotPose> ar2Pose = Optional.of(new EstimatedRobotPose(new Pose3d(0,0,0, new Rotation3d(0,0,0)), 0, null, null));

    public Vision()
    {
        try {
        aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
        } 
        catch (IOException e) {
        }

        ar1PoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, ar1Camera, Vision_Constants.robotToAR1);
        ar2PoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, ar2Camera, Vision_Constants.robotToAR2);

    }

     public Optional<EstimatedRobotPose> getEstimatedAR1Pose(Pose2d referencePose) 
    {
      //TODO: Add swerve pose estimator into method once added
        ar1PoseEstimator.setReferencePose(referencePose);
        return ar1PoseEstimator.update();
    }

    public Optional<EstimatedRobotPose> getEstimatedAR2Pose(Pose2d referencePose) 
    {
      //TODO: Add swerve pose estimator into method once added
      
        ar2PoseEstimator.setReferencePose(referencePose);
        return ar2PoseEstimator.update();
    }

    public String poseString(EstimatedRobotPose pose)
    {
    
      return pose.estimatedPose.toPose2d().toString();
      
    }
 
    //CODE FOR TESTING, PRINTS, DASHBOARD
    public double distanceFromOriginY(EstimatedRobotPose pose)
    {
        return pose.estimatedPose.toPose2d().getY();
      
    }
    
    public double distanceFromOriginX(EstimatedRobotPose pose)
    {

     
        return pose.estimatedPose.toPose2d().getX();
     
    }

     public double distanceFromOrigin(EstimatedRobotPose pose)
    {

      
        return pose.estimatedPose.toPose2d().getTranslation().getDistance(new Translation2d(0,0));
    
    }

    @Override
    public void periodic() {
  
      if(ar1Pose.isPresent() ){
        EstimatedRobotPose pose1 = ar1Pose.get();
   
        //System.out.println(pose1.estimatedPose.toPose2d());
        {
        SmartDashboard.putNumber("timestamp Ar1",pose1.timestampSeconds);
      
        SmartDashboard.putString("Pose ar1", poseString(pose1));

        }
        
      }
      if(ar2Pose.isPresent() ){
    
        EstimatedRobotPose pose2 = ar2Pose.get();
 
        {
   
        SmartDashboard.putNumber("timestamp Ar2",pose2.timestampSeconds);
     
        SmartDashboard.putString("Pose ar2", poseString(pose2));
        }
        
      }
    
    }
} 

