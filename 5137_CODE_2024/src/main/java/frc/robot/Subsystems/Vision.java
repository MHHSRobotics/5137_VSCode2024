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
    
    private final PhotonCamera camera = new PhotonCamera("SPCA2688_AV_Camera");
    private AprilTagFieldLayout aprilTagFieldLayout;
    private PhotonPoseEstimator visionPoseEstimator;
    private Optional<EstimatedRobotPose> globalPose = Optional.of(new EstimatedRobotPose(new Pose3d(0,0,0, new Rotation3d(0,0,0)), 0, null, null));

    public Vision()
    {
        try {
        aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
        } 
        catch (IOException e) {
        }

        visionPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.LOWEST_AMBIGUITY, camera, Vision_Constants.robotToCamera);
      
    }

     public Optional<EstimatedRobotPose> getEstimatedVisionPose() 
    {
      //TODO: Add swerve pose estimator into method once added
      
        globalPose = visionPoseEstimator.update();
        return globalPose;
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
  
      if(globalPose.isPresent()){
        EstimatedRobotPose pose = globalPose.get();
        {
        SmartDashboard.putString("Robot Pose", poseString(pose));
        SmartDashboard.putNumber("Y", distanceFromOriginY(pose));
        SmartDashboard.putNumber("X",distanceFromOriginX(pose));
        SmartDashboard.putNumber("Distance from Origin",distanceFromOrigin(pose));
        }
        
      }
    
    }
} 

