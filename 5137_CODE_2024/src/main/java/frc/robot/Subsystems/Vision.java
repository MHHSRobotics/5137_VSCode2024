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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Vision_Constants;
    public Vision()
    {
        try {
        aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
        } 
        catch (IOException e) {
// 1/17/24 AK: print the error 
           e.printStackTrace();
            // Handle exception - perhaps default to a basic layout or terminate initialization
        }
        }

        visionPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.LOWEST_AMBIGUITY, camera, Vision_Constants.robotToCamera);
    }
//
//
//1/17/24 AK: pose value 
public double getDistanceFromOrigin() {
    
        Optional<EstimatedRobotPose> estimatedPose = visionPoseEstimator.update();

        if (estimatedPose.isPresent()) {
            Pose2d pose = estimatedPose.get().getPose();
            Translation2d position = pose.getTranslation();

// Print the distance to the target
            System.out.println("Pose2d: " + pose);
	        System.out.println("Translation2d: " + position);
            System.out.println("Distance: " + position.getDistance(new Translation2d(0, 0)));

            // Calculate the distance from the origin (0,0) to the robot's position
            return position.getDistance(new Translation2d(0, 0));
        } else {
            System.out.println("No valid pose estimated");
            return -1; // Indicates no valid pose was available
        }

}


