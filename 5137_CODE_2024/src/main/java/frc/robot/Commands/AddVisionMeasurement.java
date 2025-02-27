package frc.robot.Commands;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import frc.robot.Subsystems.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;


public class AddVisionMeasurement extends Command {

  Vision vision;
  Swerve swerve;

  public AddVisionMeasurement(Vision vision, Swerve swerve) {
    this.vision = vision;
    this.swerve = swerve;
    addRequirements(vision);
    }

  @Override
  public void initialize() {
  }
  
  @Override
  public void execute() {
    Optional<EstimatedRobotPose> ar2Pose = vision.getEstimatedAR2Pose(swerve.getPose());
    if(ar2Pose.isPresent())
    {
      Pose2d pose = ar2Pose.get().estimatedPose.toPose2d();
      double timestamp = ar2Pose.get().timestampSeconds;
      swerve.addVisionMeasurement(pose, timestamp);
    }

    Optional<EstimatedRobotPose> ov1Pose = vision.getEstimatedOV1Pose(swerve.getPose());
    if(ov1Pose.isPresent())
    {
      Pose2d pose = ov1Pose.get().estimatedPose.toPose2d();
      double timestamp = ov1Pose.get().timestampSeconds;
      swerve.addVisionMeasurement(pose, timestamp);
    }

    /*
    Optional<EstimatedRobotPose> ov2Pose = vision.getEstimatedOV2Pose(swerve.getPose());
    if(ov2Pose.isPresent())
    {
      Pose2d pose = ov2Pose.get().estimatedPose.toPose2d();
      double timestamp = ov2Pose.get().timestampSeconds;
      swerve.addVisionMeasurement(pose, timestamp);
    }*/

 
     /* 
    Optional<EstimatedRobotPose> ar1Pose = vision.getEstimatedAR1Pose(swerve.getPose());
    if(ar1Pose.isPresent())
    {
      Pose2d pose = ar1Pose.get().estimatedPose.toPose2d();
      double timestamp = ar1Pose.get().timestampSeconds;
      swerve.addVisionMeasurement(pose, timestamp);
    }

    */
  
  
    }
    
    

  @Override
  public boolean isFinished() {
    return false;
  }
}