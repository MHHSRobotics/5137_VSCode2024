package frc.robot.Commands;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import frc.robot.Subsystems.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;


public class AddVisionMeasurement extends Command {

  Vision vision;
  Swerve swerve;
  Pose2d estimatedPose;
  double timestamp;

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
    Optional<EstimatedRobotPose> ar1Pose = vision.getEstimatedAR1Pose(swerve.getPose());
    Optional<EstimatedRobotPose> ar2Pose = vision.getEstimatedAR2Pose(swerve.getPose());

    if(ar1Pose.isPresent())
    {
      Pose2d pose = ar1Pose.get().estimatedPose.toPose2d();
      double timestamp = ar1Pose.get().timestampSeconds;
      swerve.addVisionMeasurement(pose, timestamp);
    }
    if(ar2Pose.isPresent())
    {
      Pose2d pose = ar2Pose.get().estimatedPose.toPose2d();
      double timestamp = ar2Pose.get().timestampSeconds;
      swerve.addVisionMeasurement(pose, timestamp);
    }
    //TODO: Add result from vision system to swerve pose estimator. See off-season 2023 robot code

    }
    
    

  @Override
  public boolean isFinished() {
    return false;
  }
}