package frc.robot.Commands;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import frc.robot.Subsystems.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;


public class AddVisionMeasurement extends Command {

  Vision vision;
  Pose2d estimatedPose;
  double timestamp;

  public AddVisionMeasurement(Vision vision) {
    this.vision = vision;
    addRequirements(vision);
    }

  @Override
  public void initialize() {
  }
  
  @Override
  public void execute() {
    Optional<EstimatedRobotPose> ar1Pose = vision.getEstimatedAR1Pose();
    Optional<EstimatedRobotPose> ar2Pose = vision.getEstimatedAR2Pose();

    //TODO: Add result from vision system to swerve pose estimator. See off-season 2023 robot code

    }
    
    

  @Override
  public boolean isFinished() {
    return false;
  }
}