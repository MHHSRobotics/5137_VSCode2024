package frc.robot.Constants;

import java.util.Arrays;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class Swerve_Constants { 
    // Deadbands
    public static final double LX_Deadband = 0.1;
    public static final double LY_Deadband = 0.1;
    public static final double RX_Deadband = 0.1;

    // Physics
    public static final double maxVelocity = 4.5;
    public static final double maxAngularSpeed = 10.0;
    public static final double maxModuleSpeed = 4.5;

    //Constants
    public static final double aimToleranceRadians = 0.05;
    public static final double aimToleranceRadiansPerSecond = 0.02;
    public static final double driveToleranceMeters = 0.05;
    public static final double driveToleranceMetersPerSecond = 0.2;
    public static final double turnKP = 3.5;
    public static final double turnKI = 0.0;
    public static final double turnKD = 0.0;
    public static final double driveKP = .5;
    public static final double driveKI = 0.0;
    public static final double driveKD = 0.0;

    //Align Positions
    public static final Pose2d trapAlignPoseTop = new Pose2d(4.40, 3.37, Rotation2d.fromDegrees(-120));
    public static final Pose2d trapAlignPoseBottom = new Pose2d(4.40, 4.8, Rotation2d.fromDegrees(120));
    public static final Pose2d trapAlignPoseSide = new Pose2d(5.8, 4.1, Rotation2d.fromDegrees(0));
    public static final List<Pose2d> trapAlignPoses = Arrays.asList(trapAlignPoseTop, trapAlignPoseSide, trapAlignPoseBottom);

    public static final Pose2d ampAlignPose = new Pose2d(1.8, 7.81, Rotation2d.fromDegrees(-90));
    public static final double fieldLengthMeters = Units.inchesToMeters(651.25);

}
