package frc.robot.Subsystems;

import frc.robot.Constants.Swerve_Constants;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;

import org.apache.commons.math3.util.MathUtils;
import org.photonvision.PhotonUtils;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

public class Swerve extends SubsystemBase {

    private SwerveDrive swerve;
    private SendableChooser<Command> autoChooser;
    private Field2d swerveField;

    private AprilTagFieldLayout aprilTagFieldLayout;
    private PIDController turnController;
    private PIDController driveController;

    public Swerve(File directory) {
        try {
            swerve = new SwerveParser(directory).createSwerveDrive(Swerve_Constants.maxVelocity);
        } catch (IOException e) {
            throw new RuntimeException(e);
        }

        try {
          aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
        } catch (IOException e) {
          e.printStackTrace();
        }

        turnController = new PIDController(Swerve_Constants.turnKP, Swerve_Constants.turnKI, Swerve_Constants.turnKD);
        driveController = new PIDController(Swerve_Constants.driveKP, Swerve_Constants.driveKI, Swerve_Constants.driveKD);
        turnController.setTolerance(0.02, 0.01);
        driveController.setTolerance(0.05, 0.01);
        swerveField = new Field2d();
        motorInvert();
        setUpPathPlanner();
    }

    public void setUpPathPlanner() {
        AutoBuilder.configureHolonomic(
            this::getPose,
            this::resetOdometry,
            this::getRobotVelocity,
            this::setChassisSpeeds,
            new HolonomicPathFollowerConfig(
                new PIDConstants(.02065, 0.0, 0.0),
                new PIDConstants(.01, 0, 0),
                Swerve_Constants.maxModuleSpeed,
                swerve.swerveDriveConfiguration.getDriveBaseRadiusMeters(),
                new ReplanningConfig()),
            () -> {
                var alliance = DriverStation.getAlliance();
                if(alliance.isPresent())
                {
                return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
            },
            this);
            autoChooser = AutoBuilder.buildAutoChooser("middleTop");
    }

    public void drive(Translation2d translation2d, double rotationSpeed, boolean fieldRelative) {

        swerve.drive(translation2d, rotationSpeed, fieldRelative, true);
    }

    public Command getAuto() {
        return autoChooser.getSelected();
    }

    public void setChassisSpeeds(ChassisSpeeds velocity) {
        swerve.setChassisSpeeds(velocity);
    }

    public void resetOdometry(Pose2d pose) {
        swerve.resetOdometry(pose);
      }

    public void zeroGyro() {
        swerve.zeroGyro();        
    }

    public ChassisSpeeds getRobotVelocity() {
        return swerve.getRobotVelocity();
    }

    public Pose2d getPose() {
        return swerve.getPose();
    }

    public void addVisionMeasurement(Pose2d pose, double timestamp){ 
        swerve.addVisionMeasurement(pose, timestamp);
    }

    public void aimAtSpeaker() {
        double turnVelocity = turnController.calculate(getRadiansToTarget(),0);
        drive(new Translation2d(0,0),turnVelocity, true);
    }

    public boolean turnAligned() {
        return (turnController.atSetpoint());
    }

    public double getRadiansToTarget() {
        Pose2d targetPose;
        if (DriverStation.getAlliance().isPresent() ? DriverStation.getAlliance().get() == DriverStation.Alliance.Red : false) {
            targetPose = aprilTagFieldLayout.getTagPose(4).get().toPose2d(); 
        } else {
            targetPose = aprilTagFieldLayout.getTagPose(7).get().toPose2d();
        }
        double radiansToPose = MathUtils.normalizeAngle(PhotonUtils.getYawToPose(swerve.getPose(), targetPose).rotateBy(new Rotation2d(Math.PI)).getRadians(),0);
        //TODO: Check if inverting motors screwed this control up
        return radiansToPose;
    }

    public double getDistanceToTarget() {
        Pose2d targetPose;
        if (DriverStation.getAlliance().isPresent() ? DriverStation.getAlliance().get() == DriverStation.Alliance.Red : false) {
            targetPose = aprilTagFieldLayout.getTagPose(4).get().toPose2d(); 
        } else {
            targetPose = aprilTagFieldLayout.getTagPose(7).get().toPose2d();
        }
        double distanceToPose = PhotonUtils.getDistanceToPose(swerve.getPose(), targetPose);
        return distanceToPose;
    }

    @Override
    public void periodic() {
        SmartDashboard.putString("SwervePose", swerve.getPose().toString());
        SmartDashboard.putData("Auto Selection", autoChooser);
        updateSwerveField();
    }

    public void motorInvert(){
        if(DriverStation.getAlliance().isPresent() ? DriverStation.getAlliance().get() == DriverStation.Alliance.Red : false){
            for(int i = 0; i <4; i++){
                swerve.getModules()[i].getDriveMotor().setInverted(true);
            }
        }
        else{
            for(int i = 0; i <4; i++){
                swerve.getModules()[i].getDriveMotor().setInverted(false);
            }
        }
    }

    public void updateSwerveField(){
        List<Pose2d> poses = new ArrayList<>();
        List<PathPlannerPath> paths = PathPlannerAuto.getPathGroupFromAutoFile(autoChooser.getSelected().getName());
        for(PathPlannerPath path: paths){
            PathPlannerPath autoPath = path;
            poses.addAll(autoPath.getAllPathPoints().stream()
            .map(point -> new Pose2d(point.position, new Rotation2d(0,0)))
            .collect(Collectors.toList()));
        }
        swerveField.getObject("path").setPoses(poses); //Displays selected autopath on field2d
        swerveField.setRobotPose(swerve.getPose());
        SmartDashboard.putData("Swerve Field", swerveField);
    }
}