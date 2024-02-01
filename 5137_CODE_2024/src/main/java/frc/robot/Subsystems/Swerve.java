package frc.robot.Subsystems;

import frc.robot.Constants.Swerve_Constants;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;

import org.photonvision.PhotonUtils;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import swervelib.SwerveDrive;
import swervelib.parser.PIDFConfig;
import swervelib.parser.SwerveModuleConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.parser.json.PIDFPropertiesJson;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

public class Swerve extends SubsystemBase {

    private SwerveDrive swerve;
    AprilTagFieldLayout aprilTagFieldLayout;

    
    public Swerve(File directory) {
        try {
        swerve = new SwerveParser(directory).createSwerveDrive(Swerve_Constants.maxVelocity);
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
        setUpPathPlanner();

        /* 
        try 
        {
          aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
        } 
        catch (IOException e) 
        {
          e.printStackTrace();
        }
        */
    }

    public void setUpPathPlanner() {
        SwerveModuleConfiguration moduleConfig = swerve.getModules()[0].configuration;
        PIDFConfig anglePID = moduleConfig.anglePIDF;
        PIDFConfig drivePID = moduleConfig.velocityPIDF;

        AutoBuilder.configureHolonomic(
            this::getPose,
            this::resetOdometry,
            this::getRobotVelocity,
            this::setChassisSpeeds,
            new HolonomicPathFollowerConfig(
                new PIDConstants(drivePID.p, drivePID.i, drivePID.d),
                new PIDConstants(
                    anglePID.p,
                    anglePID.i,
                    anglePID.d),
                Swerve_Constants.maxModuleSpeed,
                swerve.swerveDriveConfiguration.getDriveBaseRadiusMeters(),
                new ReplanningConfig()),
            () -> {
                var alliance = DriverStation.getAlliance();
                return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
            },
            this);
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative) {
        swerve.drive(translation, rotation, fieldRelative, false);
    }

    public Command getAuto(String name) {
        PathPlannerPath path = PathPlannerPath.fromPathFile("StartToMid");
        return AutoBuilder.followPath(path);
    }

    public void setChassisSpeeds(ChassisSpeeds velocity) {
        swerve.setChassisSpeeds(velocity);
    }

    public void resetOdometry(Pose2d pose) {
        swerve.resetOdometry(pose);
    }

    public ChassisSpeeds getRobotVelocity() {
        return swerve.getRobotVelocity();
    }

    public Pose2d getPose() {
        return swerve.getPose();
    }

    public double getRadiansToTarget()
    {
        if(DriverStation.getAlliance().equals(Alliance.Red))
        {
            Pose2d targetPose = aprilTagFieldLayout.getTagPose(4).get().toPose2d();
        }
        else
        {
            Pose2d targetPose = aprilTagFieldLayout.getTagPose(8).get().toPose2d();
        }

        Pose2d targetPose = new Pose2d(0,0, new Rotation2d());
        double radiansToPose = PhotonUtils.getYawToPose(swerve.getPose(), targetPose).getRadians();
        return radiansToPose;
    }

    public boolean robotAligned()
    {
        if(Math.abs(getRadiansToTarget()) < Swerve_Constants.aimToleranceRadians)
        {
            return true;
        }
        return false;
    }

    public void aimAtTarget()
    {
        PIDController turnController = new PIDController(Swerve_Constants.alignKP, Swerve_Constants.alignKI, Swerve_Constants.alignKD);
        double turnVelocity = turnController.calculate(getRadiansToTarget(),0);
        drive(new Translation2d(0,0),turnVelocity, true);
    }
     

}
