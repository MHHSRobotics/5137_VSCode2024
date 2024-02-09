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
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import swervelib.SwerveDrive;
import swervelib.parser.PIDFConfig;
import swervelib.parser.SwerveModuleConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.parser.json.PIDFPropertiesJson;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

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

    // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
    private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
    // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
    private final MutableMeasure<Distance> m_distance = mutable(Meters.of(0));
    // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
    private final MutableMeasure<Velocity<Distance>> m_velocity = mutable(MetersPerSecond.of(0));

    SysIdRoutine routine = new SysIdRoutine(
        new SysIdRoutine.Config(),
        new SysIdRoutine.Mechanism(
        (Measure<Voltage> volts) -> {
            setVoltage(volts.in(Volts));
        }, 
        log -> {
            // Record a frame for the left motors.  Since these share an encoder, we consider
            // the entire group to be one motor.
            log.motor("drive-left")
                .voltage(
                    m_appliedVoltage.mut_replace(
                        swerve.swerveDriveConfiguration.modules[0].getDriveMotor().getVoltage(), Volts))
                .linearPosition(m_distance.mut_replace(swerve.swerveDriveConfiguration.modules[0].getDriveMotor().getPosition(), Meters))
                .linearVelocity(
                    m_velocity.mut_replace(swerve.swerveDriveConfiguration.modules[0].getDriveMotor().getVelocity(), MetersPerSecond));
            log.motor("drive-right")
                .voltage(
                    m_appliedVoltage.mut_replace(
                        swerve.swerveDriveConfiguration.modules[1].getDriveMotor().getVoltage(), Volts))
                .linearPosition(m_distance.mut_replace(swerve.swerveDriveConfiguration.modules[1].getDriveMotor().getPosition(), Meters))
                .linearVelocity(
                    m_velocity.mut_replace(swerve.swerveDriveConfiguration.modules[1].getDriveMotor().getVelocity(), MetersPerSecond));
            },
        this));

    
    public Swerve(File directory) {
        try {
        swerve = new SwerveParser(directory).createSwerveDrive(Swerve_Constants.maxVelocity);
        } catch (IOException e) {
            throw new RuntimeException(e);
        }

        swerve.swerveDriveConfiguration.modules[0].getAngleMotor().setMotorBrake(true);
        swerve.swerveDriveConfiguration.modules[1].getAngleMotor().setMotorBrake(true);
        swerve.swerveDriveConfiguration.modules[2].getAngleMotor().setMotorBrake(true);
        swerve.swerveDriveConfiguration.modules[3].getAngleMotor().setMotorBrake(true);

        swerve.chassisVelocityCorrection = true;

        setUpPathPlanner();


        try 
        {
          aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
        } 
        catch (IOException e) 
        {
          e.printStackTrace();
        }
        
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

    public void setVoltage(double volts) {
        swerve.swerveDriveConfiguration.modules[0].getDriveMotor().setVoltage(volts);
        swerve.swerveDriveConfiguration.modules[1].getDriveMotor().setVoltage(volts);
        swerve.swerveDriveConfiguration.modules[2].getDriveMotor().setVoltage(-volts);
        swerve.swerveDriveConfiguration.modules[3].getDriveMotor().setVoltage(volts);
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

    public void zeroGyro()
  {
    swerve.zeroGyro();
  }

    public ChassisSpeeds getRobotVelocity() {
        return swerve.getRobotVelocity();
    }

    public Pose2d getPose() {
        return swerve.getPose();
    }

    public double getRadiansToTarget()
    {
        Pose2d targetPose;
        if(DriverStation.getAlliance().equals(Alliance.Red))
        {
            targetPose = aprilTagFieldLayout.getTagPose(4).get().toPose2d();
        }
        else
        {
            targetPose = aprilTagFieldLayout.getTagPose(8).get().toPose2d(); 
        }

        double radiansToPose = PhotonUtils.getYawToPose(swerve.getPose(), targetPose).getRadians();
        return radiansToPose;
    }

    public boolean robotAligned()
    {
        System.out.println(Math.abs(getRadiansToTarget()));
        if(Math.abs(getRadiansToTarget()) < Swerve_Constants.aimToleranceRadians)
        {
            return true;
        }
        return false;
    }

    public void aimAtTarget()
    {
        PIDController turnController = new PIDController(Swerve_Constants.alignKP, Swerve_Constants.alignKI, Swerve_Constants.alignKD);
        double turnVelocity = -turnController.calculate(getRadiansToTarget(),0);
        drive(new Translation2d(0,0),turnVelocity, true);
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return routine.quasistatic(direction);
      }
    
      public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return routine.dynamic(direction);
      }
}
