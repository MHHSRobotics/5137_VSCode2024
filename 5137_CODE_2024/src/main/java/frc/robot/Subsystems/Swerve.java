package frc.robot.Subsystems;

import frc.robot.Constants.Arm_Constants;
import frc.robot.Constants.Swerve_Constants;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.io.File;
import java.io.IOException;

import org.photonvision.PhotonUtils;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import static edu.wpi.first.units.MutableMeasure.mutable;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import swervelib.SwerveDrive;
import swervelib.motors.SwerveMotor;
import swervelib.parser.SwerveParser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

public class Swerve extends SubsystemBase {

    private SwerveDrive swerve;

    AprilTagFieldLayout aprilTagFieldLayout;
    
    private Timer timer;

    private SwerveMotor leftFrontMotor;
    private SwerveMotor rightFrontMotor;
    private SwerveMotor leftBackMotor;
    private SwerveMotor rightBackMotor;

    private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
    private final MutableMeasure<Distance> m_distance = mutable(Meters.of(0));
    private final MutableMeasure<Velocity<Distance>> m_velocity = mutable(MetersPerSecond.of(0));

    SysIdRoutine routine = new SysIdRoutine(
        new SysIdRoutine.Config(),
        new SysIdRoutine.Mechanism(
            (Measure<Voltage> volts) -> {
                setVoltage(volts.in(Volts));
            },
            log -> {
                log.motor("swerve-left")
                .voltage(
                    m_appliedVoltage.mut_replace(
                        RobotController.getBatteryVoltage()*leftFrontMotor.getAppliedOutput(), Volts))
                        .linearPosition(m_distance.mut_replace(leftFrontMotor.getPosition(), Meters))
                        .linearVelocity(m_velocity.mut_replace(leftFrontMotor.getVelocity(), MetersPerSecond));
                log.motor("swerve-right")
                .voltage(
                    m_appliedVoltage.mut_replace(
                        RobotController.getBatteryVoltage()*rightFrontMotor.getAppliedOutput(), Volts))
                        .linearPosition(m_distance.mut_replace(rightFrontMotor.getPosition(), Meters))
                        .linearVelocity(m_velocity.mut_replace(rightFrontMotor.getVelocity(), MetersPerSecond));
            },
            this
        ));
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

        setUpPathPlanner();
        
        leftFrontMotor = swerve.getModules()[0].getDriveMotor();
        rightFrontMotor = swerve.getModules()[1].getDriveMotor();
        leftBackMotor = swerve.getModules()[2].getDriveMotor();
        rightBackMotor = swerve.getModules()[3].getDriveMotor();
        timer = new Timer();
        timer.reset();  
    }

    public void setUpPathPlanner() {
        AutoBuilder.configureHolonomic(
            this::getPose,
            this::resetOdometry,
            this::getRobotVelocity,
            this::setChassisSpeeds,
            new HolonomicPathFollowerConfig(
                new PIDConstants(5.0, 0.0, 0.0),
                new PIDConstants(
                    swerve.swerveController.config.headingPIDF.p,
                    swerve.swerveController.config.headingPIDF.i,
                    swerve.swerveController.config.headingPIDF.d),
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

    public void setVoltage(double volts)
    {
        swerve.getModules()[0].getDriveMotor().setVoltage(volts);
        swerve.getModules()[1].getDriveMotor().setVoltage(volts);
        swerve.getModules()[2].getDriveMotor().setVoltage(volts);
        swerve.getModules()[3].getDriveMotor().setVoltage(volts);
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

    public void zeroGyro() {
        swerve.zeroGyro();
    }

    public double getRadiansToTarget() {
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

    public double getDistanceToTarget() {
        Pose2d targetPose;
        if(DriverStation.getAlliance().equals(Alliance.Red))
        {
            targetPose = aprilTagFieldLayout.getTagPose(4).get().toPose2d();
        }
        else
        {
            targetPose = aprilTagFieldLayout.getTagPose(4).get().toPose2d(); 
        }
        double distanceToPose = PhotonUtils.getDistanceToPose(swerve.getPose(), targetPose);
        return distanceToPose;
    }

    public boolean robotAligned() {
        System.out.println(Math.abs(getRadiansToTarget()));
        if(Math.abs(getRadiansToTarget()) < Swerve_Constants.aimToleranceRadians && Math.abs(swerve.getRobotVelocity().omegaRadiansPerSecond) <= 0.01)
        {
            return true;
        }
        return false;
    }

    public void aimAtTarget() {
        PIDController turnController = new PIDController(Swerve_Constants.alignKP, Swerve_Constants.alignKI, Swerve_Constants.alignKD);
        turnController.setTolerance(0.02, 0.01);
        double turnVelocity = turnController.calculate(getRadiansToTarget(),0);
        drive(new Translation2d(0,0),turnVelocity, true);
    }

    public ChassisSpeeds getRobotVelocity() {
        return swerve.getRobotVelocity();
    }

    public Pose2d getPose() {
        return swerve.getPose();
    }

    public void addVisionMeasurement(Pose2d pose, double timestamp)
    { 
        swerve.addVisionMeasurement(pose, timestamp);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Distance to Target", getDistanceToTarget());
        SmartDashboard.putString("TargetPose", aprilTagFieldLayout.getTagPose(4).get().toPose2d().toString());
        SmartDashboard.putString("SwervePose", swerve.getPose().toString());

    }
    
     public Command sysIdQuasisttatic(SysIdRoutine.Direction direction) {
        timer.restart();
        return routine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        timer.restart();
        return routine.dynamic(direction);
    }
}