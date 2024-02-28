package frc.robot.Subsystems;

import frc.robot.Constants.Arm_Constants;
import frc.robot.Constants.Swerve_Constants;
import frc.robot.Constants.Vision_Constants;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;
import java.util.stream.Collectors;

import org.apache.commons.math3.util.MathUtils;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.hal.can.CANStatus;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
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
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import swervelib.SwerveDrive;
import swervelib.SwerveModule;
import swervelib.motors.SwerveMotor;
import swervelib.parser.SwerveParser;

import com.ctre.phoenix.led.CANdle;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

public class Swerve extends SubsystemBase {

    private SwerveDrive swerve;
    private  SendableChooser<Command> autoChooser;
    private Field2d swerveField;


    private AprilTagFieldLayout aprilTagFieldLayout;
    private PIDController turnController;
    private PIDController driveController;
    
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
                        RobotController.getBatteryVoltage()*-rightFrontMotor.getAppliedOutput(), Volts))
                        .linearPosition(m_distance.mut_replace(rightFrontMotor.getPosition(), Meters))
                        .linearVelocity(m_velocity.mut_replace(rightFrontMotor.getVelocity(), MetersPerSecond));
            },
            this
        ));

    public Swerve(File directory) {
        turnController = new PIDController(Swerve_Constants.turnKP, Swerve_Constants.turnKI, Swerve_Constants.turnKD);
        driveController = new PIDController(Swerve_Constants.driveKP, Swerve_Constants.driveKI, Swerve_Constants.driveKD);
        turnController.setTolerance(0.02, 0.01);
        driveController.setTolerance(0.05, 0.01);

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
        
        leftFrontMotor = swerve.getModules()[0].getDriveMotor();
        rightFrontMotor = swerve.getModules()[1].getDriveMotor();
        leftBackMotor = swerve.getModules()[2].getDriveMotor();
        rightBackMotor = swerve.getModules()[3].getDriveMotor();
        timer = new Timer();
        timer.reset();  
         swerve.getGyro().factoryDefault();
        swerve.getGyro().clearStickyFaults();
        swerveField = new Field2d();
        swerve.getPose();
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
                return DriverStation.getAlliance().isPresent() ? DriverStation.getAlliance().get() == DriverStation.Alliance.Red : false;
            },
            this);
            autoChooser = AutoBuilder.buildAutoChooser("middleTop");

    }

    public void drive(Translation2d translation2d, double rotationSpeed, boolean fieldRelative) {

        swerve.drive(translation2d, rotationSpeed, true, true);
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
        //swerve.setGyroOffset(swerve.getGyro().getRawRotation3d());
        
        swerve.setGyroOffset(swerve.getGyro().getRawRotation3d().rotateBy(new Rotation3d(0,0,swerve.getPose().getRotation().getRadians())));
      
        //swerve.zeroGyro();
        //swerve.setGyro(new Rotation3d(0,0,-Math.PI));
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

    public void turnToNote(Translation2d translationToTarget){
       
        Translation2d translation = translationToTarget;  
        double turnVelocity = turnController.calculate(translation.getAngle().getRadians(),0);
        drive(new Translation2d(0, 0), turnVelocity, false);
    }

    public void driveToNote(Translation2d translationToTarget, double metersToNote){
       
        Translation2d translation = translationToTarget;  
        double distance = metersToNote;
        double turnVelocity = turnController.calculate(translation.getAngle().getRadians(),0);
        double driveVelocity = turnController.calculate(distance, Vision_Constants.notePickupDistance);
        drive(new Translation2d(driveVelocity, 0), turnVelocity, false);
    }

    public boolean driveComplete(){
        return driveController.atSetpoint(); 
    }

    @Override
    public void periodic() {
        SmartDashboard.putString("TargetPose", aprilTagFieldLayout.getTagPose(4).get().toPose2d().toString());
        SmartDashboard.putString("SwervePose", swerve.getPose().toString());
        SmartDashboard.putData("Auto Selection", autoChooser);
        updateSwerveField();
    }

    public void updateSwerveField(){
        if(!DriverStation.isTeleopEnabled()){
        List<Pose2d> poses = new ArrayList<>();
        List<PathPlannerPath> paths = PathPlannerAuto.getPathGroupFromAutoFile(autoChooser.getSelected().getName());
        for(PathPlannerPath path: paths){
            PathPlannerPath autoPath = path;
            poses.addAll(autoPath.getAllPathPoints().stream()
            .map(point -> new Pose2d(point.position, new Rotation2d(0,0)))
            .collect(Collectors.toList()));
        }
        swerveField.getObject("path").setPoses(poses); //Displays selected autopath on field2d
        }
        else{
            swerveField.getObject("path").close();
        }
        swerveField.setRobotPose(swerve.getPose());
        SmartDashboard.putData("Swerve Field", swerveField);
    }
    
     public Command sysIdQuasisttatic(SysIdRoutine.Direction direction) {
        timer.restart();
        return routine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        timer.restart();
        return routine.dynamic(direction);
    }

    public void setVoltage(double volts){
        leftFrontMotor.setVoltage(volts);
        rightFrontMotor.setVoltage(-volts);
        leftBackMotor.setVoltage(volts);
        rightBackMotor.setVoltage(-volts);
    }
}