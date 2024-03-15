package frc.robot.Subsystems;

import frc.robot.Constants.Swerve_Constants;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.List;
import java.util.stream.Collectors;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import org.apache.commons.math3.util.MathUtils;
import org.photonvision.PhotonUtils;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import static edu.wpi.first.units.MutableMeasure.mutable;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
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

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.RotationTarget;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

public class Swerve extends SubsystemBase {

    private SwerveDrive swerve;
    private SendableChooser<Command> autoChooser;
    private Field2d swerveField;

      private Timer timer;

    private SwerveModule module0;
    private SwerveModule module1;
    private SwerveModule module2;
    private SwerveModule module3;

    

    private AprilTagFieldLayout aprilTagFieldLayout;
    private PIDController turnController;
    private PIDController driveController;

    private boolean alignToSpeaker;
    
    private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
    private final MutableMeasure<Angle> m_distance = mutable(Radians.of(0));
    private final MutableMeasure<Velocity<Angle>> m_velocity = mutable(RadiansPerSecond.of(0));

    private PIDController angle1 = new PIDController(.01, 0, 0.0);
    private PIDController angle2 = new PIDController(.01, 0, 0.0);
    private PIDController angle3 = new PIDController(.01, 0, 0.0);
    private PIDController angle4 = new PIDController(.01, 0, 0.0);

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
                        RobotController.getBatteryVoltage()*module0.getAngleMotor().getAppliedOutput(), Volts))
                        .angularPosition(m_distance.mut_replace(module0.getAngleMotor().getPosition(), Radians))
                        .angularVelocity(m_velocity.mut_replace(module0.getAngleMotor().getVelocity(), RadiansPerSecond));
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

        turnController = new PIDController(Swerve_Constants.turnKP, Swerve_Constants.turnKI, Swerve_Constants.turnKD);
        driveController = new PIDController(Swerve_Constants.driveKP, Swerve_Constants.driveKI, Swerve_Constants.driveKD);
        turnController.setTolerance(0.02, 0.01);
        driveController.setTolerance(0.05, 0.01);
        swerveField = new Field2d();
        SmartDashboard.putData("Swerve Field", swerveField);
        motorInvert();
        swerve.chassisVelocityCorrection = true;
        alignToSpeaker = false;
        module0 = swerve.getModules()[0];
        module1 = swerve.getModules()[1];
        module2 = swerve.getModules()[2];
        module3 = swerve.getModules()[3];
    }

    public void setUpPathPlanner() {
        AutoBuilder.configureHolonomic(
            this::getPose,
            this::resetOdometry,
            this::getRobotVelocity,
            this::setChassisSpeeds,
            new HolonomicPathFollowerConfig(
                new PIDConstants(1, 0.0, 0.4),
                new PIDConstants(2, 0.0, 0.4),
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
            autoChooser = AutoBuilder.buildAutoChooser("Mid2");

        SmartDashboard.putData("Auto Selection", autoChooser);
    }

    public void drive(Translation2d translation2d, double rotationSpeed, boolean fieldRelative) {
        if (alignToSpeaker) {
            swerve.drive(translation2d, getSpeakerAimVelocity(), fieldRelative, true);
        } else {
           swerve.drive(translation2d, rotationSpeed, fieldRelative, true); 
        }
    }

    public Command getAuto() {
        Pose2d autoStartingPose = PathPlannerAuto.getStaringPoseFromAutoFile(autoChooser.getSelected().getName()); 
        if(isRedAlliance()) autoStartingPose = flipPose(autoStartingPose);
        swerve.resetOdometry(autoStartingPose);
        return autoChooser.getSelected();
    }

    public void driveToAmp(){
        Pose2d targetPose = isRedAlliance() ? flipPose(Swerve_Constants.ampAlignPose) : Swerve_Constants.ampAlignPose;       
        PathConstraints constraints = new PathConstraints(3.0, 3.0, Units.degreesToRadians(360), Units.degreesToRadians(720));
        Command pathfindingCommand = AutoBuilder.pathfindToPose(targetPose,constraints,0.0, 0.1);
        pathfindingCommand.addRequirements(this);
        pathfindingCommand.schedule();;
    }

    public void driveToTrap(){
        Pose2d targetPose = isRedAlliance() ? flipPose(Swerve_Constants.trapAlignPose) : Swerve_Constants.trapAlignPose;       
        PathConstraints constraints = new PathConstraints(3.0, 3.0, Units.degreesToRadians(360), Units.degreesToRadians(720));
        Command pathfindingCommand = AutoBuilder.pathfindToPose(targetPose,constraints,0.0, 0.1);
        pathfindingCommand.addRequirements(this);
        pathfindingCommand.schedule();
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

    public void setSpeakerAlign(boolean align) {
        this.alignToSpeaker = align;
    }

    public double getSpeakerAimVelocity() {
        return allianceInvert()*turnController.calculate(getRadiansToTarget(),0);
    }

    public boolean turnAligned() {
        return (turnController.atSetpoint());
    }

    public double getRadiansToTarget() {
        Pose2d targetPose = isRedAlliance() ? aprilTagFieldLayout.getTagPose(4).get().toPose2d() : aprilTagFieldLayout.getTagPose(7).get().toPose2d();
        double radiansToPose = MathUtils.normalizeAngle(PhotonUtils.getYawToPose(swerve.getPose(), targetPose).rotateBy(new Rotation2d(Math.toRadians(180))).getRadians(),0);
        return radiansToPose;
    }

    public double getDistanceToTarget() {
        Pose2d targetPose = isRedAlliance() ? aprilTagFieldLayout.getTagPose(4).get().toPose2d() : aprilTagFieldLayout.getTagPose(7).get().toPose2d();
        double distanceToPose = PhotonUtils.getDistanceToPose(swerve.getPose(), targetPose);
        return distanceToPose;
    }

    public void autonomousInit(){
        motorInvert();
    }

    public void motorInvert(){
        boolean invert = isRedAlliance(); 
        for(int i = 0; i < 4; i++){
            swerve.getModules()[i].getDriveMotor().setInverted(invert);
        } 
        //TODO: Check if redesigned method actually inverts properly
    }

    public boolean isRedAlliance(){
        return DriverStation.getAlliance().isPresent() ? DriverStation.getAlliance().get() == DriverStation.Alliance.Red : false; 
    }

    public Pose2d flipPose(Pose2d pose) {
        return new Pose2d(Swerve_Constants.fieldLengthMeters - pose.getX(), pose.getY(), new Rotation2d(Math.PI).minus(pose.getRotation()));
    }

    public double allianceInvert()
    {
        if(isRedAlliance())
        {
            return -1;
        }
        return 1;
    }

    @Override
    public void periodic() {
        SmartDashboard.putString("SwervePose", swerve.getPose().toString());
       //swerveField.setRobotPose(swerve.getPose());
        //SmartDashboard.putNumber("Distance to Speaker", getDistanceToTarget());
        //System.out.println(module0.getAbsolutePosition());
    }

    /**
   * Returns a command that will execute a quasistatic test in the given direction.
   *
   * @param direction The direction (forward or reverse) to run the test in
   */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return routine.quasistatic(direction);
  }

  /**
   * Returns a command that will execute a dynamic test in the given direction.
   *
   * @param direction The direction (forward or reverse) to run the test in
   */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return routine.dynamic(direction);
  }


 public void setVoltage(double volts){
    //System.out.println(volts);
        module0.getAngleMotor().setVoltage(volts);
        //module1.getAngleMotor().setVoltage(volts);
        //module2.getAngleMotor().setVoltage(volts);
        //module3.getAngleMotor().setVoltage(volts);

        
        /* 
        module0.getAngleMotor().set(angle1.calculate(module0.getAbsolutePosition(), 270));
        module1.getAngleMotor().set(angle2.calculate(module1.getAbsolutePosition(), 270));
        module2.getAngleMotor().set(angle3.calculate(module2.getAbsolutePosition(), 270));
        module3.getAngleMotor().set(angle4.calculate(module3.getAbsolutePosition(), 270));
        */


    }
}
