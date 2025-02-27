package frc.robot.Subsystems;

import frc.robot.Constants.Arm_Constants;
import frc.robot.Other.ArmTrajectoryAlignment;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.units.*;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
//import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

//import static edu.wpi.first.units.MutableMeasure.mutable;
//import static edu.wpi.first.units.Units.Volts;
//import static edu.wpi.first.units.Units.Rotations;
//import static edu.wpi.first.units.Units.RotationsPerSecond;
//import edu.wpi.first.units.Angle;
//import edu.wpi.first.units.Measure;
//import edu.wpi.first.units.MutableMeasure;
//import edu.wpi.first.units.Velocity;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.Radian;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.io.File;
import java.util.Map;

import org.apache.commons.math3.util.MathUtils;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import static edu.wpi.first.units.MutableMeasure.mutable;


public class Arm extends ProfiledPIDSubsystem {
    
    
   
     

    private ArmTrajectoryAlignment align;
    private CANSparkMax leftMotor;
    private CANSparkMax rightMotor;
    private DutyCycleEncoder encoder;
    private ArmFeedforward feedForward;

    private SendableChooser<Boolean> manualControlChoice;
    private SimpleWidget speakerShooterOffset;
    private RelativeEncoder leftEncoder;
    private RelativeEncoder rightEncoder;



    private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
    private final MutableMeasure<Angle> m_distance = mutable(Rotations.of(0));
    private final MutableMeasure<Velocity<Angle>> m_velocity = mutable(RotationsPerSecond.of(0));

    SysIdRoutine routine = new SysIdRoutine(
        new SysIdRoutine.Config(),
        new SysIdRoutine.Mechanism(
            (Measure<Voltage> volts) -> {
                setVoltage(volts.in(Volts));
            },
            log -> {
                log.motor("arm-left")
                .voltage(
                    m_appliedVoltage.mut_replace(
                        leftMotor.getBusVoltage()*leftMotor.getAppliedOutput(), Volts))
                        .angularPosition(m_distance.mut_replace(getMeasurement()/(2*Math.PI), Rotations))
                        .angularVelocity(m_velocity.mut_replace(2*Math.PI*(leftEncoder.getVelocity())/(Arm_Constants.gearRatio)/60.0, RotationsPerSecond));
                log.motor("arm-right")
                .voltage(
                    m_appliedVoltage.mut_replace(
                        rightMotor.getBusVoltage()*rightMotor.getAppliedOutput(), Volts))
                        .angularPosition(m_distance.mut_replace(getMeasurement()/(2*Math.PI), Rotations))
                        .angularVelocity(m_velocity.mut_replace((rightEncoder.getVelocity())/(Arm_Constants.gearRatio)/60.0, RotationsPerSecond));
            },
            this
        ));
    public Arm(File RobotConstants) {
        super(
            new ProfiledPIDController(
                Arm_Constants.kP,
                Arm_Constants.kI,
                Arm_Constants.kD,
                new TrapezoidProfile.Constraints(
                Arm_Constants.kMaxVelocity,
                Arm_Constants.kMaxAcceleration)),
            0.0);
        super.m_controller.setTolerance(Math.toRadians(0.5));
        
        leftMotor = new CANSparkMax(Arm_Constants.leftMotorID, MotorType.kBrushless);
        rightMotor = new CANSparkMax(Arm_Constants.rightMotorID, MotorType.kBrushless);

        leftMotor.restoreFactoryDefaults();
        rightMotor.restoreFactoryDefaults();

        leftMotor.setSmartCurrentLimit(Arm_Constants.maxSupplyCurrent);
        rightMotor.setSmartCurrentLimit(Arm_Constants.maxSupplyCurrent);

        leftMotor.setIdleMode(IdleMode.kBrake);
        rightMotor.setIdleMode(IdleMode.kBrake);

        leftMotor.setInverted(false);
        rightMotor.setInverted(true);

        
        leftEncoder = leftMotor.getEncoder();
        rightEncoder = rightMotor.getEncoder();
        leftEncoder.setPosition(0);
        rightEncoder.setPosition(0);
        leftEncoder.setPositionConversionFactor(1.0);
        leftEncoder.setVelocityConversionFactor(1.0);
        rightEncoder.setPositionConversionFactor(1.0);
        rightEncoder.setVelocityConversionFactor(1.0);

        
        feedForward = new ArmFeedforward(
            Arm_Constants.kS,
            Arm_Constants.kG,
            Arm_Constants.kV,
            Arm_Constants.kA);

        encoder = new DutyCycleEncoder(Arm_Constants.encoderID);
        encoder.setDistancePerRotation(-2*Math.PI);
        encoder.setPositionOffset(Arm_Constants.encoderOffset);

        align = new ArmTrajectoryAlignment(RobotConstants, 0.65, 10, Arm_Constants.defaultPosition);

        manualControlChoice = new SendableChooser<Boolean>();
        manualControlChoice.addOption("Enabled", true);
        manualControlChoice.setDefaultOption("Disabled", false);
        SmartDashboard.putData("Arm Manual Control", manualControlChoice);

        speakerShooterOffset = Shuffleboard.getTab("Tab 2")
        .add("Shooter Offset", 0.0)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", -5.0, "max", 5.0));

        try {
            Thread.sleep(2000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        setGoal(getMeasurement());
        updateDashboard();
    }

    @Override
    public void useOutput(double output, State setpoint) {
        double feed = feedForward.calculate(setpoint.position, setpoint.velocity);
        setVoltage(output + feed);
    }

    public void setVoltage(double volts) {
        leftMotor.setVoltage(volts);
        rightMotor.setVoltage(volts);
    }

    @Override
    public double getMeasurement() {
        double unfilteredAngle = encoder.getDistance();
        double normalizedAngle = MathUtils.normalizeAngle(unfilteredAngle, Arm_Constants.normalRangeCenter);
        return normalizedAngle;
    }

    public void runManual(double output) {
        if (encoder.isConnected()) {
            this.setGoal(MathUtil.clamp(this.getGoal()+(output*0.02),0,Math.toRadians(100)));
        } else {
            leftMotor.set(0.3*output);
            rightMotor.set(0.3*output);
        }
    }

    public void alignToSpeaker(double position) {
        setGoal((align.calculateAngle(position))-Math.toRadians(-5.0)+Math.toRadians(speakerShooterOffset.getEntry().getDouble(0.0)));
    }

    public double getGoal() {
        return super.m_controller.getGoal().position;
    }

    public boolean getMovementFinished() {
        return (Math.abs(this.getMeasurement() - super.m_controller.getGoal().position)) < Arm_Constants.errorMargin;
    }

    private void updateDashboard() {
        //SmartDashboard.putBoolean("Encoder", encoder.isConnected());
        SmartDashboard.putNumber("Arm Position", Math.toDegrees(this.getMeasurement()));
        SmartDashboard.putNumber("Arm Goal", Math.toDegrees(this.getGoal()));
    }

    @Override
    public void periodic() {
        updateDashboard();
        if (encoder.isConnected()) {
            useOutput(super.m_controller.calculate(getMeasurement()), super.m_controller.getSetpoint());
        }
    }

    public boolean atSetpoint()
    {
        return Math.abs(getMeasurement() - getGoal()) < Math.toRadians(0.5);
    }

     
    public Command sysIdQuasisttatic(SysIdRoutine.Direction direction) {
        return routine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return routine.dynamic(direction);
    }
    
}