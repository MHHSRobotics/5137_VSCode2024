package frc.robot.Subsystems;

import frc.robot.Robot;
import frc.robot.Constants.Arm_Constants;
import frc.robot.Other.ArmTrajectoryAlignment;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.io.File;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class Arm extends ProfiledPIDSubsystem {
    private CANSparkMax leftMotor;
    private CANSparkMax rightMotor;
    private DutyCycleEncoder encoder;
    private ArmFeedforward feedForward;

    private RelativeEncoder leftEncoder;
    private RelativeEncoder rightEncoder;

    private ArmTrajectoryAlignment align;

    private Timer timer;

    private boolean encoderInverted;

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
                        .angularPosition(m_distance.mut_replace((leftEncoder.getPosition())/(227.555), Rotations))
                        .angularVelocity(m_velocity.mut_replace((leftEncoder.getVelocity())/(227.555*60), RotationsPerSecond));
                log.motor("arm-right")
                .voltage(
                    m_appliedVoltage.mut_replace(
                        rightMotor.getBusVoltage()*rightMotor.getAppliedOutput(), Volts))
                        .angularPosition(m_distance.mut_replace((rightEncoder.getPosition())/(227.555), Rotations))
                        .angularVelocity(m_velocity.mut_replace((rightEncoder.getVelocity())/(227.555*60), RotationsPerSecond));
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

        feedForward = new ArmFeedforward(
            Arm_Constants.kS,
            Arm_Constants.kG,
            Arm_Constants.kV,
            Arm_Constants.kA);

        encoder = new DutyCycleEncoder(Arm_Constants.encoderID);
        encoder.setDistancePerRotation(-2*Math.PI);
        encoder.setPositionOffset(0.134);

        timer = new Timer();
        timer.reset();

        align = new ArmTrajectoryAlignment(RobotConstants, 0.65, 4.5, 30.0);

        try {
            Thread.sleep(2000);
        } catch (InterruptedException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }

        encoderInverted = (encoder.getDistance() < -Math.PI);

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
        if (encoderInverted) {
            return (encoder.getDistance()+(2*Math.PI))%(2*Math.PI);
        } else {
            return encoder.getDistance();
        }
    }

    public void runManual(double output) {
        if (!encoder.isConnected()) {
            leftMotor.set(0.3*output);
            rightMotor.set(0.3*output);
        }
    }

    public void alignToSpeaker(double position) {
        this.setGoal(align.calculateAngle(position));
    }

    public double getGoal() {
        return super.m_controller.getGoal().position;
    }

    public boolean getMovementFinished() {
        return (Math.abs(this.getMeasurement() - super.m_controller.getGoal().position)) < Arm_Constants.errorMargin;
    }

    private void updateDashboard() {
        SmartDashboard.putBoolean("Encoder", encoder.isConnected());
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

    public Command sysIdQuasisttatic(SysIdRoutine.Direction direction) {
        timer.restart();
        return routine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        timer.restart();
        return routine.dynamic(direction);
    }
}