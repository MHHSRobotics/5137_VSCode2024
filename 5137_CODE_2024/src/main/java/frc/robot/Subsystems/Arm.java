package frc.robot.Subsystems;

import frc.robot.Constants.Arm_Constants;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;

import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

public class Arm extends ProfiledPIDSubsystem {
    private TalonFX leftMotor;
    private TalonFX rightMotor;
    private CANcoder canCoder;
    private ArmFeedforward feedForward;

    public Arm() {
        super(
            new ProfiledPIDController(
                Arm_Constants.kP,
                Arm_Constants.kI,
                Arm_Constants.kD,
                new TrapezoidProfile.Constraints(
                Arm_Constants.kMaxVelocity,
                Arm_Constants.kMaxAcceleration)),
            0.0);

        leftMotor = new TalonFX(Arm_Constants.leftMotorID);
        rightMotor = new TalonFX(Arm_Constants.rightMotorID);
        canCoder = new CANcoder(Arm_Constants.canCoderID);
        feedForward = new ArmFeedforward(
            Arm_Constants.kS,
            Arm_Constants.kG,
            Arm_Constants.kV,
            Arm_Constants.kA
        );

        setGoal(0.0);  
    }

    @Override
    public void useOutput(double output, State setpoint) {
        double feed = feedForward.calculate(setpoint.position, setpoint.velocity);
        leftMotor.setVoltage(output + feed);
        rightMotor.setVoltage(output + feed);

    }

    @Override
    public double getMeasurement() {
        return Math.toRadians(canCoder.getPosition().getValueAsDouble());
    }

    @Override 
    public void simulationPeriodic()
    {
        useOutput(super.m_controller.calculate(getMeasurement()), super.m_controller.getSetpoint());
    }

    
}