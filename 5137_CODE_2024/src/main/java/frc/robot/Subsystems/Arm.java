package frc.robot.Subsystems;

import frc.robot.Constants.Arm_Constants;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class Arm extends ProfiledPIDSubsystem {
    private CANSparkMax leftMotor;
    private CANSparkMax rightMotor;
    private DutyCycleEncoder encoder;
    private ArmFeedforward feedForward;

    private Mechanism2d armSimMech;
    private MechanismRoot2d armSimRoot;
    private MechanismLigament2d armSim;
    private MechanismRoot2d armGoalSimRoot;
    private MechanismLigament2d armGoalSim;

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

        leftMotor = new CANSparkMax(Arm_Constants.leftMotorID, MotorType.kBrushless);
        rightMotor = new CANSparkMax(Arm_Constants.rightMotorID, MotorType.kBrushless);

        leftMotor.setSmartCurrentLimit(Arm_Constants.maxSupplyCurrent);
        rightMotor.setSmartCurrentLimit(Arm_Constants.maxSupplyCurrent);

        leftMotor.setIdleMode(IdleMode.kBrake);
        rightMotor.setIdleMode(IdleMode.kBrake);

        leftMotor.setInverted(true);

        feedForward = new ArmFeedforward(
            Arm_Constants.kS,
            Arm_Constants.kG,
            Arm_Constants.kV,
            Arm_Constants.kA);

        encoder = new DutyCycleEncoder(Arm_Constants.encoderID);
        encoder.setDistancePerRotation(2*Math.PI);

        armSimMech = new Mechanism2d(10, 10, new Color8Bit(Color.kBlack));
        armSimRoot = armSimMech.getRoot("ArmRoot", 5, 0);
        armSim = armSimRoot.append(new MechanismLigament2d("Arm", 5, 105, 10, new Color8Bit(Color.kBlue)));
        SmartDashboard.putData("Arm Sim", armSimMech);

        armGoalSimRoot = armSimMech.getRoot("GoalRoot", 5, 0);
        armGoalSim = armGoalSimRoot.append(new MechanismLigament2d("Goal", 5, 105, 10, new Color8Bit(Color.kOrange)));
        SmartDashboard.putData("Arm Sim", armSimMech);

        SmartDashboard.putNumber("Encoder Value", encoder.get());
        SmartDashboard.putNumber("Encoder Absolute Position", encoder.getAbsolutePosition());
        SmartDashboard.putNumber("Encoder Distance", encoder.getDistance());
        SmartDashboard.putNumber("Arm Position", this.getMeasurement());
        SmartDashboard.putNumber("Arm Goal", this.getGoal());

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
        return encoder.getAbsolutePosition();
    }

    public void runManual(double output) {
        leftMotor.set(0.3*output);
        rightMotor.set(0.3*output);
    }

    public double getGoal() {
        return super.m_controller.getGoal().position;
    }

    public boolean getMovementFinished() {
        return (Math.abs(this.getMeasurement() - super.m_controller.getGoal().position)) < Arm_Constants.errorMargin;
    }

    private void updateDashboard() {
        armSim.setAngle(-Math.toDegrees(this.getMeasurement())+180);
        armGoalSim.setAngle(-Math.toDegrees(this.getGoal())+180);
    }

    @Override
    public void periodic() {
        updateDashboard();
        //useOutput(super.m_controller.calculate(getMeasurement()), super.m_controller.getSetpoint());
    }
}