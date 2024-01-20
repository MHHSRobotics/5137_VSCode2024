package frc.robot.Subsystems;

import frc.robot.Robot;
import frc.robot.Constants.Arm_Constants;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.CANcoderSimState;

public class Arm extends ProfiledPIDSubsystem {
    private TalonFX leftMotor;
    private TalonFX rightMotor;
    private CANcoder canCoder;
    private ArmFeedforward feedForward;

    private CANcoderSimState canCoderSim;

    private Mechanism2d armSimMech;
    private MechanismRoot2d armSimRoot;
    private MechanismLigament2d armSim;

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

        if (Robot.isSimulation()) {
            canCoder.setPosition(0.0);
            canCoderSim = canCoder.getSimState();
            armSimMech = new Mechanism2d(10, 10, new Color8Bit(Color.kBlack));
            armSimRoot = armSimMech.getRoot("ArmRoot", 5, 0);
            armSim = armSimRoot.append(new MechanismLigament2d("Arm", 5, 105, 10, new Color8Bit(Color.kRed)));
            SmartDashboard.putData("Arm Sim", armSimMech);
        }

        setGoal(0.0);
    }

    @Override
    public void useOutput(double output, State setpoint) {
        double feed = feedForward.calculate(setpoint.position, setpoint.velocity);
        leftMotor.setVoltage(output + feed);
        rightMotor.setVoltage(output + feed);
        
        if (Robot.isSimulation()) {
            canCoderSim.setVelocity(setpoint.velocity);
            canCoderSim.addPosition(Math.toDegrees(setpoint.velocity)*0.02);
            armSim.setAngle(-Math.toDegrees(getMeasurement())+105);
        }
    }

    @Override
    public double getMeasurement() {
        return Math.toRadians(canCoder.getPosition().getValueAsDouble());
    }

    @Override 
    public void simulationPeriodic() {
        useOutput(super.m_controller.calculate(getMeasurement()), super.m_controller.getSetpoint());
    }
}