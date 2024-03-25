package frc.robot.Subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.Shooter_Constants;
import static edu.wpi.first.units.Units.*;
import edu.wpi.first.units.MutableMeasure;
import static edu.wpi.first.units.MutableMeasure.mutable;


public class Shooter extends SubsystemBase {
    
    
    private CANSparkMax lowerMotor;
    private CANSparkMax upperMotor;
    private RelativeEncoder lowerEncoder;
    private RelativeEncoder upperEncoder;


    private PIDController lowerPID = new PIDController(Shooter_Constants.lowerKP, Shooter_Constants.lowerKI, Shooter_Constants.lowerKD);
    private PIDController upperPID = new PIDController(Shooter_Constants.upperKP, Shooter_Constants.upperKI, Shooter_Constants.upperKD);

    private SimpleMotorFeedforward lowerFeedForward = new SimpleMotorFeedforward(Shooter_Constants.lowerKS, Shooter_Constants.lowerKV);
    private SimpleMotorFeedforward upperFeedForward = new SimpleMotorFeedforward(Shooter_Constants.upperKS, Shooter_Constants.upperKV);


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
                log.motor("lower-shooter-motor")
                .voltage(
                    m_appliedVoltage.mut_replace(
                        RobotController.getBatteryVoltage()*lowerMotor.getAppliedOutput(), Volts))
                        .angularPosition(m_distance.mut_replace(lowerEncoder.getPosition(), Rotation))
                        .angularVelocity(m_velocity.mut_replace(lowerEncoder.getVelocity(), RotationsPerSecond));
                },
            this
        ));


    public Shooter(){
        lowerMotor = new CANSparkMax(Shooter_Constants.lowerMotorID, MotorType.kBrushless);
        upperMotor = new CANSparkMax(Shooter_Constants.upperMotorID, MotorType.kBrushless);
        lowerEncoder = lowerMotor.getEncoder();
        upperEncoder = lowerMotor.getEncoder();

        lowerMotor.setSmartCurrentLimit(Shooter_Constants.maxSupplyCurrent);
        lowerMotor.setIdleMode(IdleMode.kBrake);

        upperMotor.setSmartCurrentLimit(Shooter_Constants.maxSupplyCurrent);
        upperMotor.setIdleMode(IdleMode.kBrake);
    }

    public void shoot(double desiredRPM) {
        lowerMotor.setVoltage(lowerPID.calculate(lowerEncoder.getVelocity(), desiredRPM) + lowerFeedForward.calculate(desiredRPM));
        upperMotor.setVoltage(upperPID.calculate(upperEncoder.getVelocity(), desiredRPM) + upperFeedForward.calculate(desiredRPM));

    }

    public void setVoltage(double voltage)
    {
        lowerMotor.setVoltage(voltage);
    }

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
    
    public void stop() {
        lowerMotor.set(0);
        upperMotor.set(0);
    }
}
