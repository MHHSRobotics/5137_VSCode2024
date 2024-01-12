package frc.robot.Subsystems;

import frc.robot.Constants.Arm_Constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.hardware.TalonFX;

public class Arm extends SubsystemBase {
    private static TalonFX leftMotor;
    private static TalonFX rightMotor;

    public Arm() {
        leftMotor = new TalonFX(Arm_Constants.leftMotorID);
        rightMotor = new TalonFX(Arm_Constants.rightMotorID);
    }

    public void run(Boolean direction) {
        if (direction) {
            leftMotor.set(0.5);
            rightMotor.set(0.5);
        } else {
            leftMotor.set(-0.5);
            rightMotor.set(-0.5);
        }
    }

    public void stop() {
        leftMotor.set(0.0);
        rightMotor.set(0.0);
    }
}