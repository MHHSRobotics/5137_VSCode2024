package frc.robot.Commands;

import frc.robot.Constants.Arm_Constants;
import frc.robot.Subsystems.Arm;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class Arm_Commands {

    private Arm arm;

    public Arm_Commands(Arm arm) {
        this.arm = arm;
    };

    public InstantCommand moveForward() {
        return new InstantCommand(() -> arm.setGoal(arm.getMeasurement()+0.1), arm);
    }

    public InstantCommand moveBackward() {
        return new InstantCommand(() -> arm.setGoal(arm.getMeasurement()-0.1), arm);
    }

    public InstantCommand moveToIntake() {
        return new InstantCommand(() -> arm.setGoal(0.0), arm);
    }

    public InstantCommand moveToStart() {
        return new InstantCommand(() -> arm.setGoal(Math.toRadians(Arm_Constants.intakePosition)), arm);
    }

    public InstantCommand moveToAmp() {
        return new InstantCommand(() -> arm.setGoal(Math.toRadians(Arm_Constants.ampPosition)), arm);
    }

    public InstantCommand stopMoving() {
        return new InstantCommand(() -> arm.setGoal(arm.getMeasurement()), arm);
    }
}
