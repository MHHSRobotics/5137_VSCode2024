package frc.robot.Commands;

import frc.robot.Subsystems.Arm;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class Arm_Commands {

    private Arm arm;

    public Arm_Commands(Arm arm) {
        this.arm = arm;
    };

    public InstantCommand moveForward() {
        return new InstantCommand(() -> arm.run(true), arm);
    }

    public InstantCommand moveBackward() {
        return new InstantCommand(() -> arm.run(false), arm);
    }

    public InstantCommand stopMoving() {
        return new InstantCommand(() -> arm.stop(), arm);
    }
}
