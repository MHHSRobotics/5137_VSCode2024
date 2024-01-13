package frc.robot.Commands;

import frc.robot.Constants.Arm_Constants;
import frc.robot.Subsystems.Arm;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class Arm_Commands {

    private Arm arm;

    public Arm_Commands(Arm arm) {
        this.arm = arm;
    };

    public InstantCommand moveToIntake() {
        return new InstantCommand(() -> arm.setGoal(Arm_Constants.intakePosition), arm);
    }

    public InstantCommand moveToStart() {
        return new InstantCommand(() -> arm.setGoal(0.0), arm);
    }

    public InstantCommand moveToAmp() {
        return new InstantCommand(() -> arm.setGoal(Arm_Constants.ampPosition), arm);
    }

    public InstantCommand moveToSpeaker() {
        return new InstantCommand(() -> arm.setGoal(Arm_Constants.speakerPosition), arm);
    }
}
