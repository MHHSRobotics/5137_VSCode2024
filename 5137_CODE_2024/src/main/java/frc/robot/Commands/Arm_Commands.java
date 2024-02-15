package frc.robot.Commands;

import frc.robot.Constants.Arm_Constants;
import frc.robot.Subsystems.Arm;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.*;

public class Arm_Commands {

    private Arm arm;

    public Arm_Commands(Arm arm) {
        this.arm = arm;
    };

    public InstantCommand manualMove(DoubleSupplier translationX) {
        //return new InstantCommand(() -> arm.setGoal(Math.max(0.0, Math.min(0.5*Math.PI, arm.getGoal() + Math.toRadians(0.02*Arm_Constants.kManualSpeed*translationX.getAsDouble())))), arm);
        return new InstantCommand(() -> arm.runManual(translationX.getAsDouble()), arm);
    }

    public FunctionalCommand moveToIntake() {
        return new InstantCommand(() -> arm.setGoal(Arm_Constants.intakePosition), arm);
    }

    public FunctionalCommand moveToSpeaker() {
        return new InstantCommand(() -> arm.setGoal(Arm_Constants.speakerPosition), arm);
    }

    public FunctionalCommand moveToDefault() {
        return new InstantCommand(() -> arm.setGoal(Arm_Constants.defaultPosition), arm);
    }

    public FunctionalCommand moveToAmp() {
        return new InstantCommand(() -> arm.setGoal(Arm_Constants.ampPosition), arm);
    }
}
