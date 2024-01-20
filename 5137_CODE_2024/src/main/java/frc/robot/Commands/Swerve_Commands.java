package frc.robot.Commands;

import frc.robot.Subsystems.Swerve;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class Swerve_Commands {
    Swerve swerve;

    public Swerve_Commands(Swerve swerve) {
        this.swerve = swerve;
    }

    public InstantCommand enableFieldRelative() {
        return new InstantCommand(() -> swerve.setFieldRelative(true), swerve);
    }

    public InstantCommand disableFieldRelative() {
        return new InstantCommand(() -> swerve.setFieldRelative(false), swerve);
    }
}
