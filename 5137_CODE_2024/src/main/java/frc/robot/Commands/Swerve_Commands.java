package frc.robot.Commands;

import frc.robot.Constants.Swerve_Constants;
import frc.robot.Subsystems.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class Swerve_Commands {
    private Swerve swerve;

    public Swerve_Commands(Swerve swerve) {
        this.swerve = swerve;
    }

    public InstantCommand drive(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier rotation, BooleanSupplier fieldRelative) {
        return new InstantCommand(
            () -> swerve.drive(
                new Translation2d(translationX.getAsDouble(), translationY.getAsDouble()).times(Swerve_Constants.maxModuleSpeed), 
                rotation.getAsDouble()*Swerve_Constants.maxAngularSpeed, 
                fieldRelative.getAsBoolean()),
            swerve);
    }

    public FunctionalCommand aimAtTarget() {
        return new FunctionalCommand(() -> {}, () -> swerve.aimAtTarget(), (Boolean x) -> {}, () -> swerve.robotAligned(), swerve);
    }

    public InstantCommand zeroGyro() {
        return new InstantCommand(() -> swerve.zeroGyro(), swerve);
    }

    public Command runAuto(String name) {
        return swerve.getAuto(name);
    }
}
