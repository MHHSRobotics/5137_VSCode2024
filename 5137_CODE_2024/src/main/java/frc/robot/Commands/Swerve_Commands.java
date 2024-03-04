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
                new Translation2d(
                Math.pow(translationX.getAsDouble(), 3)*Swerve_Constants.maxVelocity,
                Math.pow(translationY.getAsDouble(), 3)*Swerve_Constants.maxVelocity), 
                Math.pow(rotation.getAsDouble(), 3)*Swerve_Constants.maxAngularSpeed, 
                fieldRelative.getAsBoolean()),
            swerve);
    }

    public FunctionalCommand aimAtSpeaker() {
        return new FunctionalCommand(() -> {}, () -> swerve.aimAtSpeaker(), (Boolean x) -> {}, () -> swerve.turnAligned(), swerve);
    }

    public InstantCommand zeroGyro() {
        return new InstantCommand(() -> swerve.zeroGyro(), swerve);
    }

    public Command runAuto() {
        return swerve.getAuto();
    }

}
