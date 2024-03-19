package frc.robot.Commands;

import frc.robot.Constants.Swerve_Constants;
import frc.robot.Subsystems.Swerve;
import frc.robot.Subsystems.Vision;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class Swerve_Commands {
    private Swerve swerve;
    private Vision vision;

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

    public InstantCommand alignToSpeaker(boolean align) {
        return new InstantCommand(() -> swerve.setSpeakerAlign(align), swerve);
    }

    public InstantCommand zeroGyro() {
        return new InstantCommand(() -> swerve.zeroGyro(), swerve);
    }

    public Command runAuto() {
        return swerve.getAuto();
    }

    public Command driveToAmp() {
        return new InstantCommand(() -> swerve.driveToAmp());
    }

    public Command driveToTrap() {
        return new InstantCommand(() -> swerve.driveToTrap());
    }

    public FunctionalCommand driveToNote(DoubleSupplier distanceToNote, DoubleSupplier rotationToNote) {
        return new FunctionalCommand(() -> {}, () -> swerve.driveToNote(rotationToNote.getAsDouble(), distanceToNote.getAsDouble()), (Boolean x) -> {}, () -> swerve.driveComplete(), swerve);
    }

}
