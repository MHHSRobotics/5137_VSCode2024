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
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class Swerve_Commands {
    private Swerve swerve;
    private Vision vision;

    public Swerve_Commands(Swerve swerve, Vision vision) {
        this.swerve = swerve;
        this.vision = vision;
    }

    public InstantCommand drive(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier rotation, BooleanSupplier fieldRelative) {
        return new InstantCommand(
            () -> swerve.drive(
                new Translation2d(translationX.getAsDouble(), translationY.getAsDouble()).times(Swerve_Constants.maxModuleSpeed), 
                rotation.getAsDouble()*Swerve_Constants.maxAngularSpeed, 
                fieldRelative.getAsBoolean()),
            swerve);
    }

    public FunctionalCommand aimAtSpeaker() {
        return new FunctionalCommand(() -> {}, () -> swerve.aimAtSpeaker(), (Boolean x) -> {}, () -> swerve.turnAligned(), swerve);
    }

    public FunctionalCommand aimAtNote() {
        return new FunctionalCommand(() -> {}, () -> swerve.turnToNote(vision.getTranslationToNote()), (Boolean x) -> {}, () -> swerve.turnAligned(), swerve);
    }
     
    public FunctionalCommand driveToNote() {
        return new FunctionalCommand(() -> {}, () -> swerve.driveToNote(vision.getTranslationToNote(), vision.getMetersToNote()), (Boolean x) -> {}, () -> swerve.driveComplete(), swerve);
    }

    public SequentialCommandGroup autoNote()
    {
        return new SequentialCommandGroup(aimAtNote(), driveToNote());
    }

    public InstantCommand zeroGyro() {
        return new InstantCommand(() -> swerve.zeroGyro(), swerve);
    }

    public Command runAuto() {
        return swerve.getAuto();
    }

}
