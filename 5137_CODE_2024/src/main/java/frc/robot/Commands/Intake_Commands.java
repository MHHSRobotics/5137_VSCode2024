package frc.robot.Commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.Intake_Constants;
import frc.robot.Subsystems.Intake;

public class Intake_Commands {

    private Intake intake;
    private Timer timer;

    public Intake_Commands(Intake intake) {
        this.intake = intake;
        timer = new Timer();
        timer.reset();
    }
    
    public InstantCommand stop() {
        return new InstantCommand(() -> intake.stop(), intake);
    }
    
    public InstantCommand intakeForward() {
        return new InstantCommand(() -> intake.set(Intake_Constants.defaultMotorSpeed), intake);
    }

    public FunctionalCommand intakeForward(double delay) {
        return new FunctionalCommand(
            () -> timer.restart(),
            () -> {},
            (Boolean x) -> intake.set(Intake_Constants.defaultMotorSpeed),
            () -> {return timer.hasElapsed(delay);},
            intake);
    }

    public InstantCommand intakeReverse() {
        return new InstantCommand(() -> intake.set(-Intake_Constants.defaultMotorSpeed), intake);
    }

    /*
    public FunctionalCommand continuousIntake() {
        return new FunctionalCommand(
            () -> intake.set(Intake_Constants.defaultMotorSpeed),
            () -> {},
            (Boolean x) -> intake.stop(),
            () -> {return intake.objectInRange();},
            intake);
    }*/
}