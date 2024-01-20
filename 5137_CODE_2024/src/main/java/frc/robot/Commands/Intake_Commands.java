package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.Intake_Constants;
import frc.robot.Subsystems.Intake;

public class Intake_Commands {
    private Intake intake;

    public Intake_Commands(Intake intake) {
        this.intake = intake;
    }
    
    public InstantCommand toStop() {
        return new InstantCommand(()-> intake.stop(), intake);
    }
    
    public InstantCommand intakeForward() {
        return new InstantCommand(()->intake.set(Intake_Constants.defaultMotorSpeed),intake);
    }


    public InstantCommand intakeReverse() {
        return new InstantCommand(()->intake.set(-Intake_Constants.defaultMotorSpeed),intake);
    }
}