package frc.robot.Commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Subsystems.Climb;

public class Climb_Commands {
    
    private Climb climb;

    public Climb_Commands(Climb climb){
        this.climb = climb;
    }

    public InstantCommand move(DoubleSupplier output) {
        return new InstantCommand(
            () -> climb.setOuput(output.getAsDouble()),
            climb);
    }
}
