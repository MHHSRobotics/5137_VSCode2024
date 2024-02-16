package frc.robot.Commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Subsystems.Shooter;

public class Shooter_Commands {

    private Shooter shooter;

    public Shooter_Commands(Shooter shooter) {
        this.shooter = shooter;
    }
    
    public InstantCommand shoot(DoubleSupplier armAngle) {
        return new InstantCommand(() -> shooter.shoot(armAngle.getAsDouble()), shooter);
    }

    public InstantCommand stop() {
        return new InstantCommand(() -> shooter.stop(), shooter);
    }
}
