package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.Shooter_Constants;
import frc.robot.Subsystems.Shooter;

public class Shooter_Commands {

    private Shooter shooter;

    public Shooter_Commands(Shooter shooter) {
        this.shooter = shooter;
    }
    
    public InstantCommand shootSpeaker() {
        return new InstantCommand(() -> shooter.shoot(Shooter_Constants.defaultShooterSpeed), shooter);
    }

    public InstantCommand shootIntake() {
        return new InstantCommand(() -> shooter.shoot(Shooter_Constants.ampShooterSpeed), shooter);
    }

    public InstantCommand stop() {
        return new InstantCommand(() -> shooter.stop(), shooter);
    }
}
