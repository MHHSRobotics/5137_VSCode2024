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
        return new InstantCommand(() -> shooter.shoot(Shooter_Constants.speakerSpeed), shooter);
    }

    public InstantCommand shootTrap() {
        return new InstantCommand(() -> shooter.shoot(Shooter_Constants.trapSpeed), shooter);
    }

    public InstantCommand shootAmp() {
        return new InstantCommand(() -> shooter.shoot(Shooter_Constants.ampSpeed), shooter);
    }

    public InstantCommand pass() {
        return new InstantCommand(() -> shooter.shoot(Shooter_Constants.passSpeed), shooter);
    }

    public InstantCommand rest() {
        return new InstantCommand(() -> shooter.shoot(Shooter_Constants.restSpeed), shooter);
    }

    public InstantCommand stop() {
        return new InstantCommand(() -> shooter.stop(), shooter);
    }
}
