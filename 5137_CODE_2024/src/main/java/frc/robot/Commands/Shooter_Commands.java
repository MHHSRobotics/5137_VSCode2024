package frc.robot.Commands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.Shooter_Constants;
import frc.robot.Subsystems.Shooter;

public class Shooter_Commands {

    private Shooter shooter;
    private SendableChooser<Double> speedChooser;

    public Shooter_Commands(Shooter shooter) {
        this.shooter = shooter;
        speedChooser = new SendableChooser<Double>();
    }
    
    public InstantCommand shootSpeaker() {
        double speed = (speedChooser.getSelected() != null) ? speedChooser.getSelected() : 0;
        return new InstantCommand(() -> shooter.shoot(speed), shooter);
    }

    public InstantCommand shootIntake() {
        return new InstantCommand(() -> shooter.shoot(Shooter_Constants.ampShooterSpeed), shooter);
    }

    public InstantCommand rest() {
        return new InstantCommand(() -> shooter.shoot(Shooter_Constants.restSpeed), shooter);
    }

    public InstantCommand stop() {
        return new InstantCommand(() -> shooter.stop(), shooter);
    }
}
