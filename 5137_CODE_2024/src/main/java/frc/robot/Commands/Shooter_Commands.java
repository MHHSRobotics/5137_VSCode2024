package frc.robot.Commands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.Shooter_Constants;
import frc.robot.Subsystems.Shooter;

public class Shooter_Commands {

    private Shooter shooter;
    private SendableChooser<Double> speedChooser;

    public Shooter_Commands(Shooter shooter) {
        this.shooter = shooter;
        speedChooser = new SendableChooser<Double>();
        speedChooser.addOption("4800", 4800.0);
        speedChooser.addOption("4700", 4700.0);
        speedChooser.addOption("4600", 4600.0);
        speedChooser.addOption("4500", 4500.0);
        speedChooser.addOption("4300", 4300.0);
        speedChooser.addOption("4200", 4200.0);
        speedChooser.addOption("4200", 4200.0);


    }
    
    public InstantCommand shootSpeaker() {
        SmartDashboard.putData(speedChooser);
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
