package frc.robot.Commands;

import frc.robot.Constants.Arm_Constants;
import frc.robot.Constants.Shooter_Constants;
import frc.robot.Subsystems.Arm;
import frc.robot.Subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class Shooter_Commands {

    private Shooter shooter;

    public Shooter_Commands(Shooter shooter) {
        this.shooter = shooter;
    };

    public InstantCommand shootDefault() {
        return new InstantCommand(() -> shooter.shoot(Shooter_Constants.defaultShooterSpeed), shooter);
    }

    public InstantCommand stop() {
        return new InstantCommand(() -> shooter.stop(), shooter);
    }
}
