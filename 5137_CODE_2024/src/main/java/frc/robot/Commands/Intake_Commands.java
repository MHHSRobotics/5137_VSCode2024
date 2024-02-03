package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.Intake_Constants;
import frc.robot.Constants.Shooter_Constants;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Shooter;
public class Intake_Commands {

    private Intake intake;
    private Shooter shooter;

    public Intake_Commands(Intake intake, Shooter shooter) {
        this.intake = intake;
        this.shooter = shooter;
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
    public FunctionalCommand continuousIntake() {
        return new FunctionalCommand(()-> intake.set(Intake_Constants.defaultMotorSpeed), () -> {}, (Boolean x) -> intake.stop(), () -> {return intake.objectInRange();}, intake);
    }
        public InstantCommand shootDefault() {
        return new InstantCommand(() -> shooter.shoot(Shooter_Constants.defaultShooterSpeed), shooter);
        }

    public InstantCommand shootAmp() {
        return new InstantCommand(() -> shooter.shoot(Shooter_Constants.ampShooterSpeer), shooter);
    }

    public InstantCommand stop() {
        return new InstantCommand(() -> shooter.stop(), intake);
    }
    public SequentialCommandGroup launch() {
        return new SequentialCommandGroup(shootDefault(),new WaitCommand(1),intakeForward(), new WaitCommand(0.5), toStop(),stop());
    }
    }