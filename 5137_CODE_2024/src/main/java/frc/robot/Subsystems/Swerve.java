package frc.robot.Subsystems;

import frc.robot.Constants.Swerve_Constants;

import java.io.File;
import java.io.IOException;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;

public class Swerve extends SubsystemBase {

    private SwerveDrive swerve;
    
    public Swerve(File directory) {
        try {
        swerve = new SwerveParser(directory).createSwerveDrive(Swerve_Constants.maxVelocity);
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative) {
        swerve.drive(translation, rotation, fieldRelative, false);
    }

    public Pose2d getPose() {
        return swerve.getPose();
    }
}
