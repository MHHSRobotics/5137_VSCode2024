package frc.robot.Subsystems;

import frc.robot.Robot;
import frc.robot.Constants.Swerve_Constants;

import java.io.File;
import java.io.IOException;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;

import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;

public class Swerve extends SubsystemBase {

    private SwerveDrive swerve;
    private CommandPS4Controller driver;
    
    private Boolean fieldRelative;

    public Swerve(CommandPS4Controller driver, File directory) {
        this.driver = driver;

        fieldRelative = true;

        try {
        swerve = new SwerveParser(directory).createSwerveDrive(Swerve_Constants.maxVelocity);
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }
    
    @Override
    public void periodic() {
        double x = MathUtil.applyDeadband(driver.getLeftX(), Swerve_Constants.LX_Deadband);
        double y = MathUtil.applyDeadband(-driver.getLeftY(), Swerve_Constants.LY_Deadband);
        double z = MathUtil.applyDeadband(-driver.getRightX(), Swerve_Constants.RX_Deadband);

        swerve.drive(new Translation2d(x, y), z, fieldRelative, false);
    }

    public void setFieldRelative(boolean fieldRelative) {
        if (Robot.isReal()) this.fieldRelative = fieldRelative;
        else System.out.println("Cannot change control mode in simulation.");
    }
}
