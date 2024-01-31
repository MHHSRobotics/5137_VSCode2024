package frc.robot.Constants;

import edu.wpi.first.math.util.Units;

public class Swerve_Constants {
    // Deadbands
    public static final double LX_Deadband = 0.1;
    public static final double LY_Deadband = 0.1;
    public static final double RX_Deadband = 0.1;

    // Physics
    public static final double maxVelocity = Units.feetToMeters(5);
    public static final double maxModuleSpeed = 4.5;
}
