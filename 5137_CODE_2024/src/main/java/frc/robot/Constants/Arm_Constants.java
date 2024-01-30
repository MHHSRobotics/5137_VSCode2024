package frc.robot.Constants;

public class Arm_Constants {
    // Positions
    public static final double intakePosition = Math.toRadians(0);
    public static final double defaultPosition = Math.toRadians(75);
    public static final double ampPosition = Math.toRadians(90);

    // Hardware IDS
    public static final int leftMotorID = 1;
    public static final int rightMotorID = 2;
    public static final int canCoderID = 3;

    //Motor Limits 
    public static final int maxSupplyCurrent = 80;

    // Feedforward Constants
    public static final double kS = 0.5;
    public static final double kG = 0.43;
    public static final double kV = 1.80;
    public static final double kA = 0.02;

    // PID Constants
    public static final double kP = 1.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;

    // Control Constants
    public static final double kShooterVelocity = 11.0; // Move to Speaker Constants
    public static final double kMaxVelocity = 1.0; // Meters per Second
    public static final double kMaxAcceleration = 3.0; // Meters per Second Squared
    public static final double kManualSpeed = 5; // Degrees per Second
    public static final double errorMargin = Math.toRadians(1.0); // Degrees

    // Measurements
    public static final double kGravity = -9.80665; // Meters per Second Squared
    public static final double kTargetY = 2.0125; // Meters
}
