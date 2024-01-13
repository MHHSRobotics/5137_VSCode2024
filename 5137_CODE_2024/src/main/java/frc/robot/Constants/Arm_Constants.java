package frc.robot.Constants;

public class Arm_Constants {
    // Positions
    public static final double intakePosition = Math.toRadians(-75.0);
    public static final double ampPosition = Math.toRadians(15.0);
    public static final double speakerPosition = Math.toRadians(-45.0);

    // Hardware IDS
    public static final int leftMotorID = 1;
    public static final int rightMotorID = 2;
    public static final int canCoderID = 3;

    // Feedforward Constants
    public static final double kS = 0.5;
    public static final double kG = 0.43;
    public static final double kV = 1.80;
    public static final double kA = 0.02;

    // PID Constants
    public static final double kP = 1.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;

    // Constants
    public static final double kMaxVelocity = 1.0;
    public static final double kMaxAcceleration = 3.0;
}
