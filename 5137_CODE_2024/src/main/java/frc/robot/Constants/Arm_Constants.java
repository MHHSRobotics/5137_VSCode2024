package frc.robot.Constants;

public class Arm_Constants {
    // Positions
    public static final double intakePosition = -75.0;
    public static final double ampPosition = 15.0;
    public static final double speakerPosition = -90.0;

    // Hardware IDS
    public static final int leftMotorID = 1;
    public static final int rightMotorID = 2;
    public static final int canCoderID = 3;

    // Feedforward Constants
    public static final double kS = 1.0;
    public static final double kG = 1.0;
    public static final double kV = 1.0;
    public static final double kA = 1.0;

    // PID Constants
    public static final double kP = 1.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;

    // Constants
    public static final double kMaxVelocity = 3.0;
    public static final double kMaxAcceleration = 10.0;
}
