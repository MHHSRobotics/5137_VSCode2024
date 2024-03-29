package frc.robot.Constants;

public class Arm_Constants {
    
    // Positions
    public static final double intakePosition = Math.toRadians(0);
    public static final double defaultPosition = Math.toRadians(30);
    public static final double trapPosition = Math.toRadians(5);
    public static final double ampPosition = Math.toRadians(90);

    // Hardware IDS
    public static final int leftMotorID = 23;
    public static final int rightMotorID = 24;
    public static final int encoderID = 9;

    // Motor Limits 
    public static final int maxSupplyCurrent = 80;

    // Feedforward Constants
    public static final double kS = 0.092308; // 0.15803 Volts to overcome static friction
    public static final double kG = .50473; // 0.50797 Volts to overcome gravity
    public static final double kV = .072111; // 4.6682 
    public static final double kA = .013706; // 0.35139

    // PID Constants
    public static final double kP = 27.688; // 5.7574
    public static final double kI = 0.0; // 0.0
    public static final double kD = 0.0; // 0.82021

    // Control Constants
    public static final double kShooterVelocity = 11.0; // Move to Speaker Constants
    public static final double kMaxVelocity = 2.0; // Radians per Second
    public static final double kMaxAcceleration = 2.0; // Radians per Second Squared
    public static final double kManualSpeed = 5; // Degrees per Second
    public static final double errorMargin = Math.toRadians(0.5); // Degrees
    public static final double minimumNormalAngle = Math.toRadians(-20); // Degrees
    public static final double maximumNormalAngle = Math.toRadians(360) + minimumNormalAngle; // Degrees - Must be 360 degrees above minimum 
    public static final double normalRangeCenter = (minimumNormalAngle+maximumNormalAngle)/2.0; //Radians

    // Characteristics
    public static final double encoderOffset = 0.96819444;
    public static final double gearRatio = 227.555/1.0; 


}
