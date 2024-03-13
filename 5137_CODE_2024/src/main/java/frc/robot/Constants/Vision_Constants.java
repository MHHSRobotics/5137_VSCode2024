package frc.robot.Constants;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;

public class Vision_Constants {

    public static final double offsetXAR2 = Units.inchesToMeters(-0.9);
    public static final double offsetYAR2 = Units.inchesToMeters(-4);
    public static final double offsetZAR2 = Units.inchesToMeters(11.0);
    public static final double rollAR2= Units.degreesToRadians(0);
    public static final double pitchAR2 = Units.degreesToRadians(-60);
    public static final double yawAR2 = Units.degreesToRadians(180.48);

    public static final double offsetXAR1 = Units.inchesToMeters(6.5);
    public static final double offsetYAR1 = Units.inchesToMeters(-11.5);
    public static final double offsetZAR1 = Units.inchesToMeters(8.0);
    public static final double rollAR1 = Units.degreesToRadians(0);
    public static final double pitchAR1 = Units.degreesToRadians(-55);
    public static final double yawAR1 = Units.degreesToRadians(200);
    
    public static final double offsetXOBJ = 0;
    public static final double offsetYOBJ = 0;
    public static final double offsetZOBJ = Units.inchesToMeters(9.5);
    public static final double rollOBJ = 0;
    public static final double pitchOBJ = -11;
    public static final double yawOBJ= 0;

    public static final Transform3d robotToAR1 = new Transform3d(offsetXAR1, offsetYAR1, offsetZAR1, new Rotation3d(rollAR1,pitchAR1, yawAR1));
    public static final Transform3d robotToAR2 = new Transform3d(offsetXAR2, offsetYAR2, offsetZAR2, new Rotation3d(rollAR2,pitchAR2, yawAR2));
    public static final Transform3d robotToOBJ = new Transform3d(offsetXOBJ, offsetYOBJ, offsetZOBJ, new Rotation3d(rollOBJ, pitchOBJ, yawOBJ));

    public static final double noteDetectionHeight = 0.0; 
    public static final double notePickupDistance = Units.inchesToMeters(25.0);


    
}
