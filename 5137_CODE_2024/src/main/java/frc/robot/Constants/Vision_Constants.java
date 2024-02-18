package frc.robot.Constants;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

public class Vision_Constants {

    public static final double offsetXAR1 = -0.9; // 2 and .9
    public static final double offsetYAR1 = -2;
    public static final double offsetZAR1 = 8;
    public static final double rollAR1 = 0;
    public static final double pitchAR1 = 30;
    public static final double yawAR1 = 0;

    public static final double offsetXAR2 = -0.9; // 2 and .9
    public static final double offsetYAR2 = -2;
    public static final double offsetZAR2 = 8;
    public static final double rollAR2 = 0;
    public static final double pitchAR2 = 30;
    public static final double yawAR2 = 0;
    
    public static final double offsetXOBJ = 0;
    public static final double offsetYOBJ = 0;
    public static final double offsetZOBJ = 0;
    public static final double rollOBJ = 0;
    public static final double pitchOBJ = 0;
    public static final double yawOBJ= 0;

    public static final Transform3d robotToAR1 = new Transform3d(offsetXAR1, offsetYAR1, offsetZAR1, new Rotation3d(rollAR1,pitchAR1, yawAR1));
    public static final Transform3d robotToAR2 = new Transform3d(offsetXAR2, offsetYAR2, offsetZAR2, new Rotation3d(rollAR2,pitchAR2, yawAR2));
    public static final Transform3d robotToOBJ = new Transform3d(offsetXOBJ, offsetYOBJ, offsetZOBJ, new Rotation3d(rollOBJ, pitchOBJ, yawOBJ));

    public static final double noteDetectionHeight = 0.0; 


    
}
