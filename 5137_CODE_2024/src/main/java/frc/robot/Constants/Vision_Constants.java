package frc.robot.Constants;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

public class Vision_Constants {
    
    public static final Transform3d robotToAR1 = new Transform3d(0.0, 0.0, 0.0, new Rotation3d());
    public static final Transform3d robotToAR2 = new Transform3d(0.0, 0.0, 0.0, new Rotation3d(0,0,180));

    public static final double yawAR1 = 180;
    public static final double yawAR2 = 180;

    public static final double pitchAR1 = 30;
    public static final double pitchAR2 = 30;

    public static final double rollAR1 = 0;
    public static final double rollAR2 = 0;

    public static final double offsetXAR1 = -9.0;
    public static final double offsetYAR1 = -3.5;
    public static final double offsetXAR2 = -9.0;
    public static final double offsetYAR2 = -3.5;
    
}
