package frc.robot.Constants;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;

public class Vision_Constants {

    public static final double offsetXAR2 = Units.inchesToMeters(-8.5);
    public static final double offsetYAR2 = Units.inchesToMeters(-2.8);
    public static final double offsetZAR2 = Units.inchesToMeters(11.75);
    public static final double rollAR2= Units.degreesToRadians(0);
    public static final double pitchAR2 = Units.degreesToRadians(-37 ); //-35
    public static final double yawAR2 = Units.degreesToRadians(182.25);

    public static final double offsetXOV1 = Units.inchesToMeters(6.5);
    public static final double offsetYOV1 = Units.inchesToMeters(-11.5);
    public static final double offsetZOV1 = Units.inchesToMeters(7.9);
    public static final double rollOV1 = Units.degreesToRadians(0);
    public static final double pitchOV1 = Units.degreesToRadians(-55);
    public static final double yawOV1 = Units.degreesToRadians(200);

     public static final double offsetXOV2 = Units.inchesToMeters(11);
    public static final double offsetYOV2 = Units.inchesToMeters(11.5);
    public static final double offsetZOV2 = Units.inchesToMeters(8.25);
    public static final double rollOV2 = Units.degreesToRadians(0);
    public static final double pitchOV2 = Units.degreesToRadians(-14);
    public static final double yawOV2 = Units.degreesToRadians(85);
    
    public static final Transform3d robotToOV1 = new Transform3d(offsetXOV1, offsetYOV1, offsetZOV1, new Rotation3d(rollOV1,pitchOV1, yawOV1));
    public static final Transform3d robotToOV2 = new Transform3d(offsetXOV2, offsetYOV2, offsetZOV2, new Rotation3d(rollOV2,pitchOV2, yawOV2));
    public static final Transform3d robotToAR2 = new Transform3d(offsetXAR2, offsetYAR2, offsetZAR2, new Rotation3d(rollAR2,pitchAR2, yawAR2));

    public static final double objectCameraX = Units.inchesToMeters(0);
    public static final double objectCameraY = 0.0;
    public static final double objectCameraZ = Units.inchesToMeters(10);
    public static final double objectCameraRoll = 0.0;
    public static final double objectCameraPitch = Math.toRadians(-5);
    public static final double objectCameraYaw = 0.0;
    public static final Transform3d objectCameraToRobot = new Transform3d(objectCameraX, objectCameraY, objectCameraZ, new Rotation3d(objectCameraRoll, objectCameraPitch, objectCameraYaw));

    public static final double noteDetectionHeight = 0.0; 
    public static final double notePickupDistance = Units.inchesToMeters(0.0);

    public static final double objectDistanceKP = 1.0; 
    public static final double objectDistanceKI = 0.0; 
    public static final double objectDistanceKD = 0.0; 

    public static final double objectYawKP = 2.0; 
    public static final double objectYawKI = 0.0; 
    public static final double objectYawKD = 0.0; 

    //TODO: Change for real robot
    public static final double allowableDistanceError = .65;
    public static final double allowableRotationError = Units.degreesToRadians(3);

    
}
