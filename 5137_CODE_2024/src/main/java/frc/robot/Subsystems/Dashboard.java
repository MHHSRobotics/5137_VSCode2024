package frc.robot.Subsystems;

import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Dashboard {
    
    public Dashboard()
    {

        UsbCamera mainCamera = CameraServer.startAutomaticCapture(0);
        UsbCamera intakeCamera = CameraServer.startAutomaticCapture(1);
        NetworkTableEntry cameraSelection = NetworkTableInstance.getDefault().getTable("").getEntry("CameraSelection");
    }

    
}
