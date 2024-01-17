package frc.robot.Subsystems;

import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;

public class Dashboard {
    

    
    public Dashboard()
    {
        
        Field2d field = new Field2d();
        
        UsbCamera mainCamera = CameraServer.startAutomaticCapture(0);
        UsbCamera intakeCamera = CameraServer.startAutomaticCapture(1);
        NetworkTableEntry cameraSelection = NetworkTableInstance.getDefault().getTable("").getEntry("CameraSelection");

        SmartDashboard.putData("Robot Field", field);
        
    }

    
}
