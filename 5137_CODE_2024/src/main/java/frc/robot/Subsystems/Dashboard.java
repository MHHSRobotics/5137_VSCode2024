package frc.robot.Subsystems;

import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;

public class Dashboard {
    

    Arm arm; 
    public Dashboard(Arm arm)
    {
        
        this.arm = arm; 
        Field2d field = new Field2d();
        String joystick0 = DriverStation.getJoystickName(0);
        String joystick1 = DriverStation.getJoystickName(1); 
        
        UsbCamera mainCamera = CameraServer.startAutomaticCapture(0);
        UsbCamera intakeCamera = CameraServer.startAutomaticCapture(1);
        NetworkTableEntry cameraSelection = NetworkTableInstance.getDefault().getTable("").getEntry("CameraSelection");
        

        SmartDashboard.putData("Robot Field", field);
        SmartDashboard.putString("Driver Controller",DriverStation.getJoystickName(0));
        SmartDashboard.putString("Operator Controller", joystick0);
        
    

        
    }

    
}
