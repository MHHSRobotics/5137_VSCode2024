package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase{

    private AnalogInput ultrasonicSensor = new AnalogInput(0);
    private DigitalOutput ultrasonicTrigger = new DigitalOutput(0);
    private double ultrasonicSensorRange = 0;
    private double voltageScaleFactor = 1;


    public Intake()
    {
        SmartDashboard.putNumber("Sensor Distance", 500);
        ultrasonicTrigger.set(true);
    }
    

    @Override
    public void periodic()
    {
        SmartDashboard.putNumber("Sensor Distance", ultrasonicSensorRange);
 
        voltageScaleFactor = 5/RobotController.getVoltage5V();
        ultrasonicSensorRange = ultrasonicSensor.getValue()*voltageScaleFactor*0.125;
    }
}
