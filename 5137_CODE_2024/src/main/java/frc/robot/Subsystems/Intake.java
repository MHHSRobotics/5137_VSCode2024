package frc.robot.Subsystems;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase{

    private final DigitalInput beamSensor = new DigitalInput(0);

    public Intake()
    {
        SmartDashboard.putBoolean("Beam Interrupted", false);
        
    }
    
    public boolean getBeamBreak()
    {
        return !beamSensor.get();
    }

    @Override
    public void periodic()
    {
        SmartDashboard.putBoolean("Beam Interrupted", getBeamBreak());
    }
}
