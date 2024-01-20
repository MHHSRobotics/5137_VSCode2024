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

    private final DigitalInput beamSensor = new DigitalInput(2);
    private final Ultrasonic ultrasonic = new Ultrasonic(0, 1);
    // Ultrasonic sensors tend to be quite noisy and susceptible to sudden outliers,
  // so measurements are filtered with a 5-sample median filter
    private final MedianFilter m_filter = new MedianFilter(5);
    

    public Intake()
    {
        SmartDashboard.putBoolean("Beam Interrupted", false);
        SmartDashboard.putNumber("Object Distance", 1000.0);
        SmartDashboard.putBoolean("Object In Range", false);
        ultrasonic.setEnabled(true);
        Ultrasonic.setAutomaticMode(true);
    }
    
    public boolean getBreakBeam()
    {
        return !beamSensor.get();
    }
    
    public double getDistance()
    {
        double measurement = ultrasonic.getRangeInches();
        double filteredMeasurement = m_filter.calculate(measurement);
        return filteredMeasurement;
    }

    public boolean objectInRange()
    {
        if(getDistance() <= 3)
        {
            return true;
        }
        return false;
    }


    @Override
    public void periodic()
    {
        SmartDashboard.putBoolean("Beam Interrupted", getBreakBeam());
        SmartDashboard.putNumber("Object Distance", getDistance());
        SmartDashboard.putBoolean("Object In Range", objectInRange());
    }
}
