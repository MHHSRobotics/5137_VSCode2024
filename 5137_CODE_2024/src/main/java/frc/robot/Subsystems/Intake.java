package frc.robot.Subsystems;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase{

    private final int kUltrasonicPingPort = 0;
    private final int kUltrasonicEchoPort = 1;
    private final Ultrasonic m_ultrasonic = new Ultrasonic(kUltrasonicPingPort, kUltrasonicEchoPort);


  // Ultrasonic sensors tend to be quite noisy and susceptible to sudden outliers,
  // so measurements are filtered with a 5-sample median filter
    private final MedianFilter m_filter = new MedianFilter(5);


    public Intake()
    {
        SmartDashboard.putNumber("Sensor Distance", 500);
    }
    

    @Override
    public void periodic()
    {
        double measurement = m_ultrasonic.getRangeMM();
        double filteredMeasurement = m_filter.calculate(measurement);
        double intMeasurement = (int)filteredMeasurement;
        SmartDashboard.putNumber("Sensor Distance", intMeasurement);
        if(measurement <= 7)
        {
            SmartDashboard.putBoolean("Object Detected", true);
        }
        SmartDashboard.putBoolean("Object Detected", false);

 
        
    }
}
