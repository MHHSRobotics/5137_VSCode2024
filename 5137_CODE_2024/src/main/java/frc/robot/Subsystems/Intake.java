package frc.robot.Subsystems;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {  

    TalonSRX intakeMotor= new TalonSRX(1);

    private final Ultrasonic ultrasonic = new Ultrasonic(0, 1);
    // Ultrasonic sensors tend to be quite noisy and susceptible to sudden outliers,
  // so measurements are filtered with a 5-sample median filter
    private final MedianFilter m_filter = new MedianFilter(5);

    public Intake() {
        SmartDashboard.putNumber("Object Distance", 1000.0);
        ultrasonic.setEnabled(true);
        Ultrasonic.setAutomaticMode(true);
    }

    public double getDistance()
    {
        double measurement = ultrasonic.getRangeInches();
        double filteredMeasurement = m_filter.calculate(measurement);
        return filteredMeasurement;
    }

    public boolean objectInRange()
    {
        if(getDistance() <= 2)
        {
            return true;
        }
        return false;
    }

    public void set (double speed) {
        intakeMotor.set(TalonSRXControlMode.PercentOutput, speed);
    }
    public void stop () {
        intakeMotor.set(TalonSRXControlMode.PercentOutput, 0.0);
    }
    @Override
    public void periodic() {

        SmartDashboard.putNumber("Object Distance", getDistance());
        SmartDashboard.putBoolean("Object In Range", objectInRange());
    }
}