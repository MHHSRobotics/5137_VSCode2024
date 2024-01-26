package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {  

    TalonSRX intakeMotor= new TalonSRX(1);
    
    private final Ultrasonic ultrasonic = new Ultrasonic(0, 1);
    // Ultrasonic sensors tend to be quite noisy and susceptible to sudden outliers,
  // so measurements are filtered with a 5-sample median filter
    private final MedianFilter m_filter = new MedianFilter(5);

    public Intake() {
    }
    
    public void set(double speed) {
        intakeMotor.set(TalonSRXControlMode.PercentOutput, speed);
    }

    public void stop() {
        intakeMotor.set(TalonSRXControlMode.PercentOutput, 0.0);
    }
    @Override
    public void periodic() {
        
    }
}
