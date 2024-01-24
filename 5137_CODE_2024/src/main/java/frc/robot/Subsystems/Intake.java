package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {  

    private TalonSRX intakeMotor;
    
    public Intake() {
        intakeMotor = new TalonSRX(1);
    }
    
    public void set(double speed) {
        intakeMotor.set(TalonSRXControlMode.PercentOutput, speed);
    }

    public void stop() {
        intakeMotor.set(TalonSRXControlMode.PercentOutput, 0.0);
    }
}
