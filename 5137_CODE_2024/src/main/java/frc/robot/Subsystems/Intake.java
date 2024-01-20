package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {  

    TalonSRX intakeMotor= new TalonSRX(1);
    
    public Intake() {
    }
    
    public void set (double speed) {
        intakeMotor.set(TalonSRXControlMode.PercentOutput, speed);
    }
    public void stop () {
        intakeMotor.set(TalonSRXControlMode.PercentOutput, 0.0);
    }
    @Override
    public void periodic() {
        
    }
}
