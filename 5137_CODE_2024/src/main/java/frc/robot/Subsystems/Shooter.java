package frc.robot.Subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
private CANSparkMax lowerMotor = new CANSparkMax(14, MotorType.kBrushless);
    private CANSparkMax higherMotor = new CANSparkMax(15, MotorType.kBrushless);

    public Shooter(){
        lowerMotor.setIdleMode(IdleMode.kCoast);
        higherMotor.setIdleMode(IdleMode.kCoast);
    }

    public void shoot(double speed)
    {
        lowerMotor.set(speed);
        higherMotor.set(speed);
    }

    public void stop()
    {
        lowerMotor.set(0);
        higherMotor.set(0);
    }
}
