package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase{
    
    private CANSparkMax leftMotor = new CANSparkMax(2, MotorType.kBrushless);
    private CANSparkMax rightMotor = new CANSparkMax(5, MotorType.kBrushless);

    public Shooter(){

    }

    public void shoot(double speed)
    {
        leftMotor.set(speed);
        rightMotor.set(speed);
    }

    public void stop()
    {
        leftMotor.set(0);
        rightMotor.set(0);
    }
}
