package frc.robot.Subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Shooter_Constants;

public class Shooter extends SubsystemBase {
    
    private CANSparkMax lowerMotor = new CANSparkMax(21, MotorType.kBrushless);
    private CANSparkMax upperMotor = new CANSparkMax(22, MotorType.kBrushless);

    private RelativeEncoder upperEncoder;
    private RelativeEncoder lowerEncoder;


    public Shooter(){
        lowerMotor.setSmartCurrentLimit(Shooter_Constants.maxSupplyCurrent);
        upperMotor.setSmartCurrentLimit(Shooter_Constants.maxSupplyCurrent);
        lowerMotor.setIdleMode(IdleMode.kCoast);
        upperMotor.setIdleMode(IdleMode.kCoast);
        upperEncoder = upperMotor.getEncoder();
        lowerEncoder = lowerMotor.getEncoder();


    }

    public void shoot(double speed) {
        lowerMotor.set(speed);
        upperMotor.set(speed);
    }

    public void stop() {
        lowerMotor.set(0);
        upperMotor.set(0);

    }

    public boolean atSpeed(){
        return Math.abs(upperEncoder.getVelocity()) > 5000 && Math.abs(lowerEncoder.getVelocity()) > 5000;
    
    }


    @Override
    public void periodic(){
        SmartDashboard.putNumber(" upper RPM",upperEncoder.getVelocity());
                SmartDashboard.putNumber(" lower RPM",lowerEncoder.getVelocity());

    }
}
