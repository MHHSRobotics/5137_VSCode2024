package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Climb_Constants;

public class Climb extends SubsystemBase{

    private CANSparkMax winchMotor;
    private DigitalInput limitSwitch;
   
    public Climb(){
        winchMotor = new CANSparkMax(Climb_Constants.motorPort, MotorType.kBrushless);
        winchMotor.setInverted(false);
        winchMotor.setIdleMode(IdleMode.kBrake);

        limitSwitch = new DigitalInput(Climb_Constants.limitSwitchPort);
    }

    public void setOuput(double output){
        System.out.println(output);
        if(limitSwitch.get()){
            double speed = (output > 0) ? 0 : output;
            winchMotor.set(speed);
        }
        else{
            winchMotor.set(output);
        }
    }

    
    
}
