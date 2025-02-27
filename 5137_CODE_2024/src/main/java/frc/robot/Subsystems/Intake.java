package frc.robot.Subsystems;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Intake_Constants;

public class Intake extends SubsystemBase {  

    private TalonSRX intakeMotor;
    private Ultrasonic ultrasonic;
    private DigitalInput irSensor0;
    private DigitalInput irSensor1;
    private MedianFilter m_filter;  // Ultrasonic sensors tend to be quite noisy and susceptible to sudden outliers, so measurements are filtered with a 5-sample median filter

    public Intake() {
        SmartDashboard.putNumber("Object Distance", 1000.0);

        ultrasonic = new Ultrasonic(4, 5); //1, 0
        ultrasonic.setEnabled(true);
        Ultrasonic.setAutomaticMode(true);
        m_filter = new MedianFilter(5);
        irSensor0 = new DigitalInput(0);
        irSensor1 = new DigitalInput(1);

        intakeMotor = new TalonSRX(20);
        intakeMotor.setInverted(true);
        intakeMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, Intake_Constants.maxSupplyCurrent, Intake_Constants.maxSupplyCurrent, 0));
    }

    public double getDistance() {
        double measurement = ultrasonic.getRangeInches();
        double filteredMeasurement = m_filter.calculate(measurement);
        return filteredMeasurement;
    }

    public boolean objectInRange(){
        return (/*!irSensor0.get() || */!irSensor1.get());
        /* 
        if(getDistance() <= 10){
            return true;
        }
        return false;
        */
    }

    public void set (double speed) {
        intakeMotor.set(TalonSRXControlMode.PercentOutput, speed);        
    }

    public void stop () {
        intakeMotor.set(TalonSRXControlMode.PercentOutput, 0.0);
        intakeMotor.setNeutralMode(NeutralMode.Brake);
    }

    @Override
    public void periodic() {     
        SmartDashboard.putNumber("Object Distance", getDistance());
        SmartDashboard.putBoolean("Object In Range", objectInRange());
        SmartDashboard.putBoolean("IR 0", !irSensor0.get());
        SmartDashboard.putBoolean("IR 1", !irSensor1.get());
        //System.out.println(irSensor0.get() + "     " + irSensor1.get());
    }
}
