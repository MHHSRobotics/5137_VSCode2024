package frc.robot.Subsystems;

import com.ctre.phoenix6.controls.MusicTone;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Music extends SubsystemBase {
    private TalonFX motor1;
    private TalonFX motor2;
    private TalonFX motor3;
    private TalonFX motor4;
    private TalonFX motor5;
    private TalonFX motor6;
    private TalonFX motor7;
    private TalonFX motor8;

    public Music() {
        motor1 = new TalonFX(1);
        motor2 = new TalonFX(2);
        motor3 = new TalonFX(3);
        motor4 = new TalonFX(4);
        motor5 = new TalonFX(5);
        motor6 = new TalonFX(6);
        motor7 = new TalonFX(7);
        motor8 = new TalonFX(8);
    }

    public void playAll(double hz) {
        motor1.setControl(new MusicTone(hz));
        motor2.setControl(new MusicTone(hz));
        motor3.setControl(new MusicTone(hz));
        motor4.setControl(new MusicTone(hz));
        motor5.setControl(new MusicTone(hz));
        motor6.setControl(new MusicTone(hz));
        motor7.setControl(new MusicTone(hz));
        motor8.setControl(new MusicTone(hz));
    }
}
