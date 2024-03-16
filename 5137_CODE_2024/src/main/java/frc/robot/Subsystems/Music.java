package frc.robot.Subsystems;

import com.ctre.phoenix6.controls.MusicTone;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Objects.Note;
import frc.robot.Objects.Note.Articulation;

public class Music extends SubsystemBase {

    private double bpm;

    private TalonFX motor1;
    private TalonFX motor2;
    private TalonFX motor3;
    private TalonFX motor4;
    private TalonFX motor5;
    private TalonFX motor6;
    private TalonFX motor7;
    private TalonFX motor8;
    
    public Music() {
        bpm = 120;

        motor1 = new TalonFX(1);
        motor2 = new TalonFX(2);
        motor3 = new TalonFX(3);
        motor4 = new TalonFX(4);
        motor5 = new TalonFX(5);
        motor6 = new TalonFX(6);
        motor7 = new TalonFX(7);
        motor8 = new TalonFX(8);
    }

    public void setBPM(double bpm) {
        this.bpm = bpm;
    }

    public double getBPM() {
        return this.bpm;
    }

    public void set(double hz) {
        motor1.setControl(new MusicTone(hz));
        motor2.setControl(new MusicTone(hz));
        motor3.setControl(new MusicTone(hz));
        motor4.setControl(new MusicTone(hz));
        motor5.setControl(new MusicTone(hz));
        motor6.setControl(new MusicTone(hz));
        motor7.setControl(new MusicTone(hz));
        motor8.setControl(new MusicTone(hz));
    }

    public void play(Note note) {
        set(note.getFrequency());
        try {
            switch (note.getArticulation()) {
                case kStacatto: Thread.sleep(Double.doubleToLongBits(500*(note.getLength()*60/bpm))); break;
                case kTenuto: Thread.sleep(Double.doubleToLongBits(1000*(note.getLength()*60/bpm))); break;
                default: Thread.sleep(Double.doubleToLongBits(950*(note.getLength()*60/bpm)));
            }
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        try {
            switch (note.getArticulation()) {
                case kStacatto: set(0); Thread.sleep(Double.doubleToLongBits(500*(note.getLength()*60/bpm))); break;
                case kTenuto: break;
                default: set(0); Thread.sleep(Double.doubleToLongBits(50*(note.getLength()*60/bpm)));
            }
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    public void play(Note... notes) {
        for (var i = 0; i < notes.length; i++) {
            play(notes[i]);
        }
    }
}
