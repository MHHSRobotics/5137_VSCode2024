package frc.robot.Subsystems;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Music extends SubsystemBase {

    private TalonFX[] motors;
    private Orchestra orchesta;
    private String[] songs;
    private SendableChooser<String> songChooser;
    private String currentSongName;
    
    public Music() {
        motors = new TalonFX[] {
            new TalonFX(1),
            new TalonFX(2),
            new TalonFX(3),
            new TalonFX(4),
            new TalonFX(5),
            new TalonFX(6),
            new TalonFX(7),
            new TalonFX(8)
        };

        orchesta = new Orchestra();

        for (var i = 0; i < motors.length; i++) {
            orchesta.addInstrument(motors[i]);
        }

        songs = new String[] {
            "EntryOfTheGladiators.chrp"
        };

        songChooser = new SendableChooser<String>();
        
        for (var i = 0; i < songs.length; i++) {
            songChooser.addOption(songs[i], songs[i]);
            if (i == 0) {
                songChooser.setDefaultOption(songs[i], songs[i]);
                var status = orchesta.loadMusic(songs[i]);
                if (!status.isOK()) {
                    System.out.println(status);
                }
            }
        }

        currentSongName = songs[0];

        SmartDashboard.putData("Song Choice", songChooser);
    }

    public void play() {
        System.out.println("Orchesta Playing");
        orchesta.play();
    }

    public void pause() {
        System.out.println("Orchesta Paused");
        orchesta.pause();
    }

    public void togglePlaying() {
        if (orchesta.isPlaying()) {
            this.pause();
        } else {
            this.play();
        }
    }

    @Override
    public void periodic() {
        if (songChooser.getSelected() != null) {
            if (!songChooser.getSelected().equals(currentSongName)) {
                this.pause();
                orchesta.loadMusic(songChooser.getSelected());
                currentSongName = songChooser.getSelected();
            }
        }
    }
}
