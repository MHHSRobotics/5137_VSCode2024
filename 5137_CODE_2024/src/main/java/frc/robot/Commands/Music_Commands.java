package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Subsystems.Music;

public class Music_Commands {

    private Music music;
    
    public Music_Commands(Music music) {
        this.music = music;
    }

    public InstantCommand tuningNote() {
        return new InstantCommand(() -> music.playAll(466.164), music);
    }

    public InstantCommand ConcertD() {
        return new InstantCommand(() -> music.playAll(587.33), music);
    }

    public InstantCommand ConcertF() {
        return new InstantCommand(() -> music.playAll(698.46), music);
    }

    public InstantCommand ConcertA() {
        return new InstantCommand(() -> music.playAll(880), music);
    }
}
