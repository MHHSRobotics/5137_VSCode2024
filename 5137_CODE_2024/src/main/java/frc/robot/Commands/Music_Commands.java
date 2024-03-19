package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Subsystems.Music;

public class Music_Commands {

    private Music music;
    
    public Music_Commands(Music music) {
        this.music = music;
    }

    public InstantCommand toggleMusic() {
        return new InstantCommand(() -> music.togglePlaying(), music);
    }
}
