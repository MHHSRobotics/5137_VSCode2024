package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Objects.Note;
import frc.robot.Objects.Note.*;
import frc.robot.Subsystems.Music;

public class Music_Commands {

    private Music music;
    
    public Music_Commands(Music music) {
        this.music = music;
    }

    public InstantCommand BbMajorScale() {
        return new InstantCommand(() -> music.play(
            new Note(NoteName.Bb2, 1),
            new Note(NoteName.C3, 1),
            new Note(NoteName.D3, 1),
            new Note(NoteName.Eb3, 1),
            new Note(NoteName.F3, 1),
            new Note(NoteName.G3, 1),
            new Note(NoteName.A3, 1),
            new Note(NoteName.Bb3, 1)
        ), music);
    }

    public InstantCommand HotCrossBuns() {
        return new InstantCommand(() -> music.play(
            new Note(NoteName.Cx3, 2),
            new Note(NoteName.B2, 2),
            new Note(NoteName.A2, 3),
            new Note(NoteName.rest, 1),
            new Note(NoteName.Cx3, 2),
            new Note(NoteName.B2, 2),
            new Note(NoteName.A2, 3),
            new Note(NoteName.rest, 1),
            new Note(NoteName.A2, 1, Articulation.kStacatto),
            new Note(NoteName.A2, 1, Articulation.kStacatto),
            new Note(NoteName.A2, 1, Articulation.kStacatto),
            new Note(NoteName.A2, 1, Articulation.kStacatto),
            new Note(NoteName.B2, 1, Articulation.kStacatto),
            new Note(NoteName.B2, 1, Articulation.kStacatto),
            new Note(NoteName.B2, 1, Articulation.kStacatto),
            new Note(NoteName.B2, 1, Articulation.kStacatto),
            new Note(NoteName.Cx3, 2),
            new Note(NoteName.B2, 2),
            new Note(NoteName.A2, 3)
        ), music);
    }
}
