package frc.robot.Objects;

public class Note {

    public enum NoteName {
        rest,
                                                                    A0, Ax0, Bb0, B0,
        C1, Cx1, Db1, D1, Dx1, Eb1, E1, F1, Fx1, Gb1, G1, Gx1, Ab1, A1, Ax1, Bb1, B1,
        C2, Cx2, Db2, D2, Dx2, Eb2, E2, F2, Fx2, Gb2, G2, Gx2, Ab2, A2, Ax2, Bb2, B2,
        C3, Cx3, Db3, D3, Dx3, Eb3, E3, F3, Fx3, Gb3, G3, Gx3, Ab3, A3, Ax3, Bb3, B3,
        C4, Cx4, Db4, D4, Dx4, Eb4, E4, F4, Fx4, Gb4, G4, Gx4, Ab4, A4, Ax4, Bb4, B4,
        C5
    }

    public enum Articulation {
        kStacatto,
        kTenuto
    }

    private NoteName note;
    private double length;
    private Articulation articulation;

    /**
     * Creates a note to be used in a track.
     * @param note Name of the note
     * @param length Length of the note in beats
     */
    public Note(NoteName note, double length) {
        this.note = note;
        this.length = length;
        this.articulation = null;
    }

    /**
     * Creates a note to be used in a track.
     * @param note Name of the note
     * @param length Length of the note in beats
     * @param articulation Articulation of the note
     */
    public Note(NoteName note, double length, Articulation articulation) {
        this.note = note;
        this.length = length;
        this.articulation = articulation;
    }

    public NoteName getNote() {
        return this.note;
    }

    public double getLength() {
        return this.length;
    }

    public Articulation getArticulation() {
        return this.articulation;
    }

    public double getFrequency() {
        switch (this.note) {
            case rest: return 0.00;
            case A0: return 27.500;
            case Ax0: return 29.135;
            case Bb0: return 29.135;
            case B0: return 30.868;
            case C1: return 32.703;
            case Cx1: return 34.648;
            case Db1: return 34.648;
            case D1: return 36.708;
            case Dx1: return 38.891;
            case Eb1: return 38.891;
            case E1: return 41.203;
            case F1: return 43.654;
            case Fx1: return 46.249;
            case Gb1: return 46.249;
            case G1: return 48.999;
            case Gx1: return 51.913;
            case Ab1: return 51.913;
            case A1: return 55.000;
            case Ax1: return 58.270;
            case Bb1: return 58.270;
            case B1: return 61.735;
            case C2: return 65.406;
            case Cx2: return 69.296;
            case Db2: return 69.296;
            case D2: return 73.416;
            case Dx2: return 77.782;
            case Eb2: return 77.782;
            case E2: return 82.407;
            case F2: return 87.307;
            case Fx2: return 92.499;
            case Gb2: return 92.499;
            case G2: return 97.999;
            case Gx2: return 103.83;
            case Ab2: return 103.83;
            case A2: return 110.00;
            case Ax2: return 116.54;
            case Bb2: return 116.54;
            case B2: return 123.46;
            case C3: return 130.81;
            case Cx3: return 138.59;
            case Db3: return 138.59;
            case D3: return 146.83;
            case Dx3: return 155.56;
            case Eb3: return 155.56;
            case E3: return 164.81;
            case F3: return 174.61;
            case Fx3: return 185.00;
            case Gb3: return 185.00;
            case G3: return 196.00;
            case Gx3: return 207.65;
            case Ab3: return 207.65;
            case A3: return 220.00;
            case Ax3: return 233.08;
            case Bb3: return 233.08;
            case B3: return 246.94;
            case C4: return 261.63;
            case Cx4: return 277.18;
            case Db4: return 277.18;
            case D4: return 293.67;
            case Dx4: return 311.13;
            case Eb4: return 311.13;
            case E4: return 329.63;
            case F4: return 349.23;
            case Fx4: return 369.99;
            case Gb4: return 369.99;
            case G4: return 392.00;
            case Gx4: return 415.30;
            case Ab4: return 415.30;
            case A4: return 440.00;
            case Ax4: return 466.16;
            case Bb4: return 466.16;
            case B4: return 493.88;
            case C5: return 523.25;
            default: return 0.00;
        }
    }
}
