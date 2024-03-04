package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Subsystems.LED;

public class LED_Commands {

    private LED leds;
   
    public LED_Commands (LED leds) {
        this.leds = leds;
    }

    public InstantCommand greenLedsOn() {
        return new InstantCommand(() -> leds.setObjectIn(true), leds);
    }

    public InstantCommand greenLedsOff() {
        return new InstantCommand(() -> leds.setObjectIn(false), leds);
    }
}
