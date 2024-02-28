package frc.robot.Commands;

import frc.robot.Subsystems.LED;

public class LED_Commands {

    private LED leds;
   
    public LED_Commands (LED leds) {
        this.leds = leds;
        this.leds.getName(); //Extra line to remove unused object error
    }
}
