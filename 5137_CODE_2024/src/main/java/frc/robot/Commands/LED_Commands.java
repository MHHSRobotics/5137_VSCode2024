package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Subsystems.LED;

public class LED_Commands {
    
    private LED led;

    public LED_Commands(LED led) {
        this.led = led;
        
    }
    
    public InstantCommand rbow() {
        return new InstantCommand(() ->led.red2Gold(),led);
    }
   
    public InstantCommand purpleGerbert(){
        return new InstantCommand(() ->led.LED1(), led);
    }
    
    public InstantCommand barbiePink() {
        return new InstantCommand(() ->led.LED2(), led);
    }
    public InstantCommand deepSage() {
        return new InstantCommand(() ->led.LED3(), led);
    }
}
