package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase {

    private AddressableLED leds;
    private AddressableLEDBuffer buffer;

    private int speed;
    private int offset;
    private Timer timer;
    
    public LED() {
        leds = new AddressableLED(8);
        leds.setLength(240);
        buffer = new AddressableLEDBuffer(240);
        offset = 0;
        speed = 0;
        leds.start();
        timer = new Timer();
        timer.restart();
    }

    public void solidColor(Color color, int brightness) {
        for (int i = 0; i < buffer.getLength(); i++) {
            buffer.setRGB(i,
                (int)(brightness*color.red),
                (int)(brightness*color.green),
                (int)(brightness*color.blue));
        }
        leds.setData(buffer);
    }

    public void pulsing(Color color) {
        speed = 1;
        if (offset < 50) {
            solidColor(color, offset*5);
        } else if (offset < 100) {
            solidColor(color, 250-(offset-50)*5);
        }
        offset %= 100;
    }

    public void rainbow() {
        speed = 3;
        for (int i = 0; i < buffer.getLength(); i++) {
            var hue = (offset + i*180/buffer.getLength())%180;
            buffer.setHSV(i, hue, 255, 128);
        }
        offset %= 180;
        leds.setData(buffer);
    }

    public void pulsingCG() {
        speed = 2;
        if (offset < 50) {
            solidColor(Color.kRed, offset*5);
        } else if (offset < 100) {
            solidColor(Color.kRed, 250-(offset-50)*5);
        } else if (offset < 150) {
            solidColor(Color.kGold, (offset-100)*5);
        } else if (offset < 200) {
            solidColor(Color.kGold, 250-(offset-150)*5);
        }
        offset %= 200;
    }

    public void chasingCG() {
        speed = 1;
        for (int i = 0; i < buffer.getLength(); i++) {
            if (i%24 < 12) {
                buffer.setRGB(i,
                    (int)(255*((offset%12)/12)*Color.kRed.red),
                    (int)(255*((offset%12)/12)*Color.kRed.green),
                    (int)(255*((offset%12)/12)*Color.kRed.blue));
            } else {
                buffer.setRGB(i,
                    (int)(255*((offset%12)/12)*Color.kGold.red),
                    (int)(255*((offset%12)/12)*Color.kGold.green),
                    (int)(255*((offset%12)/12)*Color.kGold.blue));
            }
        }
        leds.setData(buffer);
    }

    
    @Override
    public void periodic() {
        if (timer.hasElapsed(40)) {
            //chasingCG();
            timer.reset();
        } else if (timer.hasElapsed(30)) {
            rainbow();
        } else if (timer.hasElapsed(20)) {
            pulsingCG();
        } else if (timer.hasElapsed(10)) {
            rainbow();
        } else {
            //chasingCG();
            pulsingCG();
        }
        offset += speed; 
    }
}
