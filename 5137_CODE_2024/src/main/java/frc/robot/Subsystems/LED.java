package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase {

    private AddressableLED leds;
    private AddressableLEDBuffer buffer;

    private int offset;
    
    public LED() {
        leds = new AddressableLED(9);
        leds.setLength(144);
        buffer = new AddressableLEDBuffer(144);
        offset = 0;
        leds.start();
    }

    private void setRed() {
        for (int i = 0; i < buffer.getLength(); i++) {
            buffer.setRGB(i, 255, 0, 0);
        }
        leds.setData(buffer);
    }

    private void rainbow() {
        for (int i = 0; i < buffer.getLength(); i++) {
            var hue = (offset + i*180/buffer.getLength())%180;
            buffer.setHSV(i, hue, 255, 128);
        }
        leds.setData(buffer);
        offset += 3;
        offset %= 180;
    }

    @Override
    public void periodic() {
        rainbow();
    }
}
