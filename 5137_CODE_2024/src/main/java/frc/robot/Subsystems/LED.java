package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LED_Constants;
public class LED extends SubsystemBase {

     AddressableLED m_led = new AddressableLED(0);

        AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(60);
        
        int offset= 0;
       
    public LED() {
         m_led.setLength(m_ledBuffer.getLength());
        m_led.setData(m_ledBuffer);
        m_led.start();

        for (var i = 0; i < m_ledBuffer.getLength(); i++){
            m_ledBuffer.setRGB(i, 255, 0, 0);
        }
        m_led.setData(m_ledBuffer);
    
    }
    public void rainbow() {
        for (var i=0; i < m_ledBuffer.getLength(); i++) {
            final var hue = (offset + (i * 180 / m_ledBuffer.getLength())) % 180;
            m_ledBuffer.setHSV(i, hue, 255, 128);
        }

        offset+=3;
        offset%=180;
    }
   
}
