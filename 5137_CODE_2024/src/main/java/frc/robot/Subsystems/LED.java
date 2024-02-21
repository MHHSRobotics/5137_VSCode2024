package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LED_Constants;
public class LED extends SubsystemBase {

     AddressableLED m_led = new AddressableLED(0);

        AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(144);
        
        int offset= 0;
       
    public void LED1() {
        m_led.setLength(m_ledBuffer.getLength());
        m_led.setData(m_ledBuffer);
        m_led.start();

        for (var i = 0; i < m_ledBuffer.getLength(); i++){
            m_ledBuffer.setRGB(i, 138,43, 226);
        }
        m_led.setData(m_ledBuffer);
    
    }
    public void LED2() {
        m_led.setLength(m_ledBuffer.getLength());
        m_led.setData(m_ledBuffer);
        m_led.start();

        for (var i = 0; i < m_ledBuffer.getLength(); i++){
            m_ledBuffer.setRGB(i, 224,33, 138);
        }
        m_led.setData(m_ledBuffer);
    
    }public void LED3() {
        m_led.setLength(m_ledBuffer.getLength());
        m_led.setData(m_ledBuffer);
        m_led.start();

        for (var i = 0; i < m_ledBuffer.getLength(); i++){
            m_ledBuffer.setRGB(i, 32,123, 110);
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
        m_led.setData(m_ledBuffer);
    }
    /* 
      public void greenfade() {
        for (var i=0; i < m_ledBuffer.getLength(); i++) {
            final var hue = 45 + (i*0.3);
            m_ledBuffer.setHSV(i, (int) hue, 255, 128);
        }
        offset+=3;
        offset%=180;
        m_led.setData(m_ledBuffer);
    }
    public void red2Gold() {
        for (var i=0; i < m_ledBuffer.getLength(); i++) {
            final var hue = (offset + (i * 180 / m_ledBuffer.getLength())) % 30;
            m_ledBuffer.setHSV(i, hue, 255, 128);
        }
        offset+=3;
        offset%=180;
        m_led.setData(m_ledBuffer);
    }*/
    //public void redGoldBricks() {
        //int hue = 30;
        //int count= 0;
        //for (var i=0; i < m_ledBuffer.getLength(); i++) {
            //if (i%10==0)
          // For sectional LEDs 
        //}
        offset+=3;
        offset%=180;
        m_led.setData(m_ledBuffer);
    }
   @Override
   public void periodic() {
       super.periodic();
       
   }
public Object LED() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'LED'");
}
}
