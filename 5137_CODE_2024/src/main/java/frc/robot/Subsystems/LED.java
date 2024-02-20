package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase {

    private AddressableLED leds;
    private AddressableLEDBuffer buffer;

    private double offset;
    private int length;
    private Timer timer;
    
    public LED() {
        leds = new AddressableLED(8);
        leds.setLength(144);
        buffer = new AddressableLEDBuffer(144);
        offset = 0;
        length = buffer.getLength();
        leds.start();
        timer = new Timer();
        timer.reset();
    }

    public void solidColor(Color color, int brightness) {
        for (int i = 0; i < length; i++) {
            buffer.setRGB(i,
                (int)(brightness*color.red),
                (int)(brightness*color.green),
                (int)(brightness*color.blue));
        }
        leds.setData(buffer);
    }

    public void CGChasing() {
        for (int i = 0; i < length; i++) {
            var x = Math.floor(i%24);
            if (x < 12) {
                buffer.setRGB((i+(int)Math.floor(offset))%length, (int)((x/11)*255), 0, 0);
            } else {
                buffer.setRGB((i+(int)Math.floor(offset))%length, (int)(((x-12)/11)*255), (int)(((x-12)/11)*90), 0);
            }
        }
        leds.setData(buffer);

        offset+=0.5;
    }

    @Override
    public void periodic() {
        if (RobotState.isDisabled()) {
            CGChasing();
        } else {

            if (RobotState.isAutonomous()) {
                
            }

            if (RobotState.isTeleop()) {

            }

            if (RobotState.isTest()) {
                timer.restart();
                if (timer.hasElapsed(2)) {
                    timer.restart();
                } else if (timer.hasElapsed(1)) {
                    solidColor(Color.kBlack, 255);
                } else {
                    solidColor(Color.kGold, 150);
                }
            }
        }
    }
}
