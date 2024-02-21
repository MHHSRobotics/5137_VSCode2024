package frc.robot.Subsystems;

import java.util.Optional;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.LED_Constants;

public class LED extends SubsystemBase {

    private AddressableLED leds;
    private AddressableLEDBuffer buffer;

    private double offset;
    private int length;
    private Timer timer;
    
    public LED() {
        leds = new AddressableLED(LED_Constants.LEDport);
        leds.setLength(144);
        buffer = new AddressableLEDBuffer(144);
        offset = 0;
        length = buffer.getLength();
        leds.start();
        timer = new Timer();
        timer.restart();
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
            var x = (double) i%24;
            if (x < 12) {
                buffer.setRGB((i+(int)Math.floor(offset))%length, (int)((x/11)*255), 0, 0);
            } else {
                buffer.setRGB((i+(int)Math.floor(offset))%length, (int)(((x-12)/11)*255), (int)(((x-12)/11)*90), 0);
            }
        }
        leds.setData(buffer);

        offset+=0.3;
    }

    public void AllianceColorChasingUp() {
        for (int i = 0; i < length; i++) {
            var x = (double) i%12;
            Optional<Alliance> ally = DriverStation.getAlliance();
            if (ally.isPresent()) {
                if (ally.get() == Alliance.Red) {
                    buffer.setRGB((i+(int)Math.floor(offset))%length, (int)((x/11)*255), 0, 0);
                } else {
                    buffer.setRGB((i+(int)Math.floor(offset))%length, 0, 0, (int)((x/11)*255));
                }
            }
        }
        leds.setData(buffer);

        offset+=0.5;
    }

    public void AllianceColorChasingDown() {
        for (int i = 0; i < length; i++) {
            var x = (double) -i%-12;
            Optional<Alliance> ally = DriverStation.getAlliance();
            if (ally.isPresent()) {
                if (ally.get() == Alliance.Red) {
                    buffer.setRGB((i+(int)Math.floor(offset))%length, (int)((x/11)*255), 0, 0);
                } else {
                    buffer.setRGB((i+(int)Math.floor(offset))%length, 0, 0, (int)((x/11)*255));
                }
            }
        }
        leds.setData(buffer);

        offset+=(length-0.5);
    }

    @Override
    public void periodic() {
        if (RobotState.isDisabled()) {
            CGChasing();
        } else {


            if (RobotState.isAutonomous()) {
                AllianceColorChasingDown();
            }

            if (RobotState.isTeleop()) {
                AllianceColorChasingUp();
            }

            if (RobotState.isTest()) {
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
