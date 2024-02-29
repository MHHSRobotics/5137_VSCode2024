package frc.robot.Subsystems;


import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LED_Constants;

public class LED extends SubsystemBase {

    private AddressableLED leds;
    private AddressableLEDBuffer buffer;

    private double offset;
    private int length;
    private Timer timer;
    
    public LED() { // FIRST STRIP: 0-43 SECOND STRIP: 44-103 THIRD STRIP: 104 - 145
        leds = new AddressableLED(LED_Constants.LEDport);
        leds.setLength(146);
        buffer = new AddressableLEDBuffer(146);
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
            if (i < 44 || i > 103) {
                var x = (double) (i+Math.floor(length-(offset%length)))%24;
                if (x < 12) {
                    buffer.setRGB(i, (int)((x/11)*255), 0, 0);
                } else {
                    buffer.setRGB(i, (int)(((x-12)/11)*255), (int)(((x-12)/11)*90), 0);
                }
            } else {
                if (timer.hasElapsed(2)) {
                    timer.restart();
                } else if (timer.hasElapsed(1)) {
                    buffer.setRGB(i, 255, 90, 0);
                } else {
                    buffer.setRGB(i, 255, 0, 0);
                }
            }
        }
        leds.setData(buffer);

        offset+=0.3;
    }

    public void AllianceColorChasingUp() {
        for (int i = 0; i < length; i++) {
            if (i < 44) {
                var x = (double) (i+Math.floor(length-offset))%12;
                if (DriverStation.getAlliance().isPresent() ? DriverStation.getAlliance().get() == DriverStation.Alliance.Red : false) {
                    buffer.setRGB(i, (int)((x/11)*255), 0, 0);
                } else {
                    buffer.setRGB(i, 0, 0, (int)((x/11)*255));
                }
            } else if (i < 104) {
                if (DriverStation.getAlliance().isPresent() ? DriverStation.getAlliance().get() == DriverStation.Alliance.Red : false) {
                    buffer.setRGB(i, 255, 0, 0);
                } else {
                    buffer.setRGB(i, 0, 0, 255);
                }    
            } else {
                var x = (double) -(i+Math.floor(offset))%-12;
                if (DriverStation.getAlliance().isPresent() ? DriverStation.getAlliance().get() == DriverStation.Alliance.Red : false) {
                    buffer.setRGB(i, (int)((x/11)*255), 0, 0);
                } else {
                    buffer.setRGB(i, 0, 0, (int)((x/11)*255));
                }
            }
        }
        leds.setData(buffer);

        offset+=0.5;
    }

    public void AllianceColorChasingDown() {
        for (int i = 0; i < length; i++) {
            if (i < 44) {
                var x = (double) -(i+Math.floor(offset))%-12;
                if (DriverStation.getAlliance().isPresent() ? DriverStation.getAlliance().get() == DriverStation.Alliance.Red : false) {
                    buffer.setRGB(i, (int)((x/11)*255), 0, 0);
                } else {
                    buffer.setRGB(i, 0, 0, (int)((x/11)*255));
                }
            } else if (i < 104) {
                if (DriverStation.getAlliance().isPresent() ? DriverStation.getAlliance().get() == DriverStation.Alliance.Red : false) {
                    buffer.setRGB(i, 255, 0, 0);
                } else {
                    buffer.setRGB(i, 0, 0, 255);
                }    
            } else {
                var x = (double) (i+Math.floor(length-offset))%12;
                if (DriverStation.getAlliance().isPresent() ? DriverStation.getAlliance().get() == DriverStation.Alliance.Red : false) {
                    buffer.setRGB(i, (int)((x/11)*255), 0, 0);
                } else {
                    buffer.setRGB(i, 0, 0, (int)((x/11)*255));
                }
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
                    solidColor(Color.kRed, 150);
                }
            }
        }
    }
}
