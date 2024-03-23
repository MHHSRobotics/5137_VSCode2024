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
    private boolean objectIn;

    public LED() { // FIRST STRIP: 0-43 SECOND STRIP: 44-94 THIRD STRIP: 95 - 136
        leds = new AddressableLED(LED_Constants.LEDport);
        leds.setLength(136);
        buffer = new AddressableLEDBuffer(LED_Constants.LEDlength);
        offset = 0;
        length = buffer.getLength();
        leds.start();
        timer = new Timer();
        timer.restart();
        objectIn = false;
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
            if (i < LED_Constants.LED1 || i > (LED_Constants.LED2-1)) {
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
            if (i < LED_Constants.LED1) {
                var x = (double) (i+Math.floor(length-offset))%12;
                if (DriverStation.getAlliance().isPresent() ? DriverStation.getAlliance().get() == DriverStation.Alliance.Red : false) {
                    buffer.setRGB(i, (int)((x/11)*255), 0, 0);
                } else {
                    buffer.setRGB(i, 0, 0, (int)((x/11)*255));
                }
            } else if (i < LED_Constants.LED2) {
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

    public void GreenChasingUp() {
        for (int i = 0; i < length; i++) {
            if (i < LED_Constants.LED1) {
                var x = (double) (i+Math.floor(length-offset))%12;
                if (DriverStation.getAlliance().isPresent() ? DriverStation.getAlliance().get() == DriverStation.Alliance.Red : false) {
                    buffer.setRGB(i, (int)((x/11)*255), 0, 0);
                } else {
                    buffer.setRGB(i, 0, 0, (int)((x/11)*255));
                }
            } else if (i < LED_Constants.LED2) {
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

    public void GreenChasingDown() {
        for (int i = 0; i < length; i++) {
            if (i < LED_Constants.LED1) {
                var x = (double) -(i+Math.floor(offset))%-12;
                if (DriverStation.getAlliance().isPresent() ? DriverStation.getAlliance().get() == DriverStation.Alliance.Red : false) {
                    buffer.setRGB(i, (int)((x/11)*255), 0, 0);
                } else {
                    buffer.setRGB(i, 0, 0, (int)((x/11)*255));
                }
            } else if (i < LED_Constants.LED2) {
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

    public void AllianceColorChasingDown() {
        for (int i = 0; i < length; i++) {
            if (i < LED_Constants.LED1) {
                var x = (double) -(i+Math.floor(offset))%-12;
                if (DriverStation.getAlliance().isPresent() ? DriverStation.getAlliance().get() == DriverStation.Alliance.Red : false) {
                    buffer.setRGB(i, (int)((x/11)*255), 0, 0);
                } else {
                    buffer.setRGB(i, 0, 0, (int)((x/11)*255));
                }
            } else if (i < LED_Constants.LED2) {
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

    public void setObjectIn(boolean objectIn) {
        this.objectIn = objectIn;
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
                if (objectIn) {
                    solidColor(Color.kGreen, 255);
                } else {
                    AllianceColorChasingUp();
                }
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
