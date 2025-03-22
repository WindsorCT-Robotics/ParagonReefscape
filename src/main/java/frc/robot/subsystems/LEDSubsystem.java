package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {
    private static int ledPort = 0;
    private static int ledLength = 2;
    private AddressableLED led;
    private AddressableLEDBuffer ledBuffer;

    public LEDSubsystem() {
        led = new AddressableLED(ledPort);
        ledBuffer = new AddressableLEDBuffer(ledLength);
        led.setLength(ledBuffer.getLength());
        led.start();
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("LED Color", convertColor());
    }

    public void setLedColorAll(int red, int green, int blue) {
        for (var i = 0; i < ledBuffer.getLength(); i++) {
            ledBuffer.setRGB(i, red, green, blue);
        }
        led.setData(ledBuffer);
    }

    public void setLedColor(int red, int green, int blue, int ledIndex) {
        ledBuffer.setRGB(ledIndex, red, green, blue);
        led.setData(ledBuffer);
    }

    public boolean convertColor(){
        if (ledBuffer.getLED(0).toString().equals("#0000FF")){
        return true;
        }
        return false;
    }

    public void checkAprilTags(Limelight limelight) {
        if (limelight.getTagCount() == 1) {
            setLedColor(0, 255, 0, 0);
            setLedColor(255, 0, 0, 1);
        } else if (limelight.getTagCount() > 1) {
            setLedColorAll(0, 255, 0);
        } else {
            setLedColorAll(255, 0, 0);
        }
    }
}