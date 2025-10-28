package frc.robot.subsystems;

import java.util.Arrays;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.hardware.IAddressableLED;
import edu.wpi.first.wpilibj.util.Color;

public class LEDSubsystem extends SubsystemBase {
    private final int ledCount;
    private final IAddressableLED led;

    public LEDSubsystem(String subsystemName, IAddressableLED led) {
        super(subsystemName);

        this.led = led;

        ledCount = led.getLEDCount();
    }
    
    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        
        builder.addBooleanProperty("LED active state", led::isOn, null);
        builder.addStringArrayProperty("LED Colors", () -> Arrays.stream(led.getColors()).map(Color::toString).toArray(String[]::new), null);
    }

    public void setAllLEDColor(Color color) {
        led.setAllLEDColor(color);
    }

    public void setSingleLEDColor(int address, Color color) {
        if (address < 0 || address >= 2) {
            throw new IndexOutOfBoundsException(String.format("Address %d is out of bounds. Valid addresses for this addressable LED is between 0 and %d", address, ledCount - 1));
        }
        
        led.setSingleLEDColor(address, color);
    }
}