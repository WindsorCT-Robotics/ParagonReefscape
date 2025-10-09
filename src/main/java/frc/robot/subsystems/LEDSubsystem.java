package frc.robot.subsystems;

import java.util.Arrays;
import java.util.List;
import java.util.stream.Collector;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.hardware.IAddressableLED;
import frc.robot.units.ColorRGB;

public class LEDSubsystem extends SubsystemBase {
    private final int ledCount;
    private final IAddressableLED led;

    public LEDSubsystem(IAddressableLED led) {
        this.led = led;

        ledCount = led.getLEDCount();
    }
    
    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
    }

    @Override
    public void periodic() {
        ColorRGB[] ledColors;
        String[] ledHexColors;

        SmartDashboard.putBoolean("Are LEDs on?", led.isOn());
        
        ledColors = led.getColors();
        ledHexColors = new String[ledColors.length];

        Arrays.stream(ledColors).map(Object::toString).toList().toArray(ledHexColors);
        
        SmartDashboard.putStringArray("LED Colors", ledHexColors);
    }

    public void setAllLEDColor(ColorRGB color) {
        led.setAllLEDColor(color);
    }

    public void setSingleLEDColor(int address, ColorRGB color) {
        if (address < 0 || address >= 2) {
            throw new IndexOutOfBoundsException(String.format("Address %d is out of bounds. Valid addresses for this addressable LED is between 0 and %d", address, ledCount - 1));
        }
        
        led.setSingleLEDColor(address, color);
    }
}