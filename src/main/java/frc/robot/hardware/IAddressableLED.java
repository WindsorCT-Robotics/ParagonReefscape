package frc.robot.hardware;

import edu.wpi.first.wpilibj.util.Color;

/**
 * Provides a common interface for an addressable LED strip.
 */
public interface IAddressableLED {
    /**
     * Gets the number of LEDs that can be addressed.
     * @return The number of LEDs that can be addressed.
     */
    public int getLEDCount();

    /**
     * Gets current color for each LED.
     * @return Current color for each LED.
     */
    public Color[] getColors();
    
    /**
     * Set all LEDs to the same color.
     * @param color The color to set.
     */
    public void setAllLEDColor(Color color);
    
    /**
     * Set a single LED or LED group to the specified color.
     * @param address The LED address to set.
     * @param color The color to set.
     */
    public void setSingleLEDColor(int address, Color color);
    
    /**
     * Turns on the LEDs.
     */
    public void turnOn();

    /**
     * Turns off the LEDs.
     */
    public void turnOff();
    
    /**
     * Gets the output status of the LEDs.
     */
    public boolean isOn();
}
