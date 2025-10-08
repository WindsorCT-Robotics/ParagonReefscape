package frc.robot.units;

public class ColorRGB {
    private final short red;
    private final short green;
    private final short blue;
    
    private boolean isInRange(short val) {
        return (val >= 0 && val <= 255);
    }

    /**
     * Describes a color in terms of its red, green, and blue components.
     * @param red The red component. Must be within range 0 to 255.
     * @param green The green component. Must be within range 0 to 255.
     * @param blue The blue component. Must be within range 0 to 255.
     */
    public ColorRGB(short red, short green, short blue) {
        if (!isInRange(red)) {
            throw new IllegalArgumentException(String.format("The value for red must be within the range from 0 to 255. Value received: %d", red));
        }

        if (!isInRange(green)) {
            throw new IllegalArgumentException(String.format("The value for green must be within the range from 0 to 255. Value received: %d", green));
        }

        if (!isInRange(blue)) {
            throw new IllegalArgumentException(String.format("The value for blue must be within the range from 0 to 255. Value received: %d", blue));
        }

        this.red = red;
        this.green = green;
        this.blue = blue;
    }

    /**
     * Gets the red component of the color.
     * @return The red component of the color.
     */
    public short getRed() {
        return red;
    }

    /**
     * Gets the green component of the color.
     * @return The green component of the color.
     */
    public short getGreen() {
        return green;
    }

    /**
     * Gets the blue component of the color.
     * @return The blue component of the color.
     */
    public short getBlue() {
        return blue;
    }
    
    @Override
    public String toString() {
        StringBuilder sb = new StringBuilder();

        sb.append("R = ");
        sb.append(red);
        sb.append("; G = ");
        sb.append(green);
        sb.append("; B = ");
        sb.append(blue);
        
        return sb.toString();
    }
}
