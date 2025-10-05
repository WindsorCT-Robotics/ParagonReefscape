package frc.robot.units;

import java.util.Objects;

public class Meters {
    private static double centimeterConversionFactor = 100;
    private final double meters;

    public Meters(double meters) {
        this.meters = meters;
    }

    public Meters(Centimeters centimeters) {
        this.meters = centimeters.asDouble() / centimeterConversionFactor;
    }

    public double asDouble() {
        return meters;
    }

    public Centimeters asCentimeters() {
        return new Centimeters(meters * centimeterConversionFactor);
    }

    public Rotations asRotations(GearRatio gearRatio, Radians circumference) {
        // TODO: Double-check to ensure this formula is correct
        return new Rotations((meters / circumference.asDouble()) * gearRatio.asDouble());
    }

    @Override
    public boolean equals(Object obj) {
        if (obj instanceof Meters m) {
            return m.meters == this.meters;
        }
        
        return false;
    }

    @Override
    public int hashCode() {
        return Objects.hashCode(meters);
    }
}
