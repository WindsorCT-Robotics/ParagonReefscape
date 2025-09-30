package frc.robot.Units;

public class Centimeters {
    private static double meterConversionFactor = 100;
    private final double centimeters;

    public Centimeters(double centimeters) {
        this.centimeters = centimeters;
    }

    public Centimeters(Meters meters) {
        this.centimeters = meters.asDouble() * meterConversionFactor;
    }

    public double asDouble() { return centimeters; }
    public Meters asMeters() { return new Meters(centimeters / meterConversionFactor); }
}