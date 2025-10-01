package frc.robot.units;

public class Meters {
    private static double centimeterConversionFactor = 100;
    private final double meters;

    public Meters(double meters) {
        this.meters = meters;
    }

    public Meters(Centimeters centimeters) {
        this.meters = centimeters.asDouble() / centimeterConversionFactor;
    }

    public double asDouble() { return meters; }
    public Centimeters asCentimeters() { return new Centimeters(meters * centimeterConversionFactor); }
}
