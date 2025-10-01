package frc.robot.units;

public class Rotations {
    private final double rotations;

    public Rotations(double rotations) {
        this.rotations = rotations;
    }

    public double asDouble() { return rotations; }

    public Radians asRadians(double offset) { return new Radians(this, offset); }

    public Meters asMeters(GearRatio gearRatio, Radians circumference) {
        return new Meters(rotations / gearRatio.asDouble() * circumference.asDouble());
    }
}