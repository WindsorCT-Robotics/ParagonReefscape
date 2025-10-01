package frc.robot.units;

public class Radians {
    private final double radians;

    public Radians(double radians) {
        this.radians = radians;
    }

    public Radians(Rotations rotations, double rotationOffset) {
        radians = 2 * Math.PI * rotations.asDouble() + rotationOffset;
    }

    public double asDouble() {
        return radians;
    }

    public Rotations asRotations() {
        return new Rotations(radians / (2 * Math.PI));
    }
}