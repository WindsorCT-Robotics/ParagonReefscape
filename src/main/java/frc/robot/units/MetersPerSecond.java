package frc.robot.units;

public class MetersPerSecond {
    private final double mps;

    public MetersPerSecond(double meters, double seconds) {
        mps = meters / seconds;
    }

    public MetersPerSecond(double mps) {
        this.mps = mps;
    }

    public double asDouble() {
        return mps;
    }
}
