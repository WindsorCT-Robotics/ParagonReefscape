package frc.robot.units;

public class RadiansPerSecond {
    private final double rps;

    public RadiansPerSecond(double radians, double seconds) {
        rps = radians / seconds;
    }

    public RadiansPerSecond(double rps) {
        this.rps = rps;
    }

    public double asDouble() {
        return rps;
    }
}
