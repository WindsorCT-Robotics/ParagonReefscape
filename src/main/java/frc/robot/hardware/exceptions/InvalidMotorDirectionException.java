package frc.robot.hardware.exceptions;

public class InvalidMotorDirectionException extends RuntimeException {
    public InvalidMotorDirectionException() {
        super("Motor must run in forward or reverse.");
    }
}
