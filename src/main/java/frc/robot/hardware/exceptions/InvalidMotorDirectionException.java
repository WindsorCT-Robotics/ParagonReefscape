package frc.robot.hardware.exceptions;

import frc.robot.hardware.MotorDirection;

public class InvalidMotorDirectionException extends RuntimeException {
    public InvalidMotorDirectionException(MotorDirection invalidDirection) {
        super("Motor must run in forward or reverse. Invalid direction: " + invalidDirection);
    }
}
