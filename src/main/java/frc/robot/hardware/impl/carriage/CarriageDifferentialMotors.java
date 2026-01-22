package frc.robot.hardware.impl.carriage;

import edu.wpi.first.units.measure.Dimensionless;
import frc.robot.hardware.IDifferentialMotors;
import frc.robot.hardware.IDutyRPMMotor;
import frc.robot.hardware.IRPMMotor;

import static edu.wpi.first.units.Units.Percent;

public class CarriageDifferentialMotors implements IDifferentialMotors {
    private final IDutyRPMMotor rightMotor;
    private final IDutyRPMMotor leftMotor;
    private static final Dimensionless SPEED_DIFFERENTIAL = Percent.of(25);

    public CarriageDifferentialMotors(IDutyRPMMotor rightMotor, IDutyRPMMotor leftMotor) {
        this.rightMotor = rightMotor;
        this.leftMotor = leftMotor;
    }
    
    @Override
    public void moveRight(Dimensionless duty) {
        rightMotor.setDuty(duty.times(SPEED_DIFFERENTIAL));
        leftMotor.setDuty(duty);
    }

    @Override
    public void moveLeft(Dimensionless duty) {
        rightMotor.setDuty(duty);
        leftMotor.setDuty(duty.times(SPEED_DIFFERENTIAL));
    }

    @Override
    public void move(Dimensionless duty) {
        rightMotor.setDuty(duty);
        leftMotor.setDuty(duty);
    }

    @Override
    public IRPMMotor getLeftMotor() {
        return leftMotor;
    }

    @Override
    public IRPMMotor getRightMotor() {
        return rightMotor;
    }

    @Override
    public boolean isAtForwardLimit() {
        return false;
    }

    @Override
    public boolean isAtReverseLimit() {
        return false;
    }

    @Override
    public boolean isAtLeftLimit() {
        return false;
    }

    @Override
    public boolean isAtRightLimit() {
        return false;
    }

    @Override
    public void resetRelativeEncoder() {
        rightMotor.resetRelativeEncoder();
        leftMotor.resetRelativeEncoder();
    }

    @Override
    public void stop() {
        leftMotor.stop();
        rightMotor.stop();
    }
}
