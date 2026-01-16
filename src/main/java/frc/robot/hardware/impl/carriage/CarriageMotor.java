package frc.robot.hardware.impl.carriage;

import edu.wpi.first.units.measure.Dimensionless;
import frc.robot.hardware.IDifferentialMotors;
import frc.robot.hardware.IDutyRPMMotor;
import frc.robot.hardware.IRPMMotor;
import frc.robot.hardware.MotorDirection;

import static edu.wpi.first.units.Units.Percent;

public class CarriageMotor implements IDifferentialMotors {
    private final IDutyRPMMotor rightMotor;
    private final IDutyRPMMotor leftMotor;
    private static final Dimensionless SPEED_DIFFERENTIAL = Percent.of(0.25);

    public CarriageMotor(IDutyRPMMotor rightMotor, IDutyRPMMotor leftMotor) {
        this.rightMotor = rightMotor;
        this.leftMotor = leftMotor;
    }
    
    @Override
    public void moveRight(Dimensionless duty) {
        rightMotor.setDuty(duty.times(SPEED_DIFFERENTIAL), MotorDirection.FORWARD);
        leftMotor.setDuty(duty, MotorDirection.FORWARD);
    }

    @Override
    public void moveLeft(Dimensionless duty) {
        rightMotor.setDuty(duty, MotorDirection.FORWARD);
        leftMotor.setDuty(duty.times(SPEED_DIFFERENTIAL), MotorDirection.FORWARD);
    }

    @Override
    public void move(Dimensionless duty, MotorDirection direction) {
        rightMotor.setDuty(duty, direction);
        leftMotor.setDuty(duty, direction);
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
