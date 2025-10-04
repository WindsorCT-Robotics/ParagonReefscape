package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLogOutput;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.units.CANId;
import frc.robot.units.Centimeters;
import frc.robot.units.Meters;
import frc.robot.units.Voltage;
import frc.robot.hardware.IPositionalMotor;

public class NewElevatorSubsystem extends SubsystemBase {
    private final IPositionalMotor motor;
    private static final Voltage gravity = new Voltage(0.4);
    private static final Centimeters LEVEL2_HEIGHT = new Centimeters(81);
    private static final Centimeters LEVEL2_ALGAE_HEIGHT = new Centimeters();
    private static final Centimeters LEVEL3_HEIGHT = new Centimeters(121);
    private static final Centimeters LEVEL1_HEIGHT = new Centimeters(46);

    public NewElevatorSubsystem(IPositionalMotor motor) {
        this.motor = motor;
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Is Fully Extended?", isExtended());
        SmartDashboard.putBoolean("Is Fully Retracted?", isRetracted());
    }

    public void stop() {
        motor.stop();
    }

    public void powerOff() {
        motor.powerOff();
    }

    @AutoLogOutput(key = "Elevator/IsRetracted")
    public boolean isRetracted() {
        if (motor.isAtReverseLimit() || motor.getPosition() == )
    }

    @AutoLogOutput(key = "Elevator/IsExtended")
    public boolean isExtended() {
        return forwardLimitSwitch.isPressed();
    }
}
