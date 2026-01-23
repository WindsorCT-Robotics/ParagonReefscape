package frc.robot.subsystems.algae;

import frc.robot.hardware.IDutyMotor;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeRemoverSubsystem extends SubsystemBase {
    private Dimensionless targetDutyCycle;
    private IDutyMotor motor;
    private static final Dimensionless DEFAULT_SPEED = Percent.of(70); // TODO: Determine if speed needs to be negated
    private static final Dimensionless MAX_DUTY = Percent.of(100);
    private static final Dimensionless MIN_DUTY = Percent.of(-100);

    public AlgaeRemoverSubsystem(String subsystemName, IDutyMotor motor) {
        super(subsystemName);

        this.motor = motor;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);

        builder.addBooleanProperty("IsEnabled", () -> targetDutyCycle.equals(Percent.zero()), null);
        builder.addDoubleProperty("Target Duty Cycle (%)", () -> targetDutyCycle.in(Percent), this::setTargetDutyCycle);
        builder.addDoubleProperty("Voltage (v)", () -> motor.getVoltage().in(Volts),
                v -> motor.setVoltage(Volts.of(v)));
    }

    private void setTargetDutyCycle(double duty) {
        setSpeed(() -> Percent.of(duty)).schedule();
    }

    private Command setSpeed(Supplier<Dimensionless> duty) {
        return run(() -> {
            Dimensionless dutyCycle = duty.get();
            if (dutyCycle.gt(MAX_DUTY) || dutyCycle.lt(MIN_DUTY)) {
                throw new IllegalArgumentException(String.format("Duty cycle must be between %d%% and %d%%; got %d%%", MIN_DUTY.in(Percent), MAX_DUTY.in(Percent), dutyCycle.in(Percent)));
            }

            targetDutyCycle = dutyCycle;

            motor.setDuty(dutyCycle);
        });
    }

    public Command removeAlgae() {
        return setSpeed(() -> DEFAULT_SPEED);
    }
}