package frc.robot.subsystems.carriage;

import frc.robot.hardware.IBeamBreak;
import frc.robot.hardware.IDifferentialMotors;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.util.sendable.SendableBuilder;

import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Seconds;

import org.littletonrobotics.junction.AutoLogOutput;

public class CarriageSubsystem extends SubsystemBase {
    private final IDifferentialMotors rollerMotors;
    private final IBeamBreak beamBreak;
    private static final Dimensionless DEFAULT_SPEED = Percent.of(25);
    private final Time debounceTime = Milliseconds.of(50);
    public final Trigger beamBroken = new Trigger(this::isBeamBroken).debounce(debounceTime.in(Seconds), DebounceType.kBoth);
    public final Trigger beamIntact = new Trigger(this::isBeamIntact).debounce(debounceTime.in(Seconds), DebounceType.kBoth);

    public enum CamberDirection {
        /**
        * Curves or spins the coral left.
        */
        LEFT,

        /**
        * Curves or spins the coral right.
        */
        RIGHT
    }

    public CarriageSubsystem(String subsystemName, IDifferentialMotors rollerMotors, IBeamBreak beamBreak) {
        super(subsystemName);
        this.rollerMotors = rollerMotors;
        this.beamBreak = beamBreak;
    }
    
    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        
        // TODO: Add left and right motor telemetry
        builder.addBooleanProperty("Outtake Beam Breaker", this::isBeamBroken, null);
    }
    
    public Command loadCoral() {
        return runEnd(this::moveRollers, rollerMotors::stop).until(beamBroken);
    }
    
    public Command scoreCoral() {
        return runEnd(this::moveRollers, rollerMotors::stop).until(beamIntact);
    }
    
    public Command unloadCoral() {
        return runEnd(this::reverseRollers, rollerMotors::stop).until(beamIntact);
    }

    public Command reloadCoral() {
        return unloadCoral().andThen(loadCoral());
    }
    
    public Command scoreCoralCambered(CamberDirection direction) {
        return runEnd(() -> {
            if (direction == CamberDirection.LEFT) {
                rollerMotors.moveLeft(DEFAULT_SPEED);
            } else {
                rollerMotors.moveRight(DEFAULT_SPEED);
            }
        }, rollerMotors::stop).until(beamIntact);
    }

    public Command resetMotorEncoder() {
        return runOnce(rollerMotors::resetRelativeEncoder);
    }

    private void moveRollers() {
        rollerMotors.move(DEFAULT_SPEED);
    }
    
    private void reverseRollers() {
        rollerMotors.move(DEFAULT_SPEED.unaryMinus());
    }

    @AutoLogOutput(key = "Sensor/OuttakeBeam")
    private boolean isBeamBroken() {
        return beamBreak.isBeamBroken();
    }
    
    private boolean isBeamIntact() {
        return !beamBreak.isBeamBroken();
    }
}