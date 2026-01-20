package frc.robot.subsystems.carriage;

import frc.robot.hardware.IBeamBreak;
import frc.robot.hardware.IDifferentialMotors;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.util.sendable.SendableBuilder;

import static edu.wpi.first.units.Units.Percent;

import org.littletonrobotics.junction.AutoLogOutput;

public class CarriageSubsystem extends SubsystemBase {
    private final IDifferentialMotors rollerMotors;
    private final IBeamBreak beamBreak;
    private static final Dimensionless DEFAULT_SPEED = Percent.of(25);

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
    
    public Dimensionless getDefaultSpeed() {
        return DEFAULT_SPEED;
    }

    public void moveRollers(Dimensionless speed) {
        rollerMotors.move(speed);
    }

    public void moveRollersRight(Dimensionless speed) {
        rollerMotors.moveRight(speed);
    }

    public void moveRollersLeft(Dimensionless speed) {
        rollerMotors.moveLeft(speed);
    }

    @AutoLogOutput(key = "Sensor/OuttakeBeam")
    public boolean isBeamBroken() {
        return beamBreak.isBeamBroken();
    }

    public void resetRollerEncoder() {
        rollerMotors.resetRelativeEncoder();
    }
    
    public void stopRollers() {
        rollerMotors.stop();
    }
}