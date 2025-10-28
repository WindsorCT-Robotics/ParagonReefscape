package frc.robot.subsystems.carriage;

import frc.robot.hardware.IBeamBreak;
import frc.robot.hardware.IDifferentialMotors;
import frc.robot.hardware.MotorDirection;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.util.sendable.SendableBuilder;

import static edu.wpi.first.units.Units.Percent;

import org.littletonrobotics.junction.AutoLogOutput;

public class CarriageSubsystem extends SubsystemBase {
    private final IDifferentialMotors rollerMotors;
    private final Dimensionless speed;
    private final IBeamBreak beamBreak;

    public CarriageSubsystem(String subsystemName, IDifferentialMotors rollerMotors, IBeamBreak beamBreak) {
        super(subsystemName);
        
        speed = Percent.of(25);
        this.rollerMotors = rollerMotors;
        this.beamBreak = beamBreak;
    }
    
    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        
        builder.addBooleanProperty("Outtake Beam Breaker", this::isBeamBroken, null);
    }

    public void moveRollers(MotorDirection direction) {
        rollerMotors.setSpeed(speed, direction);
    }

    public void manualMoveRollers(Dimensionless speed, MotorDirection direction) {
        rollerMotors.setSpeed(speed, direction);
    }

    public void moveRollersRight() {
        rollerMotors.moveRight(speed);
    }

    public void moveRollersLeft() {
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