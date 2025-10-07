package frc.robot.subsystems.carriage;

import frc.robot.hardware.IBeamBreak;
import frc.robot.hardware.IDifferentialMotors;
import frc.robot.hardware.MotorDirection;
import frc.robot.units.Percent;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.littletonrobotics.junction.AutoLogOutput;

public class CarriageSubsystem extends SubsystemBase {
    private final IDifferentialMotors rollerMotors;
    private final Percent speed;
    private final IBeamBreak beamBreak;

    public CarriageSubsystem(IDifferentialMotors rollerMotors, IBeamBreak beamBreak) {
        speed = new Percent(0.25);
        this.rollerMotors = rollerMotors;
        this.beamBreak = beamBreak;
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Outtake Beam Breaker", isBeamBroken());
    }

    public void moveRollers(MotorDirection direction) {
        rollerMotors.setSpeed(speed, direction);
    }

    public void manualMoveRollers(Percent speed, MotorDirection direction) {
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