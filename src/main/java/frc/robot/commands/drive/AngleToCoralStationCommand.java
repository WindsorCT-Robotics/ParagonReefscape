package frc.robot.commands.drive;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

import java.util.function.Supplier;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AngleToCoralStationCommand extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final Supplier<Distance> positionY;
    private Angle angle;
    private final Distance WDITH_OF_HALF_FIELD = Meters.of(AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark).getFieldWidth() * 0.5);
    
    public AngleToCoralStationCommand(CommandSwerveDrivetrain drivetrain, Supplier<Distance> positionY) {
        this.drivetrain = drivetrain;
        this.positionY = positionY;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        setAngleCommand = new SetAngleCommand(drivetrain, angle);
    }
    
    @Override
    public void execute() {
        updateAngleToCoralStation();
    }
    
    @Override
    public void end(boolean interrupted) {
        setAngleCommand.end(false);
    }
    
    @Override
    public boolean isFinished() {
        return false;
    }

    private void updateAngleToCoralStation() {
        if (positionY.get().in(Meters) > WDITH_OF_HALF_FIELD.in(Meters)) {
            setAngleToRightCoralStation();
        } else {
            setAngleToLeftCoralStation();
        }
    }

    // TODO: These angles are placeholders change them to their actual values. 
    // TODO: Check if methods have the correct logic.
    public void setAngleToLeftCoralStation() {
        if (DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Blue)) {
            angle = Degrees.of(Placeholder);
        } else {
            angle = Degrees.of(Placeholder);
        }
    }

    public void setAngleToRightCoralStation() {
        if (DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Blue)) {
            angle = Degrees.of(Placeholder);
        } else {
            angle = Degrees.of(Placeholder);
        }
    }
}
