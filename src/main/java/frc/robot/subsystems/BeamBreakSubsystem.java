package frc.robot.subsystems;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.hardware.IBeamBreak;

public class BeamBreakSubsystem extends SubsystemBase {
    private final IBeamBreak beam;

    public BeamBreakSubsystem(String name, IBeamBreak beam) {
        super(name);
        this.beam = beam;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);

        builder.addBooleanProperty("Is Beam Broken?", beam::isBeamBroken, null);
    }

    public boolean isBeamBroken() {
        return beam.isBeamBroken();
    }
}
