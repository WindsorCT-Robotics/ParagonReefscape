package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Units.Percent;
import frc.robot.subsystems.CarriageSubsystem;

public class OuttakeRollersCommand extends Command {
    private final CarriageSubsystem outtake;

    public OuttakeRollersCommand(CarriageSubsystem outtake) {
        this.outtake = outtake;
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        outtake.outtakeRollers(new Percent(0));
    }

    @Override
    public void end(boolean interrupted) {
        outtake.stop();
    }

    @Override
    public boolean isFinished() {
        return outtake.isBeamBroken();
    }
}