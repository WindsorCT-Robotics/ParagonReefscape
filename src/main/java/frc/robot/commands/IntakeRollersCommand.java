package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Units.Percent;
import frc.robot.subsystems.CarriageSubsystem;

public class IntakeRollersCommand extends Command {
    private final CarriageSubsystem intake;

    public IntakeRollersCommand(CarriageSubsystem intake) {
        this.intake = intake;
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        intake.moveIntakeRollers(new Percent(0.75));
    }

    @Override
    public void end(boolean interrupted) {
        intake.stop();
    }

    @Override
    public boolean isFinished() {
        return intake.isBeamBroken();
    }
}