package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Units.Percent;
import frc.robot.subsystems.CarriageSubsystem;

public class OuttakeRollersCommand extends Command {
    private final CarriageSubsystem rollers;

    public OuttakeRollersCommand(CarriageSubsystem rollers) {
        this.rollers = rollers;
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        rollers.moveOuttakeRollers();
    }

    @Override
    public void end(boolean interrupted) {
        rollers.stopOuttakeRollers();
    }

    @Override
    public boolean isFinished() {
        return rollers.isBeamBroken();
    }
}