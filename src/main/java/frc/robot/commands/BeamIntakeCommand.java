package frc.robot.commands;

import frc.robot.subsystems.CarriageSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

public class BeamIntakeCommand extends Command {
    private final CarriageSubsystem rollers;

    public BeamIntakeCommand(CarriageSubsystem rollers) {
        this.rollers = rollers;
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        rollers.moveRollers();
    }

    @Override
    public void end(boolean interrupted) {
        rollers.stopRollers();
    }

    @Override
    public boolean isFinished() {
        return rollers.isBeamBroken();
    }
}