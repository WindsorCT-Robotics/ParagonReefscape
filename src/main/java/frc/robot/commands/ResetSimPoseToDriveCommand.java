package frc.robot.commands;

import frc.robot.subsystems.CommandSwerveDrivetrain;

import edu.wpi.first.wpilibj2.command.Command;

public class ResetSimPoseToDriveCommand extends Command {
    private final CommandSwerveDrivetrain drivetrain;

    public ResetSimPoseToDriveCommand(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
    }

    @Override
    public void initialize() {
        drivetrain.resetPoseSimulationToDrive();
    }

    @Override
    public void execute() {
    }


    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return true;
    }
}