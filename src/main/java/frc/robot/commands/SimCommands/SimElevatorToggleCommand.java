package frc.robot.commands.SimCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator.ElevatorSubsystemSim;

public class SimElevatorToggleCommand extends Command {
    private final ElevatorSubsystemSim elevator;
    private final double level;

    public SimElevatorToggleCommand(ElevatorSubsystemSim elevator, double level) {
        this.elevator = elevator;
        this.level = level;
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        if (level == 1) {
            elevator.setToL1();
        } else if (level == 2.0) {
            elevator.setToL2();
        } else if (level == 2.5) {
            elevator.setToL2_5();
        } else {
            elevator.setToL3();
        }
    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        // if (level == 1) {
        //     return elevator.isAtL1();
        // } else if (level == 2.0) {
        //     return elevator.isAtL2();
        // } else if (level == 2.5) {
        //     return elevator.isAtL2_5();
        // } else {
        //     return elevator.isAtL3();
        // }
        return false;
    }
}