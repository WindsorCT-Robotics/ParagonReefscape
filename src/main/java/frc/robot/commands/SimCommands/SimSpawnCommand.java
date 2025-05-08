package frc.robot.commands.SimCommands;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.SimulationCommands;

public class SimSpawnCommand extends Command{
    private final SimulationCommands simCommands;
    private final String object;
    private final boolean auto;
    private double current = 0;
    private double last = 0;

    public SimSpawnCommand(SimulationCommands simCommands, String object, boolean auto) {
        this.simCommands = simCommands;
        this.object = object;
        this.auto = auto;
    }

    @Override
    public void initialize() {
        if (!auto) {
            spawnObject();
        }
    }

    @Override
    public void execute() {
        
        current = Timer.getFPGATimestamp();
        if (auto && (current - last >= 2.0)) {
            spawnObject();
            last = Timer.getFPGATimestamp();
        }
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return !auto;
    }

    private void spawnObject() {
        if (object.equalsIgnoreCase("coral")) {
            simCommands.spawnCoral();
        } else if (object.equalsIgnoreCase("algae")) {
            simCommands.spawnAlgae();
        } else {
            simCommands.spawnAlgaeOnCoral();
        }
    }
}
