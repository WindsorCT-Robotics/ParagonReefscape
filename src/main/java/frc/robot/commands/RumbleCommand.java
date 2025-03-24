package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Limelight;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;

public class RumbleCommand extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final Limelight limelight;
    private final CommandXboxController driver;
    private final CommandXboxController op;
    private final int port;
    private final double rumblePercent;
    private boolean isFinished = false;
    
    public 
    RumbleCommand(CommandSwerveDrivetrain drivetrain, Limelight limelight, CommandXboxController driver, CommandXboxController op, double rumblePercent, int port) {
        this.drivetrain = drivetrain;
        this.limelight = limelight;
        this.driver = driver;
        this.op = op;
        this.rumblePercent = rumblePercent;
        this.port = port;
    }

    @Override
    public void initialize() {
        if (port == 0) {
            driver.setRumble(RumbleType.kBothRumble, rumblePercent);
        } else {
            op.setRumble(RumbleType.kBothRumble, rumblePercent);
        }
    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {
        driver.setRumble(RumbleType.kBothRumble, 0);
        op.setRumble(RumbleType.kBothRumble, 0);
    }

    @Override
    public boolean isFinished() {
        if (drivetrain.isValidTarget(limelight)) {
            isFinished = true;
        } else {
            isFinished = false;
        }
        return isFinished;
    }
}