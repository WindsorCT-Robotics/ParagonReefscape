package frc.robot.commands;

import frc.robot.subsystems.CarriageSubsystem;
// import frc.robot.subsystems.NotificationsSubsystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class CoralOuttakeCommand extends Command {
    private final CarriageSubsystem rollers;
    private final double level;
    private final String direction;
    private double lastTime;
    private double currentTime;
    private boolean sent = false;

    public CoralOuttakeCommand(CarriageSubsystem rollers, double level, String direction) {
        this.rollers = rollers;
        this.level = level;
        this.direction = direction;
    }

    @Override
    public void initialize() {
        SmartDashboard.putBoolean("greater than one second", sent);
        lastTime = Timer.getFPGATimestamp();
        // if (!rollers.isBeamBroken()) {
        //     NotificationsSubsystem.createNotification(1, "Warning Notification", "No coral detected");
        // }
        

        if (level == 1) {
            if (direction.equalsIgnoreCase("left")) {
                rollers.moveRollersRight();
            } else {
                rollers.moveRollersLeft();
            }
        } else {
            rollers.moveRollers(false);
        }
    }


    @Override
    public void execute() {
        currentTime = Timer.getFPGATimestamp();
        if (currentTime - lastTime > 1 && !sent) {
            sent = true;
            SmartDashboard.putBoolean("greater than one second", sent);
        }
    }

    @Override
    public void end(boolean interrupted) {
        rollers.stopRollers();
    }

    @Override
    public boolean isFinished() {
        return !rollers.isBeamBroken();
    }
}