package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.NotificationsSubsystem;

public class NotificationCommand extends Command{
    private int level;
    private String name;
    private String description;
    

    public NotificationCommand(int level, String name, String description) {
        this.level = level;
        this.name = name;
        this.description = description;
    }

    @Override
    public void initialize() {
        NotificationsSubsystem.createNotification(level, name, description);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
