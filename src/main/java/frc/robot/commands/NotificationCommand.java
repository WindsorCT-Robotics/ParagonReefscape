package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.utils.Elastic;

public class NotificationCommand extends Command{
    private int level;
    private String name;
    private String description;
    private Elastic.Notification notification;

    public NotificationCommand(int level, String name, String description) {
        this.level = level;
        this.name = name;
        this.description = description;
    }

    @Override
    public void initialize() {
        if (level == 0) {
            notification = new Elastic.Notification(Elastic.Notification.NotificationLevel.INFO, name, description); 
        } else if (level == 1) {
            notification = new Elastic.Notification(Elastic.Notification.NotificationLevel.WARNING, name, description); 
        } else if (level == 2) {
            notification = new Elastic.Notification(Elastic.Notification.NotificationLevel.ERROR, name, description); 
        }

        Elastic.sendNotification(notification);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
