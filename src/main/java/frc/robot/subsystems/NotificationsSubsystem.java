package frc.robot.subsystems;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Elastic;

public class NotificationsSubsystem extends SubsystemBase {
    private double currentTime;
    private double lastTime;
    private double cooldown = 4;
    private final static ArrayList<Elastic.Notification> notifications = new ArrayList<>();
    
        public NotificationsSubsystem() {
            currentTime = 0;
            lastTime = 0;
        }
    
        @Override
        public void periodic() {
            currentTime = Timer.getFPGATimestamp();
    
            if ((currentTime - lastTime >= cooldown) && (notifications.size() != 0)) {
                System.out.println(currentTime + " : " + lastTime);
                System.out.println(currentTime - lastTime + " : " + cooldown);
                System.out.println("send notification");
                Elastic.sendNotification(notifications.get(0));
                notifications.remove(0);
                lastTime = Timer.getFPGATimestamp();
            }
        }
        public static void createNotification(int level, String name, String description) {
            Elastic.Notification notification = new Elastic.Notification(Elastic.Notification.NotificationLevel.INFO, "Test", "Test");
            
            if (level == 0) {
                notification = new Elastic.Notification(Elastic.Notification.NotificationLevel.INFO, name, description); 
            } else if (level == 1) {
                notification = new Elastic.Notification(Elastic.Notification.NotificationLevel.WARNING, name, description);
            } else if (level == 2) {
                notification = new Elastic.Notification(Elastic.Notification.NotificationLevel.ERROR, name, description);
            }
    
            // notifications[notifications.length + 1] = notification;
            System.out.println("Creating notification");
            notifications.add(notification.withDisplaySeconds(3));
            System.out.println(notifications.size());
    }
}
