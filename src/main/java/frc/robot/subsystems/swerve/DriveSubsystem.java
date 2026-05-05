package frc.robot.subsystems.swerve;

import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

public class DriveSubsystem {
    public enum SwerveModule {
        FL("Front Left"),
        FR("Front Right"),
        BL("Back Left"),
        BR("Back Right");

        private String fullName;

        SwerveModule(String fullName) {
            this.fullName = fullName;
        }

        @Override
        public String toString() {
            return fullName + " Swerve Module";
        }

        public String toKey() {
            return fullName.replace(" ", "");
        }
    };

    static final Lock ODOMETRY_LOCK = new ReentrantLock();

}
