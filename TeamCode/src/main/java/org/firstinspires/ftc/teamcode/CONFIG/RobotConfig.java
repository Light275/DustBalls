package org.firstinspires.ftc.teamcode.CONFIG;

public class RobotConfig {

    /** Alliance options */
    public enum Alliance {
        RED,
        BLUE,
        UNKNOWN
    }

    /** Public static so it can be accessed anywhere */
    public static Alliance alliance = Alliance.BLUE;

    /** Optional: store other global settings */
    public static boolean isAutonomousDone = false;
}
