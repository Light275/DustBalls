package org.firstinspires.ftc.teamcode.CONFIG;

public class RobotConfig {

    /** Alliance options */
    public enum Alliance {
        RED,
        BLUE,
        UNKNOWN
    }

    /** Public static so it can be accessed anywhere */
    public static Alliance alliance = Alliance.UNKNOWN;

}
