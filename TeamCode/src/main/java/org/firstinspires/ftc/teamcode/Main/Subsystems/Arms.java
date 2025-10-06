package org.firstinspires.ftc.teamcode.Main.Subsystems;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Arms {

    // Servos
    private Servo arm1, arm2, arm3;

    // Positions
    public static final double ARM_1_UP = 0.405;
    public static final double ARM_1_DOWN = 0.195;
    public static final double ARM_2_UP = 0.55;
    public static final double ARM_2_DOWN = 0.34;
    public static final double ARM_3_UP = 0.55;
    public static final double ARM_3_DOWN = 0.34;

    // Flick flags
    private boolean flickArm1 = false;
    private boolean flickArm2 = false;
    private boolean flickArm3 = false;

    public Arms(HardwareMap hardwareMap) {
        arm1 = hardwareMap.get(Servo.class, "arm1");
        arm2 = hardwareMap.get(Servo.class, "arm2");
        arm3 = hardwareMap.get(Servo.class, "arm3");

        // Initialize all arms to DOWN
        arm1.setPosition(ARM_1_DOWN);
        arm2.setPosition(ARM_2_DOWN);
        arm3.setPosition(ARM_3_DOWN);
    }

    /** Call this every loop to update arms positions */
    public void update() {
        // Arm 1
        if (flickArm1) {
            arm1.setPosition(ARM_1_UP);
        } else {
            arm1.setPosition(ARM_1_DOWN);
        }

        // Arm 2
        if (flickArm2) {
            arm2.setPosition(ARM_2_UP);
        } else {
            arm2.setPosition(ARM_2_DOWN);
        }

        // Arm 3
        if (flickArm3) {
            arm3.setPosition(ARM_3_UP);
        } else {
            arm3.setPosition(ARM_3_DOWN);
        }
    }

    /** Set flick flags */
    public void flickArm1(boolean state) { flickArm1 = state; }
    public void flickArm2(boolean state) { flickArm2 = state; }
    public void flickArm3(boolean state) { flickArm3 = state; }

    /** Direct getters for telemetry */
    public double getArm1Position() { return arm1.getPosition(); }
    public double getArm2Position() { return arm2.getPosition(); }
    public double getArm3Position() { return arm3.getPosition(); }

    /** Reset all arms to down immediately */
    public void reset() {
        flickArm1 = false;
        flickArm2 = false;
        flickArm3 = false;
        update();
    }
}
