package org.firstinspires.ftc.teamcode.Main.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

@Config
public class Diffy {

    // --- Targets ---
    public static double targetL = 0;
    public static double targetR = 0;

    // --- PID ---
    public static double kP_L = 0.0003;
    public static double kP_R = 0.0003;

    // --- Power settings ---
    public static double minPower = 0.07;
    public static double maxPower = 1.0;

    // --- Precision ---
    public static double toleranceTicks = 130;

    // --- Slots & turret angle ---
    public static double slot1Pos = -5500;
    public static double slot2Pos = -2800;
    public static double slot3Pos = 0;

    public static double angleOffset = 0;     // Current added offset
    public static double angleScale = 34.4;   // Ticks per degree

    // --- Angle limits ---
    public static double minAngle = 0;
    public static double maxAngle = 180;

    // --- Hardware ---
    public CRServo diffyL, diffyR;
    public DcMotorEx encL, encR;

    // --- Internal base for slot position ---
    private double currentSlotBase = 0;

    public Diffy(HardwareMap hardwareMap) {
        diffyL = hardwareMap.get(CRServo.class, "diffyL");
        diffyR = hardwareMap.get(CRServo.class, "diffyR");

        encL = hardwareMap.get(DcMotorEx.class, "intake");
        encR = hardwareMap.get(DcMotorEx.class, "leftBack");
        encL.setDirection(DcMotorSimple.Direction.FORWARD);
        encR.setDirection(DcMotorSimple.Direction.FORWARD);


        // ✅ Make both servos run the same direction
        diffyL.setDirection(CRServo.Direction.FORWARD);
        diffyR.setDirection(CRServo.Direction.FORWARD);

        resetEncoders();
    }

    /** Update proportional control loop */
    public void update() {
        double errorL = targetL - encL.getCurrentPosition();
        double errorR = targetR - encR.getCurrentPosition();

        // --- Stop motors if turret exceeds safe limits ---
        if (angleOffset < minAngle && errorL > 0 && errorR < 0) {
            errorL = 0;
            errorR = 0;
        }
        if (angleOffset > maxAngle && errorL < 0 && errorR > 0) {
            errorL = 0;
            errorR = 0;
        }

        double powerL = Range.clip(errorL * kP_L, -maxPower, maxPower);
        double powerR = Range.clip(errorR * kP_R, -maxPower, maxPower);

        // Deadband to prevent jitter
        if (Math.abs(powerL) < minPower) powerL = 0;
        if (Math.abs(powerR) < minPower) powerR = 0;

        // --- Apply power ---
        diffyL.setPower(powerL);
        diffyR.setPower(-powerR); // mirror right side here (software)
    }

    /** Encoder errors */
    public double[] getErrors() {
        return new double[]{
                targetL - encL.getCurrentPosition(),
                targetR - encR.getCurrentPosition()
        };
    }

    /** Move to slot (1, 2, or 3) */
    public void goToSlot(int slot) {
        switch (slot) {
            case 1: currentSlotBase = slot1Pos; break;
            case 2: currentSlotBase = slot2Pos; break;
            case 3:
            default: currentSlotBase = slot3Pos; break;
        }
        updateTargets();
    }

    /** Set turret rotation offset */
    public void setAngle(double offset) {
        angleOffset = Range.clip(offset, minAngle, maxAngle);
        updateTargets();
    }

    /** Update target positions */
    private void updateTargets() {
        targetL = currentSlotBase - angleOffset * angleScale;
        targetR = currentSlotBase + angleOffset * angleScale;
    }

    public void reset() {
        currentSlotBase = 0;
        angleOffset = 0;
        targetL = 0;
        targetR = 0;
        resetEncoders();
    }

    public void resetEncoders() {
        encL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        encR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        encL.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        encR.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    public boolean atTarget() {
        double[] e = getErrors();
        return Math.abs(e[0]) <= toleranceTicks && Math.abs(e[1]) <= toleranceTicks;
    }
}
