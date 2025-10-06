package org.firstinspires.ftc.teamcode.Mains;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Flywheel {
    private DcMotorEx shooterL, shooterR;
    private double lastTicks = 0;
    private double lastTime = 0;
    private double shooterRVelocity = 0;

    // constants for interpolation
    private static final double MIN_DIST = 48;
    private static final double MIN_VEL = 2400;
    private static final double MAX_DIST = 173;
    private static final double MAX_VEL = 5800;
    private static final double SLOPE = (MAX_VEL - MIN_VEL) / (MAX_DIST - MIN_DIST);

    public Flywheel(HardwareMap hardwareMap) {
        shooterL = hardwareMap.get(DcMotorEx.class, "shooterL");
        shooterR = hardwareMap.get(DcMotorEx.class, "shooterR");

        shooterL.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterR.setDirection(DcMotorSimple.Direction.REVERSE);

        shooterL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        shooterL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // initialize timestamp and ticks
        lastTicks = shooterR.getCurrentPosition();
        lastTime = System.nanoTime() / 1e9; // seconds
    }

    /**
     * Linear interpolation for target velocity based on distance.
     */
    public double getTargetVelocity(double inchDist) {
        return SLOPE * (inchDist - MIN_DIST) + MIN_VEL;
    }

    /**
     * Called every loop
     */
    public void updateVelocity() {
        double currentTicks = shooterR.getCurrentPosition();
        double currentTime = System.nanoTime() / 1e9;

        double deltaTicks = currentTicks - lastTicks;
        double deltaTime = currentTime - lastTime;

        if (deltaTime != 0) {
            shooterRVelocity = deltaTicks / deltaTime; // ticks per second
        }

        lastTicks = currentTicks;
        lastTime = currentTime;
    }

    /**
     * Get the current measured velocity (ticks/sec)
     */
    public double getShooterRVelocity() {
        return shooterRVelocity;
    }
}
