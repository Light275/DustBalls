package org.firstinspires.ftc.teamcode.Main.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import java.util.ArrayDeque;
import java.util.Deque;

public class Flywheel {

    private final DcMotorEx shooterL, shooterR;

    // velocity smoothing buffer (ticks/sec)
    private final Deque<Double> velBuffer = new ArrayDeque<>();
    private final int BUFFER_SIZE = 18;
    private double smoothedVelocity = 0.0;

    // last tick/time for instantaneous velocity calc
    private double lastTicks = 0;
    private double lastTime = 0;
    public double power;

    // control params (tunable)
    private double kP = 0.05;
    private double minPower = 0;         // minimum power to keep wheel spinning when desired
    private double maxPower = 1.0;
    private double velocityTolerance = 200;

    // target
    private double targetVelocity = 0.0;

    public Flywheel(HardwareMap hardwareMap) {
        shooterL = hardwareMap.get(DcMotorEx.class, "shooterL");
        shooterR = hardwareMap.get(DcMotorEx.class, "shooterR");

        shooterL.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterR.setDirection(DcMotorSimple.Direction.REVERSE);

        shooterL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        shooterR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        shooterL.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        shooterR.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        lastTicks = shooterL.getCurrentPosition();
        lastTime = System.nanoTime() / 1e9;
    }

    /**
     * Call frequently to compute smoothed velocity (ticks/sec).
     * This pushes the instantaneous velocity into a rolling buffer and computes the average.
     */
    public void updateVelocity() {
        double currentTicks = shooterL.getCurrentPosition();
        double currentTime = System.nanoTime() / 1e9;

        double deltaTicks = currentTicks - lastTicks;
        double deltaTime = currentTime - lastTime;

        double instVel = 0.0;
        if (deltaTime > 0) {
            instVel = deltaTicks / deltaTime;
        }

        // push into buffer
        velBuffer.addLast(instVel);
        if (velBuffer.size() > BUFFER_SIZE) velBuffer.removeFirst();

        // compute average
        double sum = 0.0;
        for (double v : velBuffer) sum += v;
        smoothedVelocity = velBuffer.isEmpty() ? 0.0 : (sum / velBuffer.size());

        lastTicks = currentTicks;
        lastTime = currentTime;
    }

    /** Returns the smoothed velocity (ticks/sec). */
    public double getShooterVelocity() {
        return smoothedVelocity;
    }

    /** Set the target velocity (ticks/sec). */
    public void setTargetVelocity(double target) {
        this.targetVelocity = Math.max(0.0, target);
    }

    /** Get current target velocity. */
    public double getTargetVelocity() {
        return targetVelocity;
    }

    /** Set proportional gain. */
    public void setkP(double kP) {
        this.kP = kP;
    }

    /** Set minimum holding power (used when targetVelocity > 0). */
    public void setMinPower(double minPower) {
        this.minPower = Range.clip(minPower, 0.0, 1.0);
    }

    /** Set max motor power. */
    public void setMaxPower(double maxPower) {
        this.maxPower = Range.clip(maxPower, 0.0, 1.0);
    }

    /** Set how close we consider "at target". */
    public void setVelocityTolerance(double tol) {
        this.velocityTolerance = Math.max(0.0, tol);
    }

    /** Returns true when smoothed velocity is within tolerance of target. */
    public boolean isAtTargetVelocity() {
        return Math.abs(smoothedVelocity - targetVelocity) <= velocityTolerance;
    }

    /**
     * Proportional control to drive motors toward targetVelocity.
     * - power = minPower + kP * (error)
     * - clipped to [0, maxPower]
     *
     * Behavior note:
     * - If targetVelocity > 0 we maintain at least minPower to keep wheel turning.
     * - If targetVelocity == 0 we allow power to go to 0 (so we can stop).
     */
    public void updateControl() {
        double error = targetVelocity - smoothedVelocity;
        power = kP * error;

        // If target is zero, allow power to drop to zero
        if (targetVelocity <= 0.0) {
            power = kP * error; // can be negative or zero; we'll clip below
        }

        // clip and ensure we don't drive negative (flywheel spins forward only)
        power = Range.clip(power, 0.0, maxPower);

        shooterL.setPower(power);
        shooterR.setPower(power);
    }

    /** Manual spin override. */
    public void spin(double power) {
        power = Range.clip(power, -maxPower, maxPower);
        shooterL.setPower(power);
        shooterR.setPower(power);
    }


}
