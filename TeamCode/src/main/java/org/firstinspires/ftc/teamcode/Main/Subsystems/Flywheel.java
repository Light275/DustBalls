package org.firstinspires.ftc.teamcode.Main.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Main.Utils.PID;

public class Flywheel {

    private DcMotorEx shooterL, shooterR;
    private double lastTicks = 0;
    private double lastTime = 0;
    private double velocity = 0; // measured velocity

    private PID pid;
    private double velocityTolerance = 50; // "ready" tolerance

    private static final double IDLE_POWER = 0.2;

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

        pid = new PID(0.0008, 0.00001, 0.00005); // tuned for smooth output
        pid.setOutputLimits(0.05, 1); // small minimum so motors spin
        pid.setEnabled(true);
    }

    /** Update encoder velocity */
    public void updateVelocity() {
        double currentTicks = shooterL.getCurrentPosition();
        double currentTime = System.nanoTime() / 1e9;

        double deltaTicks = currentTicks - lastTicks;
        double deltaTime = currentTime - lastTime;

        if (deltaTime > 0) velocity = deltaTicks / deltaTime;

        lastTicks = currentTicks;
        lastTime = currentTime;
    }

    /** Get current shooter velocity */
    public double getShooterVelocity() {
        return velocity;
    }

    /** Return PID-corrected power for a target velocity */
    public double getPIDPower(double targetVelocity) {
        // smooth PID output
        return pid.update(velocity, targetVelocity);
    }

    /** Spin motors */
    public void spin(double power) {
        shooterL.setPower(power);
        shooterR.setPower(power);
    }

    /** Enable PID */
    public void enablePID() {
        pid.setEnabled(true);
    }

    /** Disable PID */
    public void disablePID() {
        pid.setEnabled(false);
        spin(IDLE_POWER);
    }

    /** Update PID coefficients live */
    public void setPIDCoefficients(double kP, double kI, double kD) {
        pid.setPID(kP, kI, kD);
    }

    /** Flywheel "ready" check */
    public boolean isAtTargetVelocity(double targetVelocity) {
        return Math.abs(velocity - targetVelocity) <= velocityTolerance;
    }

    /** Optional: set tolerance */
    public void setVelocityTolerance(double tolerance) {
        velocityTolerance = tolerance;
    }
}
