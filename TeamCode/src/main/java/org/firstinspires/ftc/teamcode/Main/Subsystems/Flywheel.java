package org.firstinspires.ftc.teamcode.Main.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.CONFIG.RobotConfig;
import org.firstinspires.ftc.teamcode.Main.Robot;
import org.firstinspires.ftc.teamcode.Main.Utils.PID;

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
    private static final double IDLE_POWER = 0.2;

    // GOAL POSITIONS
    private static final double BLUE_X = -72;
    private static final double BLUE_Y = 72;
    private static final double RED_X = 72;
    private static final double RED_Y = 72;

    private static final double SLOPE = (MAX_VEL - MIN_VEL) / (MAX_DIST - MIN_DIST);

    private Robot robot;
    private PID pid;

    // Flywheel ready tolerance
    private double velocityTolerance = 50;

    // Cooldown timer
    private double lastBallTime = 0;
    private final double COOLDOWN = 1.0; // seconds

    public Flywheel(HardwareMap hardwareMap, Robot robot) {
        shooterL = hardwareMap.get(DcMotorEx.class, "shooterL");
        shooterR = hardwareMap.get(DcMotorEx.class, "shooterR");

        shooterL.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterR.setDirection(DcMotorSimple.Direction.REVERSE);

        shooterL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        shooterR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        shooterL.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        shooterR.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        lastTicks = shooterR.getCurrentPosition();
        lastTime = System.nanoTime() / 1e9; // seconds

        this.robot = robot;
        pid = new PID(0.001, 0.001, 0.0001);
    }

    /** Returns the goal coordinates based on current alliance */
    private double[] getGoalPosition() {
        if (RobotConfig.alliance == RobotConfig.Alliance.BLUE) {
            return new double[]{BLUE_X, BLUE_Y};
        } else {
            return new double[]{RED_X, RED_Y};
        }
    }

    /** Returns distance to the goal in inches */
    public double getDistToGoal() {
        double[] goal = getGoalPosition();
        double gx = goal[0];
        double gy = goal[1];

        double px = robot.drive.localizer.getPose().position.x;
        double py = robot.drive.localizer.getPose().position.y;

        return Math.hypot(px - gx, py - gy);
    }

    /** Linear interpolation: convert distance to target flywheel velocity */
    public double getTargetVelocity() {
        double dist = getDistToGoal();
        return SLOPE * (dist - MIN_DIST) + MIN_VEL;
    }

    /** Spins flywheel at given power */
    public void spin(double power) {
        shooterL.setPower(power);
        shooterR.setPower(power);
    }

    /** Returns PID-corrected power */
    public double PIDPower() {
        return pid.update(getShooterVelocity(), getTargetVelocity());
    }

    /** Update shooter velocity using encoder ticks */
    public void updateVelocity() {
        double currentTicks = shooterR.getCurrentPosition();
        double currentTime = System.nanoTime() / 1e9;

        double deltaTicks = currentTicks - lastTicks;
        double deltaTime = currentTime - lastTime;

        if (deltaTime != 0) shooterRVelocity = deltaTicks / deltaTime;

        lastTicks = currentTicks;
        lastTime = currentTime;
    }

    /** Main update loop: call each cycle, numBalls from ColorSensors */
    public void update(double numBalls) {
        double currentTime = System.nanoTime() / 1e9;

        if (numBalls > 0) {
            lastBallTime = currentTime; // reset cooldown
        }

        boolean cooldownActive = (currentTime - lastBallTime) < COOLDOWN;

        if (numBalls > 0 || cooldownActive) {
            updateVelocity();
            spin(PIDPower());
            pid.setEnabled(true);
        } else {
            pid.setEnabled(false);
            spin(IDLE_POWER);
        }
    }

    /** Current shooter velocity */
    public double getShooterVelocity() {
        return shooterRVelocity;
    }

    /** Check if flywheel is within target velocity range */
    public boolean isAtTargetVelocity() {
        return Math.abs(shooterRVelocity - getTargetVelocity()) <= velocityTolerance;
    }

    /** Optional: set custom tolerance */
    public void setVelocityTolerance(double tolerance) {
        velocityTolerance = tolerance;
    }
}
