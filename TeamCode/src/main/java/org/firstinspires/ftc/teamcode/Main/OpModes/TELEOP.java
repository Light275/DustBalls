package org.firstinspires.ftc.teamcode.Main.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Main.Robot;

@TeleOp(name = "TELEOP", group = "Competitions")
public class TELEOP extends OpMode {

    Robot robot;

    // Arm3 shoot button and cooldown
    private boolean shootPressedLast = false;
    private double flywheelCooldownTime = 0;
    private final double FLYWHEEL_COOLDOWN = 1.0; // seconds

    @Override
    public void init() {
        robot = new Robot(hardwareMap);

        // Initialize all arms down
        robot.arms.reset();
    }

    @Override
    public void loop() {
        double currentTime = System.nanoTime() / 1e9;

        // --- DRIVE ---
        double drive = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;
        boolean slowMode = gamepad1.right_trigger > 0.1;
        robot.drivetrain.move(drive, strafe, rotate, slowMode);

        // --- INTAKE ---
        robot.intake.update(gamepad1.left_bumper, gamepad1.dpad_down);

        // --- FLYWHEEL ---
        boolean hasBalls = robot.colorSensors.numBalls() > 0;
        boolean cooldownActive = currentTime - flywheelCooldownTime < FLYWHEEL_COOLDOWN;

        // Keep flywheel spinning if balls or cooldown active
        if (hasBalls || cooldownActive) {
            robot.flywheel.update(robot.colorSensors.numBalls());
        } else {
            robot.flywheel.update(0); // idle speed
        }

        // --- TURRET TARGET CHECK ---
        boolean turretAtTarget = true; // replace with Diffy turret check if integrated
        boolean flywheelReady = robot.flywheel.isAtTargetVelocity();


        // --- ARM3 SHOOT ---
        boolean shootButton = gamepad1.a; // press A to shoot
        if (shootButton && !shootPressedLast && turretAtTarget && flywheelReady) {
            robot.arms.flickArm3(true);
            // Run indexer while shooting using public method
            robot.intake.runIndexer(robot.intake.getIndexerSpeed());
            flywheelCooldownTime = currentTime; // start flywheel cooldown
        } else {
            robot.arms.flickArm3(false);
        }
        shootPressedLast = shootButton;

        // --- UPDATE ROBOT SUBSYSTEMS ---
        robot.update();

        // --- TELEMETRY ---
        telemetry.addData("Arm1 Pos", robot.arms.getArm1Position());
        telemetry.addData("Arm2 Pos", robot.arms.getArm2Position());
        telemetry.addData("Arm3 Pos", robot.arms.getArm3Position());
        telemetry.addData("Flywheel Vel", robot.flywheel.getShooterVelocity());
        telemetry.addData("Num Balls", robot.colorSensors.numBalls());
        telemetry.update();
    }
}
