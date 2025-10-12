package org.firstinspires.ftc.teamcode.Main.OpModes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Main.Robot;
import org.firstinspires.ftc.teamcode.Main.Subsystems.Diffy;
import org.firstinspires.ftc.teamcode.CONFIG.RobotConfig;

@Config
@TeleOp(name = "TELEOP", group = "Main")
public class TELEOP extends com.qualcomm.robotcore.eventloop.opmode.OpMode {

    private Robot robot;

    // Turret limits
    public static double MIN_ANGLE = 0;
    public static double MAX_ANGLE = 180;

    // Flywheel tuning (dashboard adjustable)
    public static double baseVelocity = 200;
    public static double distanceCoefficient = 1.3;
    public static double maxAllowedVelocity = 580;

    // Increment rates
    public static double baseVelocityIncrement = 1;
    public static double distanceCoefficientIncrement = 0.1;

    @Override
    public void init() {
        robot = new Robot(hardwareMap);
        robot.arms.reset();
        robot.diffy.reset();

        telemetry.addLine("TELEOP Initialized - Base & Coefficient Adjustable via Gamepad2");
        telemetry.update();
    }

    @Override
    public void loop() {
        robot.update();

        // --- DRIVE CONTROL ---
        double drive = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;
        boolean slowMode = gamepad1.right_trigger > 0.1;
        robot.drivetrain.move(drive, strafe, rotate, slowMode);

        // --- INTAKE ---
        robot.intake.update(gamepad1.left_bumper, gamepad1.dpad_down);

        // --- ARM FLICK & INDEXER ---
        if (gamepad1.a && robot.diffy.atTarget()) {
            robot.arms.flickArm3();
            robot.intake.runIndexer(robot.intake.getIndexerSpeed());
        }

        // --- DISTANCE & TURRET CONTROL ---
        double goalX = (RobotConfig.alliance == RobotConfig.Alliance.BLUE)
                ? Diffy.BLUE_GOAL_X : Diffy.RED_GOAL_X;
        double goalY = (RobotConfig.alliance == RobotConfig.Alliance.BLUE)
                ? Diffy.BLUE_GOAL_Y : Diffy.RED_GOAL_Y;

        double dx = goalX - robot.xPOS;
        double dy = goalY - robot.yPOS;
        double distanceInches = Math.sqrt(dx*dx + dy*dy);

        double turretAngle = robot.diffy.aimTurretAngle(robot.xPOS, robot.yPOS, robot.headingRad);
        turretAngle = Range.clip(turretAngle, MIN_ANGLE, MAX_ANGLE);
        robot.diffy.setAngle(turretAngle);
        robot.diffy.update();

        // --- ADJUST BASE VELOCITY & DISTANCE COEFFICIENT ---
        baseVelocity += -gamepad2.right_stick_y * baseVelocityIncrement;
        distanceCoefficient += -gamepad2.left_stick_y * distanceCoefficientIncrement;

        baseVelocity = Range.clip(baseVelocity, 0, maxAllowedVelocity);
        distanceCoefficient = Math.max(distanceCoefficient, 0);

        // --- FLYWHEEL CONTROL ---
        double targetVelocity = baseVelocity + distanceCoefficient * distanceInches;
        targetVelocity = Range.clip(targetVelocity, 0, maxAllowedVelocity);

        double a = 0.0000413348;
        double b = -0.00206321;
        double c = 0.548967;
        double d = 299.02768;

        // ax^3 + bx^2 + cx + d
        double goonVelocity = ((a * (Math.pow(distanceInches, 3))) + (b * Math.pow(distanceInches, 2)) + ((c * distanceInches) + d));

        robot.flywheel.setTargetVelocity(goonVelocity);
        robot.flywheel.update();

        // --- TELEMETRY ---
        telemetry.addData("Distance to Goal", "%.2f in", distanceInches);
        telemetry.addData("Turret Angle", "%.2f", turretAngle);
        telemetry.addData("Base Velocity", "%.1f", baseVelocity);
        telemetry.addData("Distance Coefficient", "%.2f", distanceCoefficient);
        telemetry.addData("Target Velocity", "%.2f rad/s", targetVelocity);
        telemetry.addData("Regression Velocity", goonVelocity);
        telemetry.addData("Flywheel Actual", "%.2f rad/s", robot.flywheel.getVelocityRadPerSec());
        telemetry.update();
    }
}
