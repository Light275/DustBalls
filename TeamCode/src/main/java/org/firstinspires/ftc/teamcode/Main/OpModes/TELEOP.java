package org.firstinspires.ftc.teamcode.Main.OpModes;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.CONFIG.RobotConfig;
import org.firstinspires.ftc.teamcode.Main.Robot;
import org.firstinspires.ftc.teamcode.Main.Subsystems.Diffy;
import org.firstinspires.ftc.teamcode.Main.Utils.PoseFusion;
import org.firstinspires.ftc.teamcode.Main.Utils.TagPoseProcessor;

@Config
@TeleOp(name = "TeleOp_FusedPose_Full", group = "Main")
public class TELEOP extends OpMode {

    private Robot robot;
    private TagPoseProcessor tagProcessor;
    private PoseFusion fusion;
    private FtcDashboard dashboard;

    // --- Diffy ---
    public static double TurretAngle = 0;
    private static final double MIN_ANGLE = 0;
    private static final double MAX_ANGLE = 180;

    // --- Flywheel (Dashboard Tunables) ---
    public static double flywheel_kP = 0.03;
    public static double flywheel_minPower = 0.2;
    public static double flywheel_velocityTolerance = 150;
    public static double targetVelocity = 1800;
    public static double velocityIncrement = 50;

    // --- Fusion Weights (Dashboard Tunables) ---
    public static double TAG_WEIGHT = 0.0005;
    public static double MAX_JUMP = 10.0;

    // --- Distance-Based Velocity Model ---
    // Velocity = baseVelocity + (distance * distanceCoefficient)
    public static double baseVelocity = 1680.0;
    public static double distanceCoefficient = 7.3; // velocity increase per inch of distance
    public static double maxAllowedVelocity = 3000;

    @Override
    public void init() {
        robot = new Robot(hardwareMap);
        tagProcessor = new TagPoseProcessor(hardwareMap);
        fusion = new PoseFusion(TAG_WEIGHT, MAX_JUMP);
        dashboard = FtcDashboard.getInstance();

        robot.arms.reset();
        robot.diffy.reset();

        // Flywheel config
        robot.flywheel.setkP(flywheel_kP);
        robot.flywheel.setMinPower(flywheel_minPower);
        robot.flywheel.setVelocityTolerance(flywheel_velocityTolerance);

        telemetry.addLine("TeleOp Initialized - Tunable Dashboard Mode (Linear Distance Coefficient)");
        telemetry.update();
    }

    @Override
    public void loop() {
        robot.update();
        tagProcessor.update();

        // Update fusion dynamically
        fusion = new PoseFusion(TAG_WEIGHT, MAX_JUMP);

        // --- POSE FUSION ---
        Pose2d odomPose = new Pose2d(robot.xPOS, robot.yPOS, robot.headingRad);
        Pose2d tagPose = tagProcessor.getTagPose();
        Pose2d fusedPose = fusion.update(odomPose, tagPose);

        double fusedX = fusedPose.position.x;
        double fusedY = fusedPose.position.y;
        double fusedHeading = odomPose.heading.toDouble(); // only odometry heading

        // --- DRIVE CONTROL ---
        double drive = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;
        boolean slowMode = gamepad1.right_trigger > 0.1;
        robot.drivetrain.move(drive, strafe, rotate, slowMode);

        // --- INTAKE ---
        robot.intake.update(gamepad1.left_bumper, gamepad1.dpad_down);

        // --- ARM FLICK ---
        if (gamepad1.a && robot.diffy.atTarget()) {
            robot.arms.flickArm3();
            robot.intake.runIndexer(robot.intake.getIndexerSpeed());
        }

        // --- TURRET AIM ---
        TurretAngle = robot.diffy.aimTurretAngle(odomPose.position.x, odomPose.position.y, odomPose.heading.toDouble());
        double clampedAngle = Range.clip(TurretAngle, MIN_ANGLE, MAX_ANGLE);
        robot.diffy.goToSlot(3);
        robot.diffy.setAngle(clampedAngle);
        robot.diffy.update();

        // --- FLYWHEEL CONTROL ---
        robot.flywheel.setkP(flywheel_kP);
        robot.flywheel.setMinPower(flywheel_minPower);
        robot.flywheel.setVelocityTolerance(flywheel_velocityTolerance);
        robot.flywheel.updateVelocity();

        // Rumble if on target
        if (robot.flywheel.isAtTargetVelocity() && robot.diffy.atTarget()) gamepad1.rumble(100);

        // --- DISTANCE CALCULATION ---
        double goalX = (RobotConfig.alliance == RobotConfig.Alliance.BLUE)
                ? Diffy.BLUE_GOAL_X : Diffy.RED_GOAL_X;
        double goalY = (RobotConfig.alliance == RobotConfig.Alliance.BLUE)
                ? Diffy.BLUE_GOAL_Y : Diffy.RED_GOAL_Y;

        double dx = goalX - fusedX;
        double dy = goalY - fusedY;
        double distanceInches = Math.sqrt(dx * dx + dy * dy);

        // --- LINEAR VELOCITY MODEL ---
        double autoTargetVelocity = baseVelocity + (distanceInches * distanceCoefficient);
        autoTargetVelocity = Range.clip(autoTargetVelocity, 0, maxAllowedVelocity);

        // Manual tuning
        double manualAdjust = -gamepad2.right_stick_y * velocityIncrement;
        targetVelocity = Range.clip(autoTargetVelocity + manualAdjust, 0, maxAllowedVelocity);

        // Apply target
        robot.flywheel.setTargetVelocity(targetVelocity);
        robot.flywheel.updateControl();

        // --- DASHBOARD TELEMETRY ---
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Distance (in)", distanceInches);
        packet.put("Auto Target Velocity", autoTargetVelocity);
        packet.put("Manual Adjust", manualAdjust);
        packet.put("Final Target Velocity", targetVelocity);
        packet.put("Flywheel Velocity", robot.flywheel.getShooterVelocity());
        packet.put("At Target", robot.flywheel.isAtTargetVelocity());
        packet.put("Fused X", fusedX);
        packet.put("Fused Y", fusedY);
        packet.put("Turret Angle", clampedAngle);
        dashboard.sendTelemetryPacket(packet);

        // --- DRIVER TELEMETRY ---
        telemetry.addData("Distance to Goal (in)", distanceInches);
        telemetry.addData("Auto Target Velocity", autoTargetVelocity);
        telemetry.addData("Manual Adjust", manualAdjust);
        telemetry.addData("Final Target Velocity", targetVelocity);
        telemetry.addData("Flywheel Velocity", robot.flywheel.getShooterVelocity());
        telemetry.addData("At Target", robot.flywheel.isAtTargetVelocity());
        telemetry.addData("Turret Angle", clampedAngle);
        telemetry.addData("Fused X", fusedX);
        telemetry.addData("Fused Y", fusedY);
        telemetry.update();
    }
}
