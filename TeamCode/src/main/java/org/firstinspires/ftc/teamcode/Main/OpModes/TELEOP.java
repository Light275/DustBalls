package org.firstinspires.ftc.teamcode.Main.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.CONFIG.RobotConfig;
import org.firstinspires.ftc.teamcode.Main.Robot;
import org.firstinspires.ftc.teamcode.Main.Subsystems.Diffy;

@Config
@TeleOp(name = "GURT TELEOP", group = "Competitions")
public class TELEOP extends OpMode {

    private Robot robot;
    private FtcDashboard dashboard;

    // --- Diffy angle tuning ---
    public static double TurretAngle = 90; // Dashboard tunable
    private static final double MIN_ANGLE = 0;
    private static final double MAX_ANGLE = 180;

    // --- Flywheel tuning ---
    public static double flywheel_kP = 0.028;
    public static double flywheel_minPower = 0.4;
    public static double flywheel_velocityTolerance = 400;
    public static double targetVelocity = 1800; // Starting target
    public static double velocityIncrement = 50; // How much each stick input changes velocity

    @Override
    public void init() {

        robot = new Robot(hardwareMap);
        dashboard = FtcDashboard.getInstance();

        robot.arms.reset();
        robot.diffy.reset();

        // Flywheel config
        robot.flywheel.setkP(flywheel_kP);
        robot.flywheel.setMinPower(flywheel_minPower);
        robot.flywheel.setVelocityTolerance(flywheel_velocityTolerance);

        telemetry.addLine("Robot Initialized");
        telemetry.addLine("Holding fixed turret angle (Dashboard)");
        telemetry.addLine("Flywheel ready for manual velocity control");
        telemetry.update();
    }

    @Override
    public void loop() {
        // --- DRIVE ---
        double drive = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;
        boolean slowMode = gamepad1.right_trigger > 0.1;
        robot.drivetrain.move(drive, strafe, rotate, slowMode);

        // --- INTAKE ---
        robot.intake.update(gamepad1.left_bumper, gamepad1.dpad_down);

        // --- ARM 3 FLICK ---
        if (gamepad1.a && robot.diffy.atTarget()) {
            robot.arms.flickArm3();
            robot.intake.runIndexer(robot.intake.getIndexerSpeed());
        }

        // --- TURRET CONTROL ---
        TurretAngle = robot.diffy.aimTurretAngle(robot.xPOS, robot.yPOS, robot.headingRad);
        double clampedAngle = Range.clip(TurretAngle, MIN_ANGLE, MAX_ANGLE);
        robot.diffy.goToSlot(3);
        robot.diffy.setAngle(clampedAngle);
        robot.diffy.update();

        // --- FLYWHEEL CONTROL ---
        // Smooth velocity measurement
        robot.flywheel.updateVelocity();
        robot.flywheel.setkP(flywheel_kP);
        robot.flywheel.setMinPower(flywheel_minPower);
        robot.flywheel.setVelocityTolerance(flywheel_velocityTolerance);

        // Adjust target velocity with gamepad2
        targetVelocity += -gamepad2.right_stick_y * velocityIncrement;
        targetVelocity = Range.clip(targetVelocity, 0, 4000);

        robot.flywheel.setTargetVelocity(targetVelocity);
        robot.flywheel.updateControl();

        // --- UPDATE ROBOT ---
        robot.update();

        // --- DASHBOARD TELEMETRY ---
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("TargetL", Diffy.targetL);
        packet.put("TargetR", Diffy.targetR);
        packet.put("EncoderL", robot.diffy.encL.getCurrentPosition());
        packet.put("EncoderR", robot.diffy.encR.getCurrentPosition());
        packet.put("Angle", robot.diffy.angleOffset);

        double[] errors = robot.diffy.getErrors();
        packet.put("ErrorL", errors[0]);
        packet.put("ErrorR", errors[1]);

        // Flywheel info
        packet.put("TargetVelocity", targetVelocity);
        packet.put("SmoothedVelocity", robot.flywheel.getShooterVelocity());
        packet.put("AtTarget", robot.flywheel.isAtTargetVelocity());

        dashboard.sendTelemetryPacket(packet);

        // --- DRIVER TELEMETRY ---
        telemetry.addData("TurretAngle", robot.diffy.angleOffset);
        telemetry.addData("ClampedAngle", clampedAngle);
        telemetry.addData("ErrorL", errors[0]);
        telemetry.addData("ErrorR", errors[1]);
        telemetry.addData("Flywheel Target", targetVelocity);
        telemetry.addData("Flywheel Velocity", robot.flywheel.getShooterVelocity());
        telemetry.addData("Flywheel At Target", robot.flywheel.isAtTargetVelocity());
        telemetry.addData("Arm3Pos", robot.arms.getArm3Pos());
        telemetry.addData("SpatulaPos", robot.spatula.getPosition());
        telemetry.addData("POWER:", robot.flywheel.power);

        telemetry.addData("X POS:", robot.xPOS);
        telemetry.addData("Y POS:", robot.yPOS);
        telemetry.addData("HEADING:", Math.toDegrees(robot.headingRad));
        telemetry.update();
    }
}
