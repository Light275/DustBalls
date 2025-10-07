package org.firstinspires.ftc.teamcode.Main.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Main.Robot;
import org.firstinspires.ftc.teamcode.Main.Subsystems.Diffy;

@Config
@TeleOp(name = "TELEOP Turret Fixed Angle Dashboard", group = "Competitions")
public class TELEOP extends OpMode {

    private Robot robot;
    private FtcDashboard dashboard;

    public static double fixedTurretAngle = 0; // Dashboard tunable

    private static final double MIN_ANGLE = 0;
    private static final double MAX_ANGLE = 180;

    @Override
    public void init() {
        robot = new Robot(hardwareMap);
        dashboard = FtcDashboard.getInstance();

        robot.arms.reset();
        robot.diffy.reset();

        telemetry.addLine("Robot Initialized");
        telemetry.addLine("Holding fixed turret angle (Dashboard)");
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

        // --- ARM3 ---
        boolean shootButton = gamepad1.a;
        if (shootButton && robot.diffy.atTarget()) {
            robot.arms.flickArm3(true);
            robot.intake.runIndexer(robot.intake.getIndexerSpeed());
        } else {
            robot.arms.flickArm3(false);
        }

        // --- TURRET HOLD ---
        double clampedAngle = Range.clip(fixedTurretAngle, MIN_ANGLE, MAX_ANGLE);
        robot.diffy.goToSlot(3);
        robot.diffy.setAngle(clampedAngle);
        robot.diffy.update();

        // --- UPDATE ROBOT ---
        robot.update();

        double[] errors = robot.diffy.getErrors();
        // --- DASHBOARD ---
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("TargetL", Diffy.targetL);
        packet.put("TargetR", Diffy.targetR);
        packet.put("EncoderL", robot.diffy.encL.getCurrentPosition());
        packet.put("EncoderR", robot.diffy.encR.getCurrentPosition());
        packet.put("ErrorL", errors[0]);
        packet.put("ErrorR", errors[1]);
        packet.put("Angle", robot.diffy.angleOffset);
        dashboard.sendTelemetryPacket(packet);

        // --- TELEMETRY ---

        telemetry.addData("ErrorL", errors[0]);
        telemetry.addData("ErrorR", errors[1]);
        telemetry.addData("Angle", robot.diffy.angleOffset);
        telemetry.update();
    }
}
