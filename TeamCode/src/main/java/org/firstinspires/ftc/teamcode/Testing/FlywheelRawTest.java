package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Raw Flywheel TeleOp", group = "Tests")
public class FlywheelRawTest extends OpMode {

    private DcMotorEx shooterL, shooterR;
    private FtcDashboard dashboard;

    @Override
    public void init() {
        shooterL = hardwareMap.get(DcMotorEx.class, "shooterL");
        shooterR = hardwareMap.get(DcMotorEx.class, "shooterR");

        shooterL.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterR.setDirection(DcMotorSimple.Direction.REVERSE);

        shooterL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        shooterR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        shooterL.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        shooterR.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        dashboard = FtcDashboard.getInstance();

        telemetry.addLine("Raw Flywheel TeleOp Ready");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Use left stick Y to control motor power
        double power = -gamepad1.left_stick_y; // up = positive
        power = Math.max(-1, Math.min(1, power)); // clamp

        shooterL.setPower(power);
        shooterR.setPower(power);

        // Send telemetry to FTC Dashboard
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Raw Power", power);
        dashboard.sendTelemetryPacket(packet);

        // Standard telemetry
        telemetry.addData("Raw Power", power);
        telemetry.update();
    }
}
