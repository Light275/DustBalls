package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Main.Robot;

@TeleOp(name = "Flywheel PID Dashboard Tuner", group = "Tests")
@Config
public class FlywheelPIDTest extends OpMode {

    private Robot robot;

    // Manual target velocity (ticks/sec)
    public static double targetVelocity = 0;

    // PID coefficients live-tunable from Dashboard
    public static double kP = 0.0008;
    public static double kI = 0.00001;
    public static double kD = 0.00005;

    private FtcDashboard dashboard;

    // Increment per loop for joystick control
    public static double velocityIncrement = 50;
    private static final double DEADZONE = 0.05;

    @Override
    public void init() {
        robot = new Robot(hardwareMap);
        dashboard = FtcDashboard.getInstance();

        // Enable PID for testing
        robot.flywheel.enablePID();

        telemetry.addLine("Flywheel PID Dashboard Tuner Ready");
        telemetry.addLine("Left stick up/down increments target velocity");
        telemetry.update();
    }

    @Override
    public void loop() {
        // --- Incremental joystick control ---
        double stickY = -gamepad1.left_stick_y; // up = positive
        if (Math.abs(stickY) > DEADZONE) {
            targetVelocity += stickY * velocityIncrement;
            targetVelocity = Math.max(0, targetVelocity);       // min 0
            targetVelocity = Math.min(7000, targetVelocity);    // max reasonable
        }

        // --- Update PID coefficients from Dashboard ---
        robot.flywheel.setPIDCoefficients(kP, kI, kD);
        robot.flywheel.setVelocityTolerance(50);

        // --- Update flywheel velocity ---
        robot.flywheel.updateVelocity(); // refresh measured velocity

        // --- Get PID power and spin motors ---
        double pidPower = robot.flywheel.getPIDPower(targetVelocity);
        robot.flywheel.spin(pidPower);

        // --- Dashboard telemetry ---
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Target Velocity", targetVelocity);
        packet.put("Current Velocity", robot.flywheel.getShooterVelocity());
        packet.put("PID Output", pidPower);
        packet.put("kP", kP);
        packet.put("kI", kI);
        packet.put("kD", kD);
        dashboard.sendTelemetryPacket(packet);

        // --- Standard telemetry ---
        telemetry.addData("Target Vel", targetVelocity);
        telemetry.addData("Current Vel", robot.flywheel.getShooterVelocity());
        telemetry.addData("PID Power", pidPower);
        telemetry.addData("kP", kP);
        telemetry.addData("kI", kI);
        telemetry.addData("kD", kD);
        telemetry.update();
    }
}
