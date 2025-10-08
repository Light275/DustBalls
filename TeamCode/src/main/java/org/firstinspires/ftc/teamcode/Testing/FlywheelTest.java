package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Main.Subsystems.Flywheel;

@Config
@TeleOp(name = "Flywheel Proportional Test", group = "Testing")
public class FlywheelTest extends OpMode {

    private Flywheel flywheel;
    private FtcDashboard dashboard;

    // --- Dashboard tunable variables ---
    public static double kP = 0.0005;
    public static double MIN_POWER = 0.6;
    public static double targetVelocity = 2000; // ticks/sec placeholder

    private double lastTelemetryTime = 0;

    @Override
    public void init() {
        flywheel = new Flywheel(hardwareMap);
        dashboard = FtcDashboard.getInstance();

        telemetry.addLine("Flywheel Test Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        double currentTime = System.nanoTime() / 1e9;

        // --- Update flywheel velocity measurement ---
        flywheel.updateVelocity();

        // --- Update tunable constants live ---
        flywheel.setkP(kP);
        flywheel.setMinPower(MIN_POWER);

        // --- Proportional control logic ---
        double error = targetVelocity - flywheel.getShooterVelocity();
        double power = MIN_POWER + (kP * error);
        power = Math.max(MIN_POWER, Math.min(power, 1));

        // Apply to motors
        flywheel.spin(power);

        // --- Telemetry every 100ms for dashboard efficiency ---
        if (currentTime - lastTelemetryTime > 0.1) {
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("Target Velocity", targetVelocity);
            packet.put("Current Velocity", flywheel.getShooterVelocity());
            packet.put("Error", error);
            packet.put("Power", power);
            packet.put("kP", kP);
            packet.put("Min Power", MIN_POWER);
            packet.put("At Target", flywheel.isAtTargetVelocity());
            dashboard.sendTelemetryPacket(packet);

            telemetry.addData("Target Velocity", targetVelocity);
            telemetry.addData("Current Velocity", flywheel.getShooterVelocity());
            telemetry.addData("Error", error);
            telemetry.addData("Power", power);
            telemetry.addData("kP", kP);
            telemetry.addData("Min Power", MIN_POWER);
            telemetry.addData("At Target", flywheel.isAtTargetVelocity());
            telemetry.update();

            lastTelemetryTime = currentTime;
        }
    }

    @Override
    public void stop() {
        flywheel.spin(0);
    }
}
