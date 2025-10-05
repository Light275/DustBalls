package org.firstinspires.ftc.teamcode.OPMODEZ;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp(name = "Nolan is amazing", group = "Testing")
public class Nolanpooki extends OpMode {
    private DcMotorEx flywheelMotor;
  //  private ElapsedTime rpmTimer;

    private double prevTicks;

    private static final double CPR = 564;
    private double RPM, tgtPWR;
    private double peakAmps;

    private double loopTime;

    @Override public void init() { // INIT BUTTON PRESSED
      //  ElapsedTime rpmTimer = new ElapsedTime();
        flywheelMotor = hardwareMap.get(DcMotorEx.class, "duzty");

        // Set flywheel motor directions
        flywheelMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        flywheelMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

    }

    @Override
    public void loop() { // PLAY BUTTON PRESSED, CODE RUNNING

        if (Math.abs(gamepad1.right_stick_y) > 0.01f) {
            tgtPWR += gamepad1.right_stick_y * -0.003;
        }

        if (flywheelMotor.getCurrent(CurrentUnit.AMPS) > peakAmps) {
            peakAmps = flywheelMotor.getCurrent(CurrentUnit.AMPS);
        }

       // loopTime = rpmTimer.time();

      //  RPM = (60/loopTime) * flywheelMotor.getCurrentPosition() - prevTicks;
      //  prevTicks = flywheelMotor.getCurrentPosition();


        flywheelMotor.setPower(tgtPWR);


     //   rpmTimer.reset();


        telemetry.addData("SHOOTER RPM:", -tgtPWR * 5800);
        telemetry.addData("SHOOTER POWER:", -tgtPWR);
        telemetry.addData("Flywheel amps:", flywheelMotor.getCurrent(CurrentUnit.AMPS));

        telemetry.addData("Peak Amps:", peakAmps);
        telemetry.addLine();
        telemetry.update();

        TelemetryPacket packet = new TelemetryPacket();
        packet.put("tgtPWR:", tgtPWR);
        packet.put("ESTIMATED RPM:", tgtPWR * 5800);
        packet.put("ACTUAL RPM:", RPM);
        packet.put("Flywheel amps:", flywheelMotor.getCurrent(CurrentUnit.AMPS));
        packet.put("Peak amps:", peakAmps);
        packet.put("Loop Time", loopTime);

        FtcDashboard.getInstance().sendTelemetryPacket(packet);

       // rpmTimer.reset();
    }
}
