package org.firstinspires.ftc.teamcode.OPMODEZ;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp(name = "FLYWHEEL TEST", group = "Testing")
public class FlywheelTesticle extends OpMode {
    private DcMotorEx rhinoL, rhinoR;
    private ElapsedTime elapsedTime;

    private static final double CPR = 564;
    private double RPM, tgtPWR;
    private double peakAmpL, peakAmpR;

    @Override public void init() {

        // Create & Reset timer elapsedTime
        elapsedTime = new ElapsedTime();
        elapsedTime.reset();

        // Initialize flywheel motors
        rhinoL = hardwareMap.get(DcMotorEx.class,"rhinoL");
        rhinoR = hardwareMap.get(DcMotorEx.class, "rhinoR");

        // Set flywheel motor directions
        rhinoL.setDirection(DcMotorSimple.Direction.FORWARD);
        rhinoR.setDirection(DcMotorSimple.Direction.REVERSE);
        rhinoL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        rhinoR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

    }

    @Override
    public void loop() {
        if (Math.abs(gamepad1.right_stick_y) > 0.1f) {
            tgtPWR += gamepad1.right_stick_y * 0.003;
        }

        if (rhinoL.getCurrent(CurrentUnit.AMPS) > peakAmpL) {
            peakAmpL = rhinoL.getCurrent(CurrentUnit.AMPS);
        }
        if (rhinoR.getCurrent(CurrentUnit.AMPS) > peakAmpR) {
            peakAmpR = rhinoR.getCurrent(CurrentUnit.AMPS);
        }


        rhinoL.setPower(tgtPWR);
        rhinoR.setPower(tgtPWR);
        telemetry.addData("EST. RPM:", -tgtPWR * 5800);
        telemetry.addData("POWER:", -tgtPWR);
        telemetry.addData("RhinoL amps:", rhinoL.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("RhinoR amps:", rhinoR.getCurrent(CurrentUnit.AMPS));

        telemetry.addLine();
        telemetry.addData("Peak L:", peakAmpL);
        telemetry.addData("Peak R:", peakAmpR);
        telemetry.update();
    }
}
