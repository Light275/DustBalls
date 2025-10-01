package org.firstinspires.ftc.teamcode.OPMODEZ;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp(name = "FLYWHEEL TEST", group = "Testing")
public class FlywheelTesticle extends OpMode {
    private DcMotorEx rhinoL, rhinoR, lindexer;
    private Servo spatula, arm3;
    private ElapsedTime elapsedTime;

    private static final double CPR = 564;
    private double RPM, tgtPWRFW, tgtPWRIX;
    private double peakAmpL, peakAmpR;

    @Override public void init() {

        // Create & Reset timer elapsedTime
        elapsedTime = new ElapsedTime();
        elapsedTime.reset();

        // Initialize flywheel motors
        rhinoL = hardwareMap.get(DcMotorEx.class,"rhinoL");
        rhinoR = hardwareMap.get(DcMotorEx.class, "rhinoR");

        lindexer = hardwareMap.get(DcMotorEx.class, "lindex");
        spatula = hardwareMap.get(Servo.class, "servo");

        arm3 = hardwareMap.get(Servo.class, "back");

        // Set flywheel motor directions
        rhinoL.setDirection(DcMotorSimple.Direction.FORWARD);
        rhinoR.setDirection(DcMotorSimple.Direction.REVERSE);
        lindexer.setDirection(DcMotorSimple.Direction.FORWARD);
        rhinoL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        rhinoR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        lindexer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

    }

    @Override
    public void loop() {
        spatula.setPosition(0.565); // ENGAGED: 0.565, DISENGAGED: 0.604

        if (Math.abs(gamepad1.right_stick_y) > 0.01f) {
            tgtPWRFW += gamepad1.right_stick_y * 0.003;
        }
        if (Math.abs(gamepad1.left_stick_y) > 0.01f) {
            tgtPWRIX += gamepad1.left_stick_y * 0.003;
        }

        if (rhinoL.getCurrent(CurrentUnit.AMPS) > peakAmpL) {
            peakAmpL = rhinoL.getCurrent(CurrentUnit.AMPS);
        }
        if (rhinoR.getCurrent(CurrentUnit.AMPS) > peakAmpR) {
            peakAmpR = rhinoR.getCurrent(CurrentUnit.AMPS);
        }

        if (gamepad1.a) {
            arm3.setPosition(0.35);
        }
        if (gamepad1.b) {
            arm3.setPosition(0.6);
        }


        rhinoL.setPower(tgtPWRFW);
        rhinoR.setPower(tgtPWRFW);
        lindexer.setPower(tgtPWRIX);
        telemetry.addData("SHOOTER RPM:", -tgtPWRFW * 5800);
        telemetry.addData("SHOOTER POWER:", -tgtPWRFW);
        telemetry.addData("RhinoL amps:", rhinoL.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("RhinoR amps:", rhinoR.getCurrent(CurrentUnit.AMPS));

        telemetry.addData("Peak L:", peakAmpL);
        telemetry.addData("Peak R:", peakAmpR);
        telemetry.addLine();

        telemetry.addData("INDEXER RPM:", tgtPWRIX * 1620);
        telemetry.addData("INDEXER POWER:", tgtPWRIX);
        telemetry.update();
    }
}
