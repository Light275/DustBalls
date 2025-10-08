package org.firstinspires.ftc.teamcode.CAL;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@Disabled
@TeleOp(name = "Nolan is amazing", group = "Testing")
public class Nolanpooki extends OpMode {
    private DcMotorEx flywheelMotor, intake, leftFront, leftBack, rightFront, rightBack;
    private CRServo spinner;
  //  private ElapsedTime rpmTimer;

    private double prevTicks;

    private static final double CPR = 564;
    private double RPM, intakePWR;
    private double peakAmps;
    private double loopCounter = 0;

    private double tgtPWR = 0.87;
    private double loopTime;


    @Override public void init() { // INIT BUTTON PRESSED


        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightBack");  // intentionally swapped
        rightBack = hardwareMap.get(DcMotorEx.class, "rightFront");  // intentionally swapped

        // Directions — KEEP SAME
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intake = hardwareMap.get(DcMotorEx.class, "intake");
        flywheelMotor = hardwareMap.get(DcMotorEx.class, "flywheel");
        spinner = hardwareMap.get(CRServo.class, "spinner");

        // Set flywheel motor directions
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        flywheelMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        flywheelMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

    }

    public void move(double drive, double strafe, double rotate, boolean slowMode) {
        double speedMultiplier = slowMode ? 0.3 : 1.0;

        double lf = drive + strafe + rotate;
        double lb = drive - strafe + rotate;
        double rf = drive + strafe - rotate;
        double rb = drive - strafe - rotate;

        // Normalize to prevent ratio distortion
        double max = Math.max(1.0, Math.abs(lf));
        max = Math.max(max, Math.abs(lb));
        max = Math.max(max, Math.abs(rf));
        max = Math.max(max, Math.abs(rb));

        lf /= max;
        lb /= max;
        rf /= max;
        rb /= max;

        leftFront.setPower(lf * speedMultiplier);
        leftBack.setPower(lb * speedMultiplier);
        rightFront.setPower(rf * speedMultiplier);
        rightBack.setPower(rb * speedMultiplier);
    }

    @Override
    public void loop() { // PLAY BUTTON PRESSED, CODE RUNNING
        double drive = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;
        boolean slowMode = gamepad1.right_trigger > 0.1;
        move(drive, strafe, rotate, slowMode);

        if (Math.abs(gamepad2.right_stick_y) > 0.01f) {
            tgtPWR += gamepad2.right_stick_y * -0.0005;
        }

        if (gamepad2.left_trigger > 0.1f) {
            intakePWR = -1;
        } else if (gamepad2.left_bumper){
            intakePWR = 1;
        } else if (!gamepad2.a){
            intakePWR = 0;
        }

        intake.setPower(intakePWR);

        if (gamepad2.a) {
            spinner.setPower(-1);
            intake.setPower(intakePWR);
        } else {
            spinner.setPower(0);
        }

        if (flywheelMotor.getCurrent(CurrentUnit.AMPS) > peakAmps) {
            peakAmps = flywheelMotor.getCurrent(CurrentUnit.AMPS);
        }

        flywheelMotor.setPower(tgtPWR);

        telemetry.addData("SHOOTER RPM:", tgtPWR * 5800);
        telemetry.addData("SHOOTER POWER:", tgtPWR);
        telemetry.addLine();
        telemetry.addLine();
        telemetry.addData("Flywheel amps:", flywheelMotor.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Peak Amps:", peakAmps);
        telemetry.addLine();
        telemetry.addLine();
        telemetry.addData("INTAKE PWR:", intakePWR);
        telemetry.update();

        TelemetryPacket packet = new TelemetryPacket();
        packet.put("tgtPWR:", tgtPWR);
        packet.put("ESTIMATED RPM:", tgtPWR * 5800);
        packet.put("Flywheel amps:", flywheelMotor.getCurrent(CurrentUnit.AMPS));
        packet.put("Peak amps:", peakAmps);

        FtcDashboard.getInstance().sendTelemetryPacket(packet);


    }
}
