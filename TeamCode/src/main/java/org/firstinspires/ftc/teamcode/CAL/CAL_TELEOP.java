package org.firstinspires.ftc.teamcode.CAL;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.CAL.Flywheel.CALFlywheelClass;

@TeleOp(name = "Nolan is amazing", group = "Testing")
public class CAL_TELEOP extends OpMode {
    private DcMotorEx flywheelMotor, intake, leftFront, leftBack, rightFront, rightBack;
    private CRServo spinner;

    private double prevTicks;

    private static final double CPR = 564;
    private double RPM, intakePWR;
    private double peakAmps;
    private double loopCounter = 0;

    private double tgtVEL = 250;
    private double loopTime;
    private double flyspeed = 1;


    CALFlywheelClass flywheel;


    @Override public void init() { // INIT BUTTON PRESSED
        VoltageSensor battery = hardwareMap.voltageSensor.iterator().next();
        flywheel = new CALFlywheelClass(hardwareMap, battery);

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
     //   flywheelMotor = hardwareMap.get(DcMotorEx.class, "flywheel");

        spinner = hardwareMap.get(CRServo.class, "spinner");

        // Set flywheel motor directions
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
      //  flywheelMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    //    flywheelMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

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
        boolean isdown = false;
        move(drive, strafe, rotate, slowMode);

        if (Math.abs(gamepad2.right_stick_y) > 0.01f) {
            tgtVEL += gamepad2.right_stick_y * -flyspeed;
        }

        else if (!gamepad2.a){
            intakePWR = 0;
        } else {
            intakePWR = 1; // goof
        }
        if (gamepad2.left_trigger > 0.1f && flywheel.getTargetVelocity() < 6000 && !isdown) {
            flywheel.setTargetVelocity(flywheel.getTargetVelocity()+1000);
            isdown = true;
            while (isdown){
                if(gamepad2.left_trigger < 0.1f)  {
                    isdown = false;
                    break;
                }
            }
        }
        if (gamepad2.dpad_up){
        isdown = true;
            if (isdown && !gamepad2.dpad_up){
            flywheel.setTargetVelocity(2500);
            isdown = false;
            }
        }
        if (gamepad2.dpad_up){
            isdown = true;
            if (isdown && !gamepad2.dpad_up){
                flywheel.setTargetVelocity(2500);
                isdown = false;
            }
        }
        if (gamepad2.right_trigger > 0.1f && flywheel.getVelocityRPM() >= 1000){
            isdown = true;
            if (isdown && gamepad2.right_trigger < 0.1f){
                flywheel.setTargetVelocity(flywheel.getTargetVelocity()-1000);
                isdown = false;
            }

        }
        else if (!gamepad2.a){
            intakePWR = 0;
        }
        if (gamepad2.left_bumper && flywheel.getTargetVelocity() < 6000) {
            isdown = true;
            if (isdown && !gamepad2.left_bumper){
                flywheel.setTargetVelocity(flywheel.getTargetVelocity()+100);
                isdown = false;
            }
        }
        else if (gamepad2.right_bumper && flywheel.getVelocityRPM() >= 100){
            isdown = true;
            if (isdown && !gamepad2.right_bumper){
                flywheel.setTargetVelocity(flywheel.getTargetVelocity()-100);
                isdown = false;
            }
        }
        else {
            intakePWR = 1; // goof
        }

        intake.setPower(intakePWR);

        if (gamepad2.a) {
            spinner.setPower(-1);
            intake.setPower(intakePWR);
        }
        else {
            spinner.setPower(0);
        }

        flywheel.setTargetVelocity(tgtVEL);
        flywheel.update();

        telemetry.addData("TARGET VELOCITY:", flywheel.getTargetVelocity());
        telemetry.addLine();
        telemetry.addData("SHOOTER VELOCITY:", flywheel.getVelocityRadPerSec());
        telemetry.addData("SHOOTER RPM:", flywheel.getVelocityRPM());
        telemetry.addData("SHOOTER POWER:", flywheel.getPower());
        telemetry.addLine();
        telemetry.addLine();
        telemetry.addData("INTAKE PWR:", intakePWR);
        telemetry.update();

        TelemetryPacket packet = new TelemetryPacket();
        packet.put("tgtVEL:", tgtVEL);
        packet.put("Current Vel:", flywheel.getVelocityRadPerSec());
        packet.put("ESTIMATED RPM:", tgtVEL * 9.549297);

        FtcDashboard.getInstance().sendTelemetryPacket(packet);


    }
}
