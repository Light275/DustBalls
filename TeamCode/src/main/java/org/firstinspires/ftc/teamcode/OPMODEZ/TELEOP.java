package org.firstinspires.ftc.teamcode.OPMODEZ;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "TELEOP", group = "Competitions")
public class TELEOP extends OpMode {

    private DcMotorEx leftFront, leftBack, rightFront, rightBack, intake, indexer;
    private DcMotor shooterL, shooterR;
    private Servo arm1, arm2, arm3, spatula;
    private CRServo diffyL, diffyR;

    public static final double ARM_1_UP = 0.405;
    public static final double ARM_1_DOWN = 0.195;
    public static final double ARM_2_UP = 0.55;
    public static final double ARM_2_DOWN = 0.34;
    public static final double ARM_3_UP = 0.55;
    public static final double ARM_3_DOWN = 0.34;

    public static final double SPATULA_ON = 0.565;
    public static final double SPATULA_OFF = 0.604;

    @Override public void init() {

        leftFront = hardwareMap.get(DcMotorEx.class,"leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class,"leftBack");
        rightFront = hardwareMap.get(DcMotorEx.class,"rightBack");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightFront");

        indexer = hardwareMap.get(DcMotorEx.class, "indexer");
        intake = hardwareMap.get(DcMotorEx.class, "intake");

        shooterL = hardwareMap.get(DcMotorEx.class, "shooterL");
        shooterR = hardwareMap.get(DcMotorEx.class, "shooterR");

        arm1 = hardwareMap.get(Servo.class, "arm1");
        arm2 = hardwareMap.get(Servo.class, "arm2");
        arm3 = hardwareMap.get(Servo.class, "arm3");

        spatula = hardwareMap.get(Servo.class, "spatula");

        diffyL = hardwareMap.get(CRServo.class, "diffyL");
        diffyR = hardwareMap.get(CRServo.class, "diffyR");

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        indexer.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        indexer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        indexer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        shooterL.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterR.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        // INIT ACTUATORS
        spatula.setPosition(SPATULA_ON); // ENGAGED: 0.565, DISENGAGED: 0.604
        arm1.setPosition(ARM_1_DOWN);
        arm2.setPosition(ARM_2_DOWN);
        arm3.setPosition(ARM_3_DOWN);
    }

    public void intake(boolean gamepadInputIn, boolean gamepadInputOut, double intakeSpeed, double idxSpeed, double idxIdle) {
        if (gamepadInputIn) {
            intake.setPower(intakeSpeed);
            indexer.setPower(idxSpeed);
        } else if (gamepadInputOut) {
            intake.setPower(-intakeSpeed);
            indexer.setPower(-idxSpeed);
        } else {
            intake.setPower(0);
            indexer.setPower(idxIdle);
        }
    } // TODO: make indexer constant

    public void drivetrain () {
        double drive = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x * 0.75;
        double rotate = gamepad1.right_stick_x * 0.75;
        double speedMultiplier = gamepad1.right_trigger > 0.1 ? 0.3 : 1.0;

        leftFront.setPower(Range.clip((drive + strafe + rotate) * speedMultiplier, -1.0, 1.0));
        leftBack.setPower(Range.clip((drive - strafe + rotate) * speedMultiplier, -1.0, 1.0));
        rightFront.setPower(Range.clip((drive + strafe - rotate) * speedMultiplier, -1.0, 1.0));
        rightBack.setPower(Range.clip((drive - strafe - rotate) * speedMultiplier, -1.0, 1.0));
    }

    public void diffyMove(){
        double lateral = gamepad2.left_stick_y;
        double rotary = gamepad2.right_stick_x;
        double lPower = lateral - rotary;
        double rPower = -lateral - rotary;

        if (Math.abs(lPower) > 1) { // Clip L Power
            double divisor = lPower;
            lPower = Range.clip(lPower,-1, 1); // Map lPower to Max +- 1
            rPower = rPower / divisor;

        } else if (Math.abs(rPower) > 1) { // Clip R Power
            double divisor = rPower;
            rPower = Range.clip(rPower, -1, 1); // Map rPower to Max +- 1
            lPower = lPower / divisor;
        }

        diffyL.setPower(lPower);
        diffyR.setPower(rPower);
    }

    @Override
    public void loop() {

        intake(gamepad1.left_bumper, gamepad1.dpad_down, 0.90, 0.75, 0.1);
        drivetrain();
        diffyMove();

        telemetry.addData("FLYWHEEL TICKS L:", shooterL.getCurrentPosition());
        telemetry.addData("FLYWHEEL TICKS R", shooterR.getCurrentPosition());
        telemetry.update();


    }
}
