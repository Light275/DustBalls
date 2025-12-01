/*

package org.firstinspires.ftc.teamcode.CAL.Autos;
import org.firstinspires.ftc.teamcode.CAL.Flywheel.CALFlywheelClass;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Auto", group = "Autonomous")
public class Auto extends LinearOpMode {
    private DcMotorEx flywheelMotor, intake, Frontleft, Backleft, Frontright, Backright;
    private CRServo spinner;
    ElapsedTime timer = new ElapsedTime();
    private double tpi = .2;
    private double tgtVEL;
    private boolean isactive;
    CALFlywheelClass Flywheel;
    private void brake(){
        Frontleft.setPower(0);
        Backleft.setPower(0);
        Backright.setPower(0);
        Frontright.setPower(0);

    }
    private void forward(double t,double p){
        timer.reset();
        isactive = true;
        while (timer.time() <= t) {
            Frontleft.setPower(p);
            Frontright.setPower(p);
            Backleft.setPower(p);
            Backright.setPower(p);
        }
        isactive = false;
        brake();

    }
    private void backward(double t, double p){
        timer.reset();
        isactive = true;
        while (timer.time() <= t) {
            Frontleft.setPower(-p);
            Frontright.setPower(-p);
            Backleft.setPower(-p);
            Backright.setPower(-p);
        }
        isactive = false;
        brake();
    }

    private void right(double t, double p){
        timer.reset();
        isactive = true;
        while (timer.time() <= t) {
            Frontleft.setPower(-p);
            Frontright.setPower(p);
            Backleft.setPower(-p);
            Backright.setPower(p);
        }
        isactive = false;
        brake();
    }
    private void left(double t){
        timer.reset();
        isactive = true;
        while (timer.time() <= t) {
            Frontleft.setPower(t);
            Frontright.setPower(-t);
            Backleft.setPower(t);
            Backright.setPower(-t);
        }
        isactive = false;
        brake();
    }



    private void shoot(double t){
        timer.reset();
        isactive = true;
        while (timer.time() <= t) {
            intake.setPower(1);
            spinner.setPower(1);
        }
        isactive = false;
        brake();

    }
    public void runOpMode() {
        Frontleft = hardwareMap.get(DcMotorEx.class, "leftFront");
        Backleft = hardwareMap.get(DcMotorEx.class, "leftBack");
        Frontleft = hardwareMap.get(DcMotorEx.class, "rightFront");
        Backright = hardwareMap.get(DcMotorEx.class, "rightBack");
        intake = hardwareMap.get(DcMotorEx.class, "intake");

        //   flywheelMotor = hardwareMap.get(DcMotorEx.class, "flywheel");inner;
        spinner = hardwareMap.get(CRServo.class, "spinner");

        // Set flywheel motor directions
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        Frontleft.setDirection(DcMotorSimple.Direction.REVERSE);
        Backleft.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();
        if (isStopRequested()) return;


        while (!isactive) {
             int step = 0;
            if (step == 0){
                backward(54, 1);
                step += 1;
            }
             if (step == 1){
                 tgtVEL = 320;
                 Flywheel.setTargetVelocity(tgtVEL);
                 Flywheel.update();
                 step += 1;
             }
            if (step == 2){
                shoot(5);
                step += 1;
            }

        }




    }
}

 */