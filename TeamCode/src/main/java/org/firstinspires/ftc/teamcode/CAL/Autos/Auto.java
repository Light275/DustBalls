package org.firstinspires.ftc.teamcode.CAL.Autos;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.*;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.CAL.Flywheel.CALFlywheelClass;
import org.firstinspires.ftc.teamcode.CONFIG.RobotConfig;
import org.firstinspires.ftc.teamcode.Main.Robot;
import org.firstinspires.ftc.teamcode.Main.Utils.PoseStorage;
import org.firstinspires.ftc.teamcode.Main.Utils.TagProcessor;
import org.firstinspires.ftc.teamcode.MecanumDrive;

import java.lang.Math;

@Autonomous(name = "Auto", group = "Autonomous")
public class Auto extends LinearOpMode {
    private DcMotorEx flywheelMotor, intake, Frontleft, Backleft, Frontright, Backright;
    ElapsedTime timer  = new ElapsedTime();
    private double tpi = .2;

    private void brake(){
        Frontleft.setPower(0);
        Backleft.setPower(0);
        Backright.setPower(0);
        Frontright.setPower(0);

    }
    private void forward(double t,double p){
        timer.reset();
        while (timer.time() <= t) {
            Frontleft.setPower(p);
            Frontright.setPower(p);
            Backleft.setPower(p);
            Backright.setPower(p);
        }
        brake();

    }
    private void backward(double t, double p){
        timer.reset();
        while (timer.time() <= t) {
            Frontleft.setPower(-p);
            Frontright.setPower(-p);
            Backleft.setPower(-p);
            Backright.setPower(-p);
        }
        brake();
    }
    private void right(double t, double p){
        timer.reset();
        while (timer.time() <= t) {
            Frontleft.setPower(-p);
            Frontright.setPower(p);
            Backleft.setPower(-p);
            Backright.setPower(p);
        }
        brake();
    }
    private void left(double t){
        timer.reset();
        while (timer.time() <= t) {
            Frontleft.setPower(t);
            Frontright.setPower(-t);
            Backleft.setPower(t);
            Backright.setPower(-t);
        }
        brake();
    }
    public void runOpMode() {

        Frontleft = hardwareMap.get(DcMotorEx.class, "leftFront");
        Backleft = hardwareMap.get(DcMotorEx.class, "leftBack");
        Frontleft = hardwareMap.get(DcMotorEx.class, "rightFront");
        Backright = hardwareMap.get(DcMotorEx.class, "rightBack");

        Frontleft.setDirection(DcMotorSimple.Direction.REVERSE);
        Backleft.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();
        if (isStopRequested()) return;
        // forward two seconds
        timer.reset();
        while (timer.time() <= 2){
            Frontleft.setPower(1);
            Frontright.setPower(1);
            Backright.setPower(1);
            Backleft.setPower(1);
        }

        backward(54 * tpi, 1);






    }
}