package org.firstinspires.ftc.teamcode.Main;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Main.Subsystems.*;
import org.firstinspires.ftc.teamcode.Main.Utils.PoseFusion;
import org.firstinspires.ftc.teamcode.Main.Utils.TagPoseProcessor;
import org.firstinspires.ftc.teamcode.MecanumDrive;

public class Robot {

    public Flywheel flywheel;
    public MecanumDrive drive;
    public ColorSensors colorSensors;
    public Drivetrain drivetrain;
    public Intake intake;
    public Arms arms;
    public Diffy diffy;
    public Spatula spatula;

    public Pose2d calibratedPose = new Pose2d(7.3, -56.5, Math.toRadians(90));
    public double xPOS, yPOS, headingRad;

    public Robot(HardwareMap hardwareMap) {
        colorSensors = new ColorSensors(hardwareMap);
        drive = new MecanumDrive(hardwareMap, calibratedPose);
        drivetrain = new Drivetrain(hardwareMap);
        intake = new Intake(hardwareMap, colorSensors);
        flywheel = new Flywheel(hardwareMap);
        arms = new Arms(hardwareMap);
        diffy = new Diffy(hardwareMap);
        spatula = new Spatula(hardwareMap);
    }

    public void update() {
        drive.updatePoseEstimate();
        Pose2d odoPose = drive.localizer.getPose();

        xPOS = odoPose.position.x;
        yPOS = odoPose.position.y;
        headingRad = odoPose.heading.toDouble();

        arms.update();
        spatula.spatulaON();
    }
}
