package org.firstinspires.ftc.teamcode.Main;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Main.Subsystems.Arms;
import org.firstinspires.ftc.teamcode.Main.Subsystems.ColorSensors;
import org.firstinspires.ftc.teamcode.Main.Subsystems.Diffy;
import org.firstinspires.ftc.teamcode.Main.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Main.Subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.Main.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Main.Subsystems.Spatula;
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

    public double xPOS, yPOS, headingRad;

    public Robot(HardwareMap hardwareMap) {
        // Initialize subsystems
        colorSensors = new ColorSensors(hardwareMap);
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, Math.toRadians(90)));
        drivetrain = new Drivetrain(hardwareMap);
        intake = new Intake(hardwareMap, colorSensors);
        flywheel = new Flywheel(hardwareMap);
        arms = new Arms(hardwareMap);
        diffy = new Diffy(hardwareMap);
        spatula = new Spatula(hardwareMap);
    }

    /** Call every loop to update subsystems. */
    public void update() {
        drive.updatePoseEstimate();
        xPOS = drive.localizer.getPose().position.x;
        yPOS = drive.localizer.getPose().position.y;
        headingRad = drive.localizer.getPose().heading.toDouble();
        arms.update();

        // Keep spatula always ON
        spatula.spatulaON();
    }
}
