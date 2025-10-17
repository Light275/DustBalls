package org.firstinspires.ftc.teamcode.Main;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.Main.Subsystems.*;
import org.firstinspires.ftc.teamcode.Main.Utils.PoseStorage;
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

    //public Pose2d robotPose = PoseStorage.storedPose;
    public Pose2d robotPose = null;
    public double xPOS, yPOS, headingRad;

    public Robot(HardwareMap hardwareMap) {
        colorSensors = new ColorSensors(hardwareMap);
        drive = new MecanumDrive(hardwareMap, robotPose);
        drivetrain = new Drivetrain(hardwareMap);
        intake = new Intake(hardwareMap, colorSensors);

        // --- Grab the first voltage sensor in the hardware map ---
        VoltageSensor battery = hardwareMap.voltageSensor.iterator().hasNext()
                ? hardwareMap.voltageSensor.iterator().next()
                : null;

        flywheel = new Flywheel(hardwareMap, battery); // Pass battery to Flywheel
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
    }

    public void end() {
        robotPose = PoseStorage.storedPose; // set pose to stored pose
    }
}
