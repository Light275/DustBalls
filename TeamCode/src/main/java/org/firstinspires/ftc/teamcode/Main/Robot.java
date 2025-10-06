package org.firstinspires.ftc.teamcode.Main;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Main.Subsystems.Arms;
import org.firstinspires.ftc.teamcode.Main.Subsystems.ColorSensors;
import org.firstinspires.ftc.teamcode.Main.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Main.Subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.Main.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.MecanumDrive;

public class Robot {

    public Flywheel flywheel;
    public MecanumDrive drive;
    public ColorSensors colorSensors;
    public Drivetrain drivetrain;
    public Intake intake;
    public Arms arms;

    public Robot(HardwareMap hardwareMap) {
        // Initialize subsystems
        colorSensors = new ColorSensors(hardwareMap);
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        drivetrain = new Drivetrain(hardwareMap);
        intake = new Intake(hardwareMap, colorSensors);
        flywheel = new Flywheel(hardwareMap, this);
        arms = new Arms(hardwareMap); // Arm1/2/3 initialized to DOWN
    }

    public void update() {
        drive.updatePoseEstimate();
        colorSensors.update();
        flywheel.update(colorSensors.numBalls());
        arms.update(); // apply flick flags
    }
}
