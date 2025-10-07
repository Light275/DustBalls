package org.firstinspires.ftc.teamcode.Main;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Main.Subsystems.Arms;
import org.firstinspires.ftc.teamcode.Main.Subsystems.ColorSensors;
import org.firstinspires.ftc.teamcode.Main.Subsystems.Diffy;
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
    public Diffy diffy;

    public Robot(HardwareMap hardwareMap) {
        // Initialize subsystems
        colorSensors = new ColorSensors(hardwareMap);
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        drivetrain = new Drivetrain(hardwareMap);
        intake = new Intake(hardwareMap, colorSensors);
        flywheel = new Flywheel(hardwareMap); // can leave PID off for testing
        arms = new Arms(hardwareMap);
        diffy = new Diffy(hardwareMap);       // Diffy turret subsystem
    }

    /**
     * Call every loop to update subsystems.
     * Important: Diffy update should only be called from OpMode to prevent conflicts.
     */
    public void update() {
        // Update drivetrain & pose
        drive.updatePoseEstimate();

        // Update arms
        arms.update();

        // Update intake with default inputs (can be overridden by OpMode)
        intake.update(false, false);

        // Diffy NOT automatically updated here — leave it to TeleOp for precise control
        // flywheel.updateVelocity(); // optionally update flywheel if needed
        // colorSensors.update();     // optionally update color sensors if needed
    }
}
