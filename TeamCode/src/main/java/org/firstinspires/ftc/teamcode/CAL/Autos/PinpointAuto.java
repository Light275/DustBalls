package org.firstinspires.ftc.teamcode.CAL.Autos;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.*;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.CAL.Flywheel.CALFlywheelClass;
import org.firstinspires.ftc.teamcode.CONFIG.RobotConfig;
import org.firstinspires.ftc.teamcode.Main.Robot;
import org.firstinspires.ftc.teamcode.Main.Utils.PoseStorage;
import org.firstinspires.ftc.teamcode.Main.Utils.TagProcessor;
import org.firstinspires.ftc.teamcode.MecanumDrive;

import java.lang.Math;

@Autonomous(name = "Pinpoint Auto", group = "Autonomous")
public class PinpointAuto extends LinearOpMode {

    CALFlywheelClass flywheel;

    public static class UpdatingAction implements Action {
        private final Action inner;
        private final CALFlywheelClass robot;

        public UpdatingAction(Action inner, CALFlywheelClass robot) {
            this.inner = inner;
            this.robot = robot;
        }

        @Override
        public boolean run(TelemetryPacket packet) {
            robot.update();
            return inner.run(packet);
        }
    }

        // TODO: =======================================================================================
        @Override
        public void runOpMode() {
            Pose2d startPose = new Pose2d(-52, 51, Math.toRadians(330));
            MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

            flywheel = new CALFlywheelClass(hardwareMap);


            waitForStart();
            if (opModeIsActive()) {


                // TODO: ============================== Auto Sequence ============================================

                Actions.runBlocking(drive.actionBuilder(startPose) // DEPOSIT PRELOAD AND INTAKE FIRST GROUND SEQUENCE

                        .strafeToLinearHeading(new Vector2d(-15, 15), Math.toRadians(315), // BUCKET POSITION
                                new TranslationalVelConstraint(40),
                                new ProfileAccelConstraint(-40, 40))
                        .waitSeconds(1.5)
                        .build());
                double counter = 0;
                while (counter == 0) {
                    flywheel.setTargetVelocity(300);
                    flywheel.update();
                    if (flywheel.getVelocityRadPerSec() > 300) {
                        counter ++;
                    }
                }


        }
    }
}