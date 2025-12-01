package org.firstinspires.ftc.teamcode.CAL.Autos;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.*;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.CONFIG.RobotConfig;
import org.firstinspires.ftc.teamcode.Main.Robot;
import org.firstinspires.ftc.teamcode.Main.Utils.PoseStorage;
import org.firstinspires.ftc.teamcode.Main.Utils.TagProcessor;
import org.firstinspires.ftc.teamcode.MecanumDrive;

import java.lang.Math;

@Autonomous(name = "Pinpoint Auto", group = "Autonomous")
public class PinpointAuto extends LinearOpMode {


        // TODO: =======================================================================================
        @Override
        public void runOpMode() {
            Pose2d startPose = new Pose2d(-52, 51, Math.toRadians(240));
            MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);




            waitForStart();
            if (opModeIsActive()) {


                // TODO: ============================== Auto Sequence ============================================

                Actions.runBlocking(drive.actionBuilder(startPose) // DEPOSIT PRELOAD AND INTAKE FIRST GROUND SEQUENCE

                        .strafeToLinearHeading(new Vector2d(15, -15), Math.toRadians(225), // BUCKET POSITION
                                new TranslationalVelConstraint(40),
                                new ProfileAccelConstraint(-40, 40))
                        .waitSeconds(1.5)
                        .build());



        }
    }
}