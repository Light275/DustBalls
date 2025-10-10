package org.firstinspires.ftc.teamcode.Main.autos;

import com.acmerobotics.roadrunner.*;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Main.Robot;
import org.firstinspires.ftc.teamcode.MecanumDrive;

import java.lang.Math;

@Autonomous(name = "Blue 9 Ball Auto", group = "Autonomous")
public class Blue9Ball extends LinearOpMode {

    private Robot robot;
    private ElapsedTime gameClock;

    // ----------------------------------------
    // Runnable -> Action wrapper
    public static class RunnableAction implements Action {
        private final Runnable runnable;
        private boolean executed = false;

        public RunnableAction(Runnable runnable) {
            this.runnable = runnable;
        }

        @Override
        public boolean run(com.acmerobotics.dashboard.telemetry.TelemetryPacket packet) {
            if (!executed) {
                runnable.run();
                executed = true;
            }
            return false;
        }
    }

    // ----------------------------------------
    // Updating wrapper for continuous robot updates
    public static class UpdatingAction implements Action {
        private final Action inner;
        private final Robot robot;

        public UpdatingAction(Action inner, Robot robot) {
            this.inner = inner;
            this.robot = robot;
        }

        @Override
        public boolean run(com.acmerobotics.dashboard.telemetry.TelemetryPacket packet) {
            robot.diffy.update();
            robot.flywheel.updateControl();
            robot.flywheel.updateVelocity();
            robot.update();
            return inner.run(packet);
        }
    }

    @Override
    public void runOpMode() {
        robot = new Robot(hardwareMap);
        Pose2d startPose = new Pose2d(-16.7, -56.5, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        telemetry.addLine("Ready for start");
        telemetry.update();

        // --- Actions ---
        Action indexerUp = new RunnableAction(() -> robot.intake.runIndexer(-1));
        Action intakeIn = new RunnableAction(() -> robot.intake.runIntake(1));
        Action indexerStop = new RunnableAction(() -> robot.intake.runIndexer(0));
        Action intakeStop = new RunnableAction(() -> robot.intake.runIntake(0));
        Action flywheelSpin = new RunnableAction(() -> robot.flywheel.setTargetVelocity(2700));
        Action flywheelStop = new RunnableAction(() -> robot.flywheel.setTargetVelocity(0));
        Action turretPreload = new RunnableAction(() -> robot.diffy.setAngle(65));
        Action gantryPreload = new RunnableAction(() -> robot.diffy.goToSlot(2));
        Action spatulaOut = new RunnableAction(() -> robot.spatula.spatulaON());
        Action spatulaIn = new RunnableAction(() -> robot.spatula.spatulaOFF());
        Action flickArm1 = new RunnableAction(() -> robot.arms.flickArm1());
        Action flickArm2 = new RunnableAction(() -> robot.arms.flickArm2());
        Action flickArm3 = new RunnableAction(() -> robot.arms.flickArm3());

        waitForStart();

        if (opModeIsActive()) {
            // --- Main trajectory with proper updates ---
            Action Sequence1 = new UpdatingAction(
                    drive.actionBuilder(startPose)
                            .afterTime(0, flywheelSpin)
                            .afterTime(0, turretPreload)
                            .afterTime(0, indexerUp)
                            .afterTime(2, flickArm3)
                            .afterTime(4, indexerStop)
                            .afterTime(4, flywheelStop)
                            .afterTime(4, intakeIn)
                            .waitSeconds(3)
                            .strafeToLinearHeading(new Vector2d(-20, -50), Math.toRadians(90),
                                    new TranslationalVelConstraint(40),
                                    new ProfileAccelConstraint(-40, 40))
                            .strafeToLinearHeading(new Vector2d(-30, -28.5), Math.toRadians(180),
                                    new TranslationalVelConstraint(40),
                                    new ProfileAccelConstraint(-40, 40))
                            .strafeToLinearHeading(new Vector2d(-50, -28.5), Math.toRadians(180),
                                    new TranslationalVelConstraint(40),
                                    new ProfileAccelConstraint(-40, 40))
                            .build(),
                    robot
            );

            Actions.runBlocking(Sequence1);

        }

        robot.flywheel.updateControl();
        robot.diffy.update();
        robot.update();
    }
}
