/*
package org.firstinspires.ftc.teamcode.Main.autos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.*;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Main.Robot;
import org.firstinspires.ftc.teamcode.MecanumDrive;

import java.lang.Math;

@Autonomous(name = "Blue 9 Ball Auto", group = "Autonomous")
public class Blue9Ball extends LinearOpMode {
    private Robot robot;

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
        FtcDashboard dashboard;

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

    // ----------------------------------------
    // FlickArmAction that holds and resets automatically
    public static class FlickArmAction implements Action {
        private final Runnable flick;
        private final Runnable reset;
        private final double holdTime;
        private double startTime = 0;
        private boolean started = false;

        public FlickArmAction(Runnable flick, Runnable reset, double holdTime) {
            this.flick = flick;
            this.reset = reset;
            this.holdTime = holdTime;
        }

        @Override
        public boolean run(com.acmerobotics.dashboard.telemetry.TelemetryPacket packet) {
            double currentTime = System.nanoTime() / 1e9;
            if (!started) {
                flick.run();
                startTime = currentTime;
                started = true;
            }
            if (currentTime - startTime > holdTime) {
                reset.run();
                return false; // done
            }
            return true; // keep looping
        }
    }

    // ----------------------------------------
    // Factory methods for fresh actions

    private FlickArmAction flickArm3Action() {
        return new FlickArmAction(() -> robot.arms.flickArm3(), () -> robot.arms.reset(), 0.35);
    }
    private RunnableAction indexerIn() { return new RunnableAction(() -> robot.intake.runIndexer(-0.8)); }
    private RunnableAction indexerInHARD() { return new RunnableAction(() -> robot.intake.runIndexer(-1)); }
    private RunnableAction indexerOut() { return new RunnableAction(() -> robot.intake.runIndexer(1)); }
    private RunnableAction intakeInAction() { return new RunnableAction(() -> robot.intake.update(true, false)); }
    private RunnableAction intakeOutAction() { return new RunnableAction(() -> robot.intake.runIntake(-1)); }
    private RunnableAction stopIndexerAction() { return new RunnableAction(() -> robot.intake.runIndexer(0)); }
    private RunnableAction stopIntakeAction() { return new RunnableAction(() -> robot.intake.runIntake(0)); }

    private RunnableAction flywheelSpinAction1() { return new RunnableAction(() -> robot.flywheel.setTargetVelocity(2100)); }
    private RunnableAction flywheelACCELERATE() { return new RunnableAction(() -> robot.flywheel.spin(1));}
    private RunnableAction flywheelSpinAction2() { return new RunnableAction(() -> robot.flywheel.setTargetVelocity(2000)); }

    private RunnableAction flywheelStopAction() { return new RunnableAction(() -> robot.flywheel.setTargetVelocity(0)); }

    private RunnableAction turret1() { return new RunnableAction(() -> robot.diffy.setAngle(38.5)); }
    private RunnableAction turret2() { return new RunnableAction(() -> robot.diffy.setAngle(40)); }
    private RunnableAction turretReset() { return new RunnableAction(() -> robot.diffy.setAngle(0)); }
    private RunnableAction turretUpdate() { return new RunnableAction(() -> robot.diffy.update()); }

    private RunnableAction gantrySlot3() { return new RunnableAction(() -> robot.diffy.goToSlot(3)); }
    private RunnableAction gantrySlot2() { return new RunnableAction(() -> robot.diffy.goToSlot(2)); }
    private RunnableAction gantrySlot1() { return new RunnableAction(() -> robot.diffy.goToSlot(1)); }
    private RunnableAction spatulaOnAction() { return new RunnableAction(() -> robot.spatula.spatulaON()); }
    private RunnableAction spatulaOffAction() { return new RunnableAction(() -> robot.spatula.spatulaOFF()); }
    private RunnableAction resetArmsAction() { return new RunnableAction(() -> robot.arms.reset()); }

    @Override
    public void runOpMode() {
        robot = new Robot(hardwareMap);
        Pose2d startPose = new Pose2d(-16.7, -56.5, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);



        // Initialize robot states
        robot.arms.reset();
        robot.spatula.spatulaON();
        robot.diffy.reset();
      //  robot.diffy.setAngle(TURRET_PRELOAD_ANGLE);
     //   robot.diffy.update();

        telemetry.addLine("Ready for start");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {

            // --- Sequence 1 --- Shoot preloads, intake first 3
            Action Sequence1 = new UpdatingAction(
                    drive.actionBuilder(startPose)
                            .afterTime(0, gantrySlot3())
                            .afterTime(0, resetArmsAction())
                            .afterTime(0, spatulaOnAction())
                            .afterTime(0, turret1())
                            .afterTime(0, indexerIn())
                            .afterTime(0, flywheelSpinAction1())

                            .afterTime(3, flickArm3Action())
                            .afterTime(5, flickArm3Action())
                            .afterTime(7, flickArm3Action())

                            .strafeToLinearHeading(new Vector2d(-16.7, -45), Math.toRadians(90),
                                    new TranslationalVelConstraint(70),
                                    new ProfileAccelConstraint(-70, 70))
                            .waitSeconds(9)
                            .build(),
                    robot
            );

            Actions.runBlocking(Sequence1);
            robot.update();
            robot.diffy.update();
        }

        // Final updates
        robot.flywheel.updateControl();
        robot.diffy.update();
        robot.update();
    }
}

 */