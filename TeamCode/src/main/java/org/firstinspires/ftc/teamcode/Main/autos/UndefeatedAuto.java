package org.firstinspires.ftc.teamcode.Main.autos;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.*;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.CONFIG.RobotConfig;
import org.firstinspires.ftc.teamcode.Main.Robot;
import org.firstinspires.ftc.teamcode.Main.Utils.TagProcessor;
import org.firstinspires.ftc.teamcode.MecanumDrive;

import java.lang.Math;

@Autonomous(name = "9 Ball Auto (Blue/Red Selectable)", group = "Autonomous")
public class UndefeatedAuto extends LinearOpMode {

    private Robot robot;
    private MecanumDrive drive;
    private TagProcessor tagProcessor;
    private int nextGantry = 0;
    private int detectedTag = 21;


    // -------------------------------
    // Reusable Action Wrappers
    // -------------------------------

    /** One-shot runnable wrapper */
    public static class RunnableAction implements Action {
        private final Runnable runnable;
        private boolean done = false;

        public RunnableAction(Runnable runnable) {
            this.runnable = runnable;
        }

        @Override
        public boolean run(TelemetryPacket packet) {
            if (!done) {
                runnable.run();
                done = true;
            }
            return false;
        }
    }

    /** Wrapper that ensures robot updates continuously */
    public static class UpdatingAction implements Action {
        private final Action inner;
        private final Robot robot;

        public UpdatingAction(Action inner, Robot robot) {
            this.inner = inner;
            this.robot = robot;
        }

        @Override
        public boolean run(TelemetryPacket packet) {
            robot.diffy.update();
            robot.update();
            robot.flywheel.update();
            return inner.run(packet);
        }
    }

    /** Flick arm for some time, then reset */
    public static class TimedFlickAction implements Action {
        private final Runnable flick;
        private final Runnable reset;
        private final double duration;
        private double startTime = 0;
        private boolean started = false;

        public TimedFlickAction(Runnable flick, Runnable reset, double duration) {
            this.flick = flick;
            this.reset = reset;
            this.duration = duration;
        }

        @Override
        public boolean run(TelemetryPacket packet) {
            double now = System.nanoTime() / 1e9;
            if (!started) {
                flick.run();
                startTime = now;
                started = true;
            }
            if (now - startTime > duration) {
                reset.run();
                return false;
            }
            return true;
        }
    }

    // -------------------------------
    // Action Factory Methods
    // -------------------------------

    private RunnableAction runIndexer(double power) { return new RunnableAction(() -> robot.intake.runIndexerOnly(power)); }
    private RunnableAction runIntake(double power) { return new RunnableAction(() -> robot.intake.runIntakeOnly(power)); }
    private RunnableAction spinFlywheel(double radPerSec) { return new RunnableAction(() -> robot.flywheel.setTargetVelocity(radPerSec)); }
    private RunnableAction turretAngle(double angleDeg) { return new RunnableAction(() -> robot.diffy.setAngle(angleDeg)); }
    private RunnableAction gantrySlot(int slot) { return new RunnableAction(() -> robot.diffy.goToSlot(slot)); }
    private RunnableAction spatulaOn() { return new RunnableAction(() -> robot.spatula.spatulaON()); }
    private RunnableAction spatulaOff() { return new RunnableAction(() -> robot.spatula.spatulaOFF()); }
    private RunnableAction resetArms() { return new RunnableAction(() -> robot.arms.reset()); }

    // -------------------------------
    // NEXT GANTRY FEATURE
    // -------------------------------
    private RunnableAction setNextGantry(int slot) { return new RunnableAction(() -> this.nextGantry = slot); }
    private RunnableAction goToNextGantrySlot() { return new RunnableAction(() -> robot.diffy.goToSlot(nextGantry)); }

    private TimedFlickAction flickArm3() { return new TimedFlickAction(() -> robot.arms.flickArm3(), () -> robot.arms.reset(), 0.35); }
    private TimedFlickAction flickArm2() { return new TimedFlickAction(() -> robot.arms.flickArm2(), () -> robot.arms.reset(), 0.35); }
    private TimedFlickAction flickArm1() { return new TimedFlickAction(() -> robot.arms.flickArm1(), () -> robot.arms.reset(), 0.35); }

    // -------------------------------
    // MAIN AUTO
    // -------------------------------

    @Override
    public void runOpMode() {
        Pose2d startPose;

        robot = new Robot(hardwareMap);
        tagProcessor = new TagProcessor(hardwareMap);
        robot.arms.reset();
        robot.diffy.reset();
        robot.spatula.spatulaON();

        // --- INIT LOOP ---
        while (opModeInInit()) {
            tagProcessor.handleInitLoop(this);
        }

        if (tagProcessor.getDetectedTagId() != -1) {
            detectedTag = tagProcessor.getDetectedTagId();
        }

        TagProcessor.Alliance alliance = tagProcessor.getAlliance();

        // Apply alliance to RobotConfig for rest of match
        RobotConfig.alliance = (alliance == TagProcessor.Alliance.BLUE)
                ? RobotConfig.Alliance.BLUE
                : RobotConfig.Alliance.RED;

        if (RobotConfig.alliance == RobotConfig.Alliance.BLUE) { // BLUE START POSE
            startPose = new Pose2d(-16.7, -56.5, Math.toRadians(90));
        } else { // RED START POSE
            startPose = new Pose2d(16.7, -56.5, Math.toRadians(90));
        }

        drive = new MecanumDrive(hardwareMap, startPose);

        telemetry.addLine("=== FINAL INIT DATA ===");
        telemetry.addData("Alliance", RobotConfig.alliance);
        telemetry.addData("Detected Tag", (detectedTag == -1) ? "None" : detectedTag);
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // -------------------------------
        // BLUE SEQUENCES
        // -------------------------------
        Action blue_PGP_preloads = new UpdatingAction(
                drive.actionBuilder(startPose)
                        .afterTime(0.0, spatulaOn())
                        .afterTime(0.0, gantrySlot(2))
                        .afterTime(0.0, spinFlywheel(434))
                        .afterTime(0.0, turretAngle(74))
                        .afterTime(0.0, runIndexer(-0.7))
                        .afterTime(0.8, flickArm2())
                        .afterTime(1.25, gantrySlot(3))
                        .afterTime(1.25, runIndexer(-1.0))
                        .afterTime(1.75, flickArm3())
                        .afterTime(2.75, flickArm3())
                        .afterTime(0, setNextGantry(2)) // Next Gantry Pos after PGP Preload
                        .waitSeconds(3.25)
                        .build(),
                robot
        );

        Action blue_PPG_preloads = new UpdatingAction(
                drive.actionBuilder(startPose)
                        .afterTime(0.0, spatulaOn())
                        .afterTime(0.0, gantrySlot(2))
                        .afterTime(0.0, spinFlywheel(434))
                        .afterTime(0.0, turretAngle(74))
                        .afterTime(0.0, runIndexer(-1.0))
                        .afterTime(0.8, flickArm2())
                        .afterTime(1.5, flickArm2())
                        .afterTime(1.9, gantrySlot(3))
                        .afterTime(2.4, flickArm3())
                        .afterTime(0, setNextGantry(2)) // Next Gantry Pos after PPG Preload
                        .waitSeconds(2.75)
                        .build(),
                robot
        );

        Action blue_GPP_preloads = new UpdatingAction(
                drive.actionBuilder(startPose)
                        .afterTime(0.0, spatulaOn())
                        .afterTime(0.0, gantrySlot(3))
                        .afterTime(0.0, spinFlywheel(434))
                        .afterTime(0.0, turretAngle(74))
                        .afterTime(0.0, runIndexer(-1.0))
                        .afterTime(0.8, flickArm3())
                        .afterTime(1.5, flickArm3())
                        .afterTime(2.2, flickArm3())
                        .afterTime(0, setNextGantry(3)) // Next Gantry Pos after GPP Preload
                        .waitSeconds(2.75)
                        .build(),
                robot
        );

        // TODO: STACK INTAKE SEQUENCES
        Action blue_firstStackIntake = new UpdatingAction(
                drive.actionBuilder(startPose)
                        .afterTime(0.0, spatulaOn())
                        .afterTime(0.0, goToNextGantrySlot())
                        .afterTime(0.0, spinFlywheel(120))
                        .afterTime(0.5, turretAngle(20))
                        .afterTime(3, turretAngle(73))
                        .afterTime(0.0, runIndexer(1.0))
                        .afterTime(0.0, runIntake(-1.0))
                        .afterTime(1.5, runIntake(1.0))
                        .afterTime(1.5, runIndexer(-1.0))
                        .afterTime(4, runIntake(0))
                        .afterTime(4, spinFlywheel(434))
                        .strafeToLinearHeading(new Vector2d(-30, -36), Math.toRadians(180),
                                new TranslationalVelConstraint(60),
                                new ProfileAccelConstraint(-60, 60))
                        .setReversed(true)
                        .splineToConstantHeading(new Vector2d(-54, -36), Math.toRadians(180),
                                new TranslationalVelConstraint(60),
                                new ProfileAccelConstraint(-60, 60))
                        .strafeToLinearHeading(new Vector2d(-16.7, -54), Math.toRadians(90),
                                new TranslationalVelConstraint(30),
                                new ProfileAccelConstraint(-30, 30))
                        .build(),
                robot
        );

        // TODO: ==================================================================================================
        // RED SEQUENCES
        // -------------------------------
        Action red_PGP_preloads = new UpdatingAction(
                drive.actionBuilder(startPose)
                        .afterTime(0.0, spatulaOn())
                        .afterTime(0.0, gantrySlot(2))
                        .afterTime(0.0, spinFlywheel(434))
                        .afterTime(0.0, turretAngle(106))
                        .afterTime(0.0, runIndexer(-0.7))
                        .afterTime(0.8, flickArm2())
                        .afterTime(1.25, gantrySlot(3))
                        .afterTime(1.25, runIndexer(-1.0))
                        .afterTime(1.75, flickArm3())
                        .afterTime(2.75, flickArm3())
                        .waitSeconds(3.25)
                        .build(),
                robot
        );

        Action red_PPG_preloads = new UpdatingAction(
                drive.actionBuilder(startPose)
                        .afterTime(0.0, spatulaOn())
                        .afterTime(0.0, gantrySlot(2))
                        .afterTime(0.0, spinFlywheel(434))
                        .afterTime(0.0, turretAngle(106))
                        .afterTime(0.0, runIndexer(-1.0))
                        .afterTime(0.8, flickArm2())
                        .afterTime(1.5, flickArm2())
                        .afterTime(1.9, gantrySlot(3))
                        .afterTime(2.4, flickArm3())
                        .waitSeconds(2.75)
                        .build(),
                robot
        );

        Action red_GPP_preloads = new UpdatingAction(
                drive.actionBuilder(startPose)
                        .afterTime(0.0, spatulaOn())
                        .afterTime(0.0, gantrySlot(3))
                        .afterTime(0.0, spinFlywheel(434))
                        .afterTime(0.0, turretAngle(106))
                        .afterTime(0.0, runIndexer(-1.0))
                        .afterTime(0.8, flickArm3())
                        .afterTime(1.5, flickArm3())
                        .afterTime(2.2, flickArm3())
                        .waitSeconds(2.75)
                        .build(),
                robot
        );


        // TODO: SHOOTING SEQUENCES [133/223/233/333] 4 possible efficient combinations ==================================
        Action shoot_133 = new UpdatingAction(
                drive.actionBuilder(drive.localizer.getPose())
                        .afterTime(0.0, spatulaOff())
                        .afterTime(0.0, gantrySlot(1))
                        .afterTime(0.0, runIndexer(-0.7))
                        .afterTime(0.8, flickArm1())
                        .afterTime(1.25, gantrySlot(3))
                        .afterTime(1.25, spatulaOn())
                        .afterTime(1.25, runIndexer(-1.0))
                        .afterTime(2, flickArm3())
                        .afterTime(3, flickArm3())
                        .waitSeconds(3.4)
                        .build(),
                robot
        );

        Action shoot_223 = new UpdatingAction(
                drive.actionBuilder(drive.localizer.getPose())
                        .afterTime(0.0, spatulaOn())
                        .afterTime(0.0, gantrySlot(2))
                        .afterTime(0.0, runIndexer(-1.0))
                        .afterTime(0.8, flickArm2())
                        .afterTime(1.5, flickArm2())
                        .afterTime(1.9, gantrySlot(3))
                        .afterTime(2.4, flickArm3())
                        .waitSeconds(2.75) // wait to keep action sequence in instance to not power down flywheel too soon
                        .build(),
                robot
        );

        Action shoot_233 = new UpdatingAction(
                drive.actionBuilder(drive.localizer.getPose())
                        .afterTime(0.0, spatulaOn())
                        .afterTime(0.0, gantrySlot(2))
                        .afterTime(0.0, runIndexer(-0.7))
                        .afterTime(0.8, flickArm2())
                        .afterTime(1.25, gantrySlot(3))
                        .afterTime(1.25, runIndexer(-1.0))
                        .afterTime(1.75, flickArm3())
                        .afterTime(2.75, flickArm3())
                        .waitSeconds(3.25)
                        .build(),
                robot
        );

        Action shoot_333 = new UpdatingAction(
                drive.actionBuilder(drive.localizer.getPose())
                        .afterTime(0.0, spatulaOn())
                        .afterTime(0.0, gantrySlot(3))
                        .afterTime(0.0, runIndexer(-1.0))
                        .afterTime(0.8, flickArm3())
                        .afterTime(1.5, flickArm3())
                        .afterTime(2.2, flickArm3())
                        .waitSeconds(2.65)
                        .build(),
                robot
        );


        // -------------------------------
        // * RUN SEQUENCES BASED ON TAG *
        // - ID 21: GPP; 333/333/233/133
        // - ID 22: PGP; 233/233/333/223
        // - ID 23: PPG; 223/223/133/333
        // -------------------------------
        if (RobotConfig.alliance == RobotConfig.Alliance.BLUE) {
            if (detectedTag == 21) Actions.runBlocking(blue_GPP_preloads);
            else if (detectedTag == 22) Actions.runBlocking(blue_PGP_preloads);
            else if (detectedTag == 23) Actions.runBlocking(blue_PPG_preloads);
            else Actions.runBlocking(blue_GPP_preloads);

            // After preloads are run, intake first stack
            Actions.runBlocking(blue_firstStackIntake);

            // 2nd shooting opportunity in sequence
            if (detectedTag == 21) Actions.runBlocking(shoot_333);
            else if (detectedTag == 22) Actions.runBlocking(shoot_233);
            else if (detectedTag == 23) Actions.runBlocking(shoot_223);
            else Actions.runBlocking(shoot_333);




        } else if (RobotConfig.alliance == RobotConfig.Alliance.RED) {
            if (detectedTag == 21) Actions.runBlocking(red_GPP_preloads);
            else if (detectedTag == 22) Actions.runBlocking(red_PGP_preloads);
            else if (detectedTag == 23) Actions.runBlocking(red_PPG_preloads);
            else Actions.runBlocking(red_GPP_preloads);
        }

        // Final updates
        robot.update();
        robot.flywheel.update();
        robot.diffy.update();
        tagProcessor.close();

        Pose2d finalRestingPlace = drive.localizer.getPose();
    }
}
