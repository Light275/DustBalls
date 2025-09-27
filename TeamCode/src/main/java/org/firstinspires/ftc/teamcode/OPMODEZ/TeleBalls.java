package org.firstinspires.ftc.teamcode.OPMODEZ;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Drawing;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;

import java.util.List;

@TeleOp(name = "TELEOP", group = "Competitions")
public class TeleBalls extends LinearOpMode {

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private double dist;
    Pose2d pose = new Pose2d(0, 0, 0);

    private Position cameraPosition = new Position(DistanceUnit.INCH,
            0, 0, 20, 0);
    private YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
            0, -90, 0, 0);

    private static double normalizeRadians(double a) {
        if (a >= Math.PI) a -= 2.0 * Math.PI;
        if (a < -Math.PI) a += 2.0 * Math.PI;
        return a;
    }

    @Override
    public void runOpMode() {

        initAprilTag();
        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {

                telemetry.addData("x", pose.position.x);
                telemetry.addData("y", pose.position.y);
                telemetry.addData("heading (deg)", Math.toDegrees(pose.heading.toDouble()));

                telemetryAprilTag();

                TelemetryPacket packet = new TelemetryPacket();
                packet.fieldOverlay().setStroke("#3F51B5");
                packet.put("pose", pose);

                Drawing.drawRobot(packet.fieldOverlay(), pose);

                FtcDashboard.getInstance().sendTelemetryPacket(packet);
                telemetry.addData("DIST",dist);
                telemetry.update();

                /*
                if (gamepad1.dpad_down) {
                    visionPortal.stopStreaming();
                } else if (gamepad1.dpad_up) {
                    visionPortal.resumeStreaming();
                }
                */
            }
        }

        // Save more CPU resources when camera is no longer needed.
        visionPortal.close();
    }   // end method runOpMode()

    /**
     * Initialize the AprilTag processor.
     */
    // normalize to [-PI, PI)


    /**
     * Map a tag-relative pose to a field/global Pose2d.
     *
     * @param tagFieldX     Tag X position in field coordinates (inches)
     * @param tagFieldY     Tag Y position in field coordinates (inches)
     * @param tagYawDeg     Tag orientation in field coordinates (degrees)
     *                       (interpretation: the rotation that maps the tag's forward axis into field axes)
     * @param relX          Robot X relative to tag (inches). AprilTag SDK: X is "right"
     * @param relY          Robot Y relative to tag (inches). AprilTag SDK: Y is "forward"
     * @param relYawDeg     Robot yaw relative to tag (degrees). AprilTag SDK usually reports yaw in degrees.
     * @return Pose2d in field coordinates (x, y in inches, heading in radians)
     */
    public static Pose2d GlobalPose(
            double tagFieldX, double tagFieldY,
            double tagYawDeg,
            double relX, double relY,
            double relYawDeg) {

        double tagYawRad = Math.toRadians(tagYawDeg);
        double relYawRad = Math.toRadians(relYawDeg);

        // rotate relative vector into field frame
        double cos = Math.cos(tagYawRad);
        double sin = Math.sin(tagYawRad);

        double globalX = tagFieldX + (relX * cos - relY * sin);
        double globalY = tagFieldY + (relX * sin + relY * cos);

        // robot heading in field frame
        double globalHeading = normalizeRadians(tagYawRad + relYawRad);

        return new Pose2d(globalX, globalY, globalHeading);
    }



    private void initAprilTag() {

        aprilTag = new AprilTagProcessor.Builder()
                // Optional settings:
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagOutline(true)
                //.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                //.setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .setCameraPose(cameraPosition, cameraOrientation)
                // If no lens intrinsics are set, SDK uses default calibration if available
                .setLensIntrinsics(822.317f, 822.317f, 319.495f, 242.502f)

                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setStreamFormat(VisionPortal.StreamFormat.MJPEG);

        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));

        builder.setCameraResolution(new Size(640, 480));
        builder.addProcessor(aprilTag);
        visionPortal = builder.build();

        // Stream camera to FTC Dashboard
        FtcDashboard.getInstance().startCameraStream(visionPortal, 0);
    }


    private void telemetryAprilTag() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());
        // Step through the detections and display info.


        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                double x = detection.ftcPose.x;    // in inches
                double y = detection.ftcPose.y;    // in inches
                double t = detection.ftcPose.yaw; // convert degrees → radians
                dist = Math.sqrt((Math.pow(x, 2) + Math.pow(y, 2)));

                pose = new Pose2d(x, y, t);
                GlobalPose(68.84, -69.6, 54, x,y,t);

                telemetry.addLine(String.format("\n==== (ID %d) %s",
                        detection.id, detection.metadata.name));
            }
        }

       // telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
    }   // end method telemetryAprilTag()

}   // end class
