package org.firstinspires.ftc.teamcode.OPMODEZ;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Drawing;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;

import java.util.List;

@TeleOp(name = "Tag Pose Processor", group = "Competitions")
public class TagPoseProcessor extends LinearOpMode {

    private AprilTagProcessor goalTag;
    private VisionPortal visionPortal;

    private Pose2d relativePose = new Pose2d(0, 0, 0);
    private Pose2d globalPose   = new Pose2d(0, 0, 0);

    private final Position cameraPosition = new Position(DistanceUnit.INCH,
            0, 0, 23, 0);

    private final YawPitchRollAngles cameraOrientation =
            new YawPitchRollAngles(AngleUnit.DEGREES, 0, -90, 0, 0);

    @Override
    public void runOpMode() {
        initAprilTag();
        waitForStart();

        while (opModeIsActive()) {
            processAprilTag();
        }

        visionPortal.close();
    }

    private void initAprilTag() {
        goalTag = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagOutline(true)
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .setCameraPose(cameraPosition, cameraOrientation)
                // C270 Intrinsics
                .setLensIntrinsics(822.317f, 822.317f, 319.495f, 242.502f)
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setStreamFormat(VisionPortal.StreamFormat.MJPEG);
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        builder.setCameraResolution(new Size(640, 480));
        builder.addProcessor(goalTag);
        visionPortal = builder.build();

        FtcDashboard.getInstance().startCameraStream(visionPortal, 0);
    }

    private void processAprilTag() {
        List<AprilTagDetection> detections = goalTag.getDetections();

        TelemetryPacket packet = new TelemetryPacket();
        packet.fieldOverlay().setStroke("#3F51B5");

        if (detections.isEmpty()) {
            packet.put("Status", "No tag detected");
        } else {
            boolean foundTarget = false;

            for (AprilTagDetection detection : detections) {
                if (detection.metadata != null && detection.id == 24) { // lock onto tag ID 20
                    foundTarget = true;

                    // Relative pose (robot wrt tag)
                    double relX = detection.ftcPose.y;
                    double relY = -detection.ftcPose.x;
                    double relYaw = detection.ftcPose.yaw; // deg
                    relativePose = new Pose2d(relX, relY, Math.toRadians(relYaw));

                    // Distance from camera to tag
                    double distance = Math.hypot(relX, relY);

                    // Tag global pose
                    VectorF tagPos = detection.metadata.fieldPosition;   // mm
                    Quaternion tagQ = detection.metadata.fieldOrientation;

                    // Extract yaw from quaternion if available
                    double tagYawDeg = 144; // default if null
                    if (tagQ != null) {
                        tagYawDeg = getYawFromQuaternion(tagQ, AngleUnit.DEGREES);
                    }

                    double tagX = tagPos.get(0) / 25.4; // convert mm→in
                    double tagY = tagPos.get(1) / 25.4;

                    // Transform into field coordinates
                    globalPose = toGlobalPose(tagX, tagY, tagYawDeg, relX, relY, relYaw);

                    // Dashboard telemetry
                    packet.put("tagYawDeg", tagYawDeg);
                    packet.put("rel X", relX);
                    packet.put("rel Y", relY);
                    packet.put("rel Yaw", relYaw);
                    packet.put("Robot X (in)", globalPose.position.x);
                    packet.put("Robot Y (in)", globalPose.position.y);
                    packet.put("Robot Heading (deg)", Math.toDegrees(globalPose.heading.toDouble()));
                    packet.put("Distance to Tag (in)", distance);

                    Drawing.drawRobot(packet.fieldOverlay(), globalPose);
                    Drawing.drawRobot(packet.fieldOverlay(), relativePose);
                }
            }

            if (!foundTarget) {
                packet.put("Status", "No target tag (ID 20) detected");
            }
        }

        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }

    /**
     * Extract yaw (rotation around Z) from a Quaternion.
     */
    private static double getYawFromQuaternion(Quaternion q, AngleUnit unit) {
        if (q == null) return 0; // null-safe
        double w = q.w;
        double x = q.x;
        double y = q.y;
        double z = q.z;

        // Yaw (Z axis rotation)
        double yaw = Math.atan2(2.0 * (w * z + x * y),
                1.0 - 2.0 * (y * y + z * z));

        if (unit == AngleUnit.DEGREES) {
            yaw = Math.toDegrees(yaw);
        }
        return yaw;
    }

    /**
     * Convert tag-relative pose → global field pose.
     */
    private static Pose2d toGlobalPose(
            double tagFieldX, double tagFieldY,
            double tagYawDeg,
            double relX, double relY,
            double relYawDeg) {

        double tagYawRad = Math.toRadians(tagYawDeg);
        double relYawRad = Math.toRadians(relYawDeg);

        double cos = Math.cos(tagYawRad - relYawRad);
        double sin = Math.sin(tagYawRad - relYawRad);

        double globalX = tagFieldX - (relX * cos - relY * sin);
        double globalY = tagFieldY - (relX * sin + relY * cos);

        double globalHeading = normalizeRadians(tagYawRad + relYawRad);

        return new Pose2d(globalY, -globalX, -globalHeading);
    }

    private static double normalizeRadians(double a) {
        while (a >= Math.PI) a -= 2.0 * Math.PI;
        while (a < -Math.PI) a += 2.0 * Math.PI;
        return a;
    }
}
