package org.firstinspires.ftc.teamcode.Main.Utils;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.List;

public class TagPoseProcessor {

    private AprilTagProcessor goalTag;
    private VisionPortal visionPortal;

    private Pose2d relativePose = new Pose2d(0, 0, 0);
    private Pose2d globalPose = new Pose2d(0, 0, 0);

    // Camera pose
    private final Position cameraPosition = new Position(DistanceUnit.INCH, 4.875, 7.5, 14.25, 0);
    private final YawPitchRollAngles cameraOrientation =
            new YawPitchRollAngles(AngleUnit.DEGREES, 0, -80.5, 180, 0);

    public TagPoseProcessor(HardwareMap hardwareMap) {
        initAprilTag(hardwareMap);
    }

    private void initAprilTag(HardwareMap hardwareMap) {
        goalTag = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagOutline(true)
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .setCameraPose(cameraPosition, cameraOrientation)
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

    public void update() {
        List<AprilTagDetection> detections = goalTag.getDetections();
        if (detections.isEmpty()) return;

        for (AprilTagDetection detection : detections) {
            if (detection.metadata != null && detection.id == 20) {

                double relX = detection.ftcPose.y;
                double relY = -detection.ftcPose.x;
                double relYaw = detection.ftcPose.yaw; // degrees
                relativePose = new Pose2d(relX, relY, Math.toRadians(relYaw));

                VectorF tagPos = detection.metadata.fieldPosition;
                Quaternion tagQ = detection.metadata.fieldOrientation;

                double tagX = tagPos.get(0) / 25.4; // mm -> in
                double tagY = tagPos.get(1) / 25.4;
                double tagYawDeg = (tagQ != null) ? getYawFromQuaternion(tagQ) : 144;

                globalPose = toGlobalPose(tagX, tagY, tagYawDeg, relX, relY, relYaw);

                TelemetryPacket packet = new TelemetryPacket();
                packet.put("Robot X", globalPose.position.x);
                packet.put("Robot Y", globalPose.position.y);
                packet.put("Robot Heading", Math.toDegrees(globalPose.heading.toDouble()));
                FtcDashboard.getInstance().sendTelemetryPacket(packet);
            }
        }
    }

    public Pose2d getTagPose() {
        return globalPose;
    }

    public void close() {
        visionPortal.close();
    }

    private static double getYawFromQuaternion(Quaternion q) {
        double w = q.w;
        double x = q.x;
        double y = q.y;
        double z = q.z;

        double yaw = Math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y*y + z*z));
        return Math.toDegrees(yaw);
    }

    private static Pose2d toGlobalPose(double tagX, double tagY, double tagYawDeg,
                                       double relX, double relY, double relYawDeg) {
        double tagYawRad = Math.toRadians(tagYawDeg);
        double relYawRad = Math.toRadians(relYawDeg);

        double cos = Math.cos(tagYawRad - relYawRad);
        double sin = Math.sin(tagYawRad - relYawRad);

        double globalY = tagX - (relX * cos - relY * sin) - 60;
        double globalX = tagY - (relX * sin + relY * cos) + 37;
        double globalHeading = normalizeRadians(tagYawRad + relYawRad);

        return new Pose2d(-globalX, -globalY, globalHeading);
    }

    private static double normalizeRadians(double a) {
        while (a >= Math.PI) a -= 2.0 * Math.PI;
        while (a < -Math.PI) a += 2.0 * Math.PI;
        return a;
    }
}
