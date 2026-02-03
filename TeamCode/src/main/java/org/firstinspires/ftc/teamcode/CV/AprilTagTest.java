package org.firstinspires.ftc.teamcode.CV;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.apache.commons.math3.ode.nonstiff.ClassicalRungeKuttaFieldIntegrator;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import android.util.Size;

import java.util.List;
@TeleOp (name = "Aptest2/3")
public class AprilTagTest extends LinearOpMode {
    public CameraName cam;
    public VisionPortal vp;
    public AprilTagProcessor ap;
    private final Size CAMERA_RESOLUTION = new Size(640, 480);
    public AprilTagDetection lastTagDetected;
    public static double apTag = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        this.cam = hardwareMap.get(CameraName.class, "Webcam 1");
        this.ap = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawCubeProjection(true)
                .build();
        this.vp = new VisionPortal.Builder()
                .setCamera(cam)
                .enableLiveView(true)
                .setCameraResolution(CAMERA_RESOLUTION)
                .addProcessor(ap)
                .setAutoStartStreamOnBuild(true)
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .setLiveViewContainerId(hardwareMap.appContext.getResources().getIdentifier(
                        "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName()))
                .build();
        vp.setProcessorEnabled(ap, true);
        waitForStart();
        while (opModeIsActive()) {
            detectTags(ap);
            if (lastTagDetected != null) {
                telemetry.addData("Proximity", xInchRadius(15, lastTagDetected));
            }
            if (getTargetTag(21,22,23) != null) {
                int motifId = getTargetTag(21,22,23).id;
                if (motifId == 21) telemetry.addLine("GPP");
                else if (motifId == 22) telemetry.addLine("PGP");
                else telemetry.addLine("PPG");
            }
            if (lastTagDetected != null) {
                apTag = lastTagDetected.ftcPose.range;
            }
            telemetry.addData("AP Value", apTag);
            telemetry.update();
        }
    }
    public void detectTags(AprilTagProcessor ap) {
        List<AprilTagDetection> detections = ap.getDetections();
        if (detections != null && !detections.isEmpty()) {
            for (AprilTagDetection tag : detections) {
                telemetry.addData("X", tag.ftcPose.x);
                telemetry.addData("Y", tag.ftcPose.y);
                telemetry.addData("Z", tag.ftcPose.z);
                telemetry.addData("Range", tag.ftcPose.range);
                telemetry.addData("Bearing", tag.ftcPose.bearing);
                telemetry.addData("Yaw", tag.ftcPose.yaw);
                telemetry.addData("ID", tag.id);
                telemetry.addData("Field X", tag.robotPose.getPosition().x);
                telemetry.addData("Field Y", tag.robotPose.getPosition().y);
                telemetry.addData("Field Z", tag.robotPose.getPosition().z);
                lastTagDetected = tag;
            }
        } else {
            lastTagDetected = null;
        }
    }
    public AprilTagDetection getTargetTag(int targetId1, int targetId2, int targetId3) {
        List<AprilTagDetection> detections = ap.getDetections();
        AprilTagDetection sol = null;
        for (AprilTagDetection tag : detections) {
            if (tag.id == targetId1 || tag.id == targetId2 || tag.id == targetId3) {
                sol = tag;
                break;
            }
        }
        return sol;
    }
    public boolean xInchRadius(int radius, AprilTagDetection target) {
        return target.ftcPose.range <= radius;
    }
}
