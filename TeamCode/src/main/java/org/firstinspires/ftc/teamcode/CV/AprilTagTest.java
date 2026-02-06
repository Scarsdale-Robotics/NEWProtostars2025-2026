package org.firstinspires.ftc.teamcode.CV;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.panels.Panels;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.control.LowPassFilter;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.apache.commons.math3.ode.nonstiff.ClassicalRungeKuttaFieldIntegrator;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import android.util.Size;


import java.util.ArrayList;
import java.util.List;
@Configurable
@TeleOp (name = "Aptest2/3")
public class AprilTagTest extends LinearOpMode {
    public CameraName cam;
    public VisionPortal vp;
    public AprilTagProcessor ap;
    private final Size CAMERA_RESOLUTION = new Size(640, 480);
    public AprilTagDetection lastTagDetected;
    public double apTag = 0;
    //public static double alpha = 0.001;
    public static double alpha = 0.0005;
    public TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
    double lastValue = 0;
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
                panelsTelemetry.addData("Proximity", xInchRadius(15, lastTagDetected));
            }
            if (getTargetTag(21,22,23) != null) {
                int motifId = getTargetTag(21,22,23).id;
                if (motifId == 21) panelsTelemetry.addLine("GPP");
                else if (motifId == 22) panelsTelemetry.addLine("PGP");
                else panelsTelemetry.addLine("PPG");
            }
            double raw = 0;
            if (lastTagDetected != null) {
                raw = lastTagDetected.ftcPose.bearing;
                apTag = lastTagDetected.ftcPose.bearing;
                if (lastValue == 0) {
                    lastValue = apTag; // initialize once
                } else {
                    lastValue = getState(apTag, lastValue);
                }
            }
            panelsTelemetry.addData("ap filtered", lastValue);
            panelsTelemetry.addData("raw", raw);
            panelsTelemetry.update(telemetry);
        }
    }
    public void detectTags(AprilTagProcessor ap) {
        List<AprilTagDetection> detections = ap.getDetections();
        if (detections != null && !detections.isEmpty()) {
            for (AprilTagDetection tag : detections) {
                if (tag.ftcPose != null) {
                    panelsTelemetry.addData("X", tag.ftcPose.x);
                    panelsTelemetry.addData("Y", tag.ftcPose.y);
                    panelsTelemetry.addData("Z", tag.ftcPose.z);
                    panelsTelemetry.addData("Range", tag.ftcPose.range);
                    panelsTelemetry.addData("Bearing", tag.ftcPose.bearing);
                    panelsTelemetry.addData("Yaw", tag.ftcPose.yaw);
                    panelsTelemetry.addData("ID", tag.id);
                    panelsTelemetry.addData("Field X", tag.robotPose.getPosition().x);
                    panelsTelemetry.addData("Field Y", tag.robotPose.getPosition().y);
                    panelsTelemetry.addData("Field Z", tag.robotPose.getPosition().z);
                    lastTagDetected = tag;
                }
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
    public double getState(double current, double last) {
        return last + alpha * (current - last);
    }
}
