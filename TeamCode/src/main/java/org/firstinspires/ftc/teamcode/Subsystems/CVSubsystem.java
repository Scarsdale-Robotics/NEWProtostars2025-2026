package org.firstinspires.ftc.teamcode.Subsystems;

import android.graphics.Camera;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.teamcode.CV.ColorDetection;
import org.firstinspires.ftc.teamcode.CV.MotifPipeline;
import org.firstinspires.ftc.vision.VisionPortal;

import android.graphics.Color;
import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.List;
import java.util.concurrent.TimeUnit;

public class CVSubsystem {
    public VisionPortal vp;
    public AprilTagProcessor aprilTagProcessor;
    private final Size CAMERA_RESOLUTION = new Size(640, 480);
    public CVSubsystem (WebcamName cameraName, LinearOpMode opMode, HardwareMap hardwareMap) {
        this.aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawCubeProjection(true)
                .build();
        this.vp = new VisionPortal.Builder()
                .setCamera(cameraName)
                .setCameraResolution(CAMERA_RESOLUTION)
                .addProcessor(aprilTagProcessor)
                .setAutoStartStreamOnBuild(true)
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .setLiveViewContainerId(hardwareMap.appContext.getResources().getIdentifier(
                        "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName()))
                .build();
        vp.setProcessorEnabled(aprilTagProcessor, true);
    }
}
