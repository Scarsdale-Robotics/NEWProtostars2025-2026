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

import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.List;
import java.util.concurrent.TimeUnit;

public class CVSubsystem {
    public MotifPipeline pipeline;
    public VisionPortal vp;
    public AprilTagProcessor aprilTagProcessor;
    public ColorDetection colorDetection;
    private final Size CAMERA_RESOLUTION = new Size(640, 480);
    public CVSubsystem (WebcamName cameraName, LinearOpMode opMode, int viewcontainerid) {
        this.aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawCubeProjection(true)
                .build();
        this.vp = new VisionPortal.Builder()
                .setCamera(cameraName)
                .enableLiveView(true)
                .setCameraResolution(CAMERA_RESOLUTION)
                .addProcessor(pipeline)
                .addProcessor(aprilTagProcessor)
                .setAutoStartStreamOnBuild(true)
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .setLiveViewContainerId(viewcontainerid)
                .build();
        vp.setProcessorEnabled(pipeline, true);
        vp.setProcessorEnabled(colorDetection, true);
        vp.setProcessorEnabled(aprilTagProcessor, true);
        while ((opMode.opModeInInit() || opMode.opModeIsActive()) && vp.getCameraState() != VisionPortal.CameraState.STREAMING);

        updateExposure(vp);
    }
    public void updateExposure(VisionPortal visionPortal) {
        ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
        exposureControl.setMode(ExposureControl.Mode.Manual);
        exposureControl.setExposure(15, TimeUnit.MILLISECONDS);  // exposure may have to be adjusted during competitions
    }
    public String Motif() {
        return pipeline.motif();
    }
    public List<Integer> getPixels() {
        return colorDetection.colors();
    }
}
