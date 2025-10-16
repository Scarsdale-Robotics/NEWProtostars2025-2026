package org.firstinspires.ftc.teamcode.CV;

import android.graphics.Canvas;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class MotifPipeline implements VisionProcessor {
    public String sequence;
    Rect ch1 = new Rect(50, 120, 20, 20);
    Rect ch2 = new Rect(100, 120, 20, 20);
    Rect ch3 = new Rect(150, 120, 20, 20);
    //TODO: tune values for accurate detection
    Rect image = new Rect(500,500,20,20);
    public Scalar upper = new Scalar(150, 255, 255);
    public Scalar lower = new Scalar(125, 50, 50);

    @Override
    public void init(int width, int height, CameraCalibration calibration) {

    }

    @Override
    public Mat processFrame(Mat img, long captureTimeNanoSeconds) {
        Mat cropped = img.submat(image);
        Mat hsv = new Mat();
        Imgproc.cvtColor(cropped, hsv, Imgproc.COLOR_RGB2HSV);

        boolean first = checkRegion(hsv, ch1, lower, upper, 0.8);
        boolean second = checkRegion(hsv, ch2, lower, upper, 0.8);
        boolean third = checkRegion(hsv, ch3, lower, upper, 0.8);
        sequence = getMotif(first, second, third);
        Imgproc.putText(
                img,
                "Seq: " + sequence,
                new Point(50, 50),
                Imgproc.FONT_HERSHEY_SIMPLEX,
                0.5,
                new Scalar(255, 0, 0),
                2
        );

        return cropped;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }
    public String motif() {
        return sequence;
    }
    public String getMotif(boolean one, boolean two, boolean three) {
        StringBuilder sb = new StringBuilder();
        sb.append(one ? "P" : "G");
        sb.append(two ? "P" : "G");
        sb.append(three ? "P" : "G");
        return sb.toString();
    }

    public boolean checkRegion(Mat src, Rect roi, Scalar lower, Scalar upper, double minFraction) {
        Mat submat = src.submat(roi);
        Mat mask = new Mat();
        Core.inRange(submat, lower, upper, mask);
        int totalpixels = roi.width * roi.height;
        int nonzero = Core.countNonZero(mask);
        return ((double) nonzero / totalpixels) >= minFraction;
    }
}
