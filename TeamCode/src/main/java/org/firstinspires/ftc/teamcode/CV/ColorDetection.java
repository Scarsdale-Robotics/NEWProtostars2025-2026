package org.firstinspires.ftc.teamcode.CV;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.Arrays;
import java.util.List;
import java.util.stream.LongStream;

public class ColorDetection implements VisionProcessor {
    int greenpixels = 0;
    int bluepixels = 0;
    int yellowpixels = 0;
    int redpixels = 0;
    // YELLOW
    Scalar lowerYellow = new Scalar(20, 100, 100);
    Scalar upperYellow = new Scalar(30, 255, 255);

    // GREEN
    Scalar lowerGreen = new Scalar(40, 70, 70);
    Scalar upperGreen = new Scalar(80, 255, 255);

    // RED (often needs two ranges because red wraps around 0° in HSV)
    Scalar lowerRed1 = new Scalar(0, 100, 100);
    Scalar upperRed1 = new Scalar(10, 255, 255);

    Scalar lowerRed2 = new Scalar(160, 100, 100);
    Scalar upperRed2 = new Scalar(179, 255, 255);

    // BLUE
    Scalar lowerBlue = new Scalar(100, 150, 50);
    Scalar upperBlue = new Scalar(140, 255, 255);
    @Override
    public Mat processFrame(Mat img, long processTimeNanoSeconds) {
        Mat hsv = new Mat();
        Imgproc.cvtColor(img, hsv, Imgproc.COLOR_RGB2HSV);
        bluepixels = getColor(lowerBlue, upperBlue, hsv, "Blue", img, 50, 50);
        greenpixels = getColor(lowerGreen, upperGreen, hsv, "Green", img, 300, 50);
        yellowpixels = getColor(lowerYellow, upperYellow, hsv, "Yellow", img, 400, 50);
        Mat maskRed1 = new Mat();
        Mat maskRed2 = new Mat();
        Core.inRange(hsv, lowerRed1, upperRed1, maskRed1);
        Core.inRange(hsv, lowerRed2, upperRed2, maskRed2);
        Mat maskRed = new Mat();
        redpixels = Core.countNonZero(maskRed);
        return img;
    }
    public int getColor(Scalar lower, Scalar upper, Mat src, String color, Mat text,int x, int y) {
        Mat mask = new Mat();
        Core.inRange(src, lower, upper, mask);
        int Target = Core.countNonZero(mask);
        return Target;
    }
    public List<Integer> colors() {
        return Arrays.asList(redpixels, greenpixels, bluepixels, yellowpixels);
    }
    @Override
    public void init(int width, int height, CameraCalibration calibration) {

    }
    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }
}
