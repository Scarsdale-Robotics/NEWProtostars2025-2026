package org.firstinspires.ftc.teamcode.TestPipelines;

import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvPipeline;

public class TestPipeline1 extends OpenCvPipeline {
    @Override
    public Mat processFrame(Mat img) {
        return img;
    }
}
