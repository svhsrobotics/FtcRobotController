package org.firstinspires.ftc.teamcode.vision;

import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvPipeline;

public class SwitchablePipeline extends OpenCvPipeline {

    public OpenCvPipeline currentPipeline;

    public SwitchablePipeline(OpenCvPipeline initialPipeline) {
        this.currentPipeline = initialPipeline;
    }

    @Override
    public Mat processFrame(Mat input) {
        return this.currentPipeline.processFrame(input);
    }
}
