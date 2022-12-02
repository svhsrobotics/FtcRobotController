package org.firstinspires.ftc.teamcode.vision;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;
import org.opencv.videoio.VideoWriter;
import org.openftc.easyopencv.OpenCvPipeline;

public class RecordingPipeline extends OpenCvPipeline {

    // XVID codec should work when wrapped in MP4?
    int CODEC = VideoWriter.fourcc('M', 'J', 'P', 'G');
    int FPS = 30;

    public VideoWriter videoWriter;

    @Override
    public void init(Mat firstFrame) {
        android.util.Log.i("RecordingPipeline", "Creating new VideoWriter");
        videoWriter = new VideoWriter("/sdcard/%02d.jpg", 0, FPS, firstFrame.size());
    }

    @Override
    public Mat processFrame(Mat input) {
        android.util.Log.d("RecordingPipeline", "Writing frame");
        Mat bgr = new Mat();
        Imgproc.cvtColor(input, bgr, Imgproc.COLOR_RGB2BGR);
        videoWriter.write(bgr);
        return input;
    }
}
