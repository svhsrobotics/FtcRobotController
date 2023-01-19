package org.firstinspires.ftc.teamcode.vision;

import static org.openftc.apriltag.AprilTagDetectorJNI.runApriltagDetector;
import static org.openftc.apriltag.ApriltagDetectionJNI.freeDetectionList;
import static org.openftc.apriltag.ApriltagDetectionJNI.getDetectionPointers;
import static org.openftc.apriltag.ApriltagDetectionJNI.getId;

import android.icu.text.BidiRun;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.vision.pole.GlobalMatPool;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.apriltag.AprilTagDetectorJNI;
import org.openftc.easyopencv.OpenCvPipeline;

@Config
public class AprilTagPipeline extends OpenCvPipeline
{
    private int[] ids;

    private long nativeApriltagPtr;

    public AprilTagPipeline()
    {
        // Allocate a native context object. See the corresponding deletion in the finalizer
        nativeApriltagPtr = AprilTagDetectorJNI.createApriltagDetector(AprilTagDetectorJNI.TagFamily.TAG_36h11.string, 3, 3);
    }

    @Override
    public void finalize()
    {
        // Might be null if createApriltagDetector() threw an exception
        if(nativeApriltagPtr != 0)
        {
            // Delete the native context we created in the constructor
            AprilTagDetectorJNI.releaseApriltagDetector(nativeApriltagPtr);
            nativeApriltagPtr = 0;
        }
        else
        {
            System.out.println("AprilTagDetectionPipeline.finalize(): nativeApriltagPtr was NULL");
        }
    }

    /// This function does all the actual work. It works with JNI pointers, be careful!
    private synchronized int[] detectTags(Mat input) {
        // Get a JNI pointer to the array containing the detections
        if (nativeApriltagPtr == 0) {
            throw new RuntimeException("AprilTagDetectionPipeline.detectTags(): nativeApriltagPtr was NULL");
        }
        if (input.nativeObj == 0) {
            throw new RuntimeException("AprilTagDetectionPipeline.detectTags(): input.nativeObj was NULL");
        }
        long detectionArrayPtr = runApriltagDetector(nativeApriltagPtr, input.nativeObj);
        // If no tags were found, return null
        if (detectionArrayPtr == 0) {
            //freeDetectionList(detectionArrayPtr);
            return null;
        }
        // Turn the JNI pointer to an array into an array of JNI pointers
        long[] detectionsPtr = getDetectionPointers(detectionArrayPtr);
        // Array to store the IDs of the tags we detect
        int[] ids = new int[detectionsPtr.length];
        // For each JNI pointer
        for (int i = 0; i < detectionsPtr.length; i++) {
            if (detectionsPtr[i] == 0) {
                throw new RuntimeException("AprilTagDetectionPipeline.detectTags(): detectionsPtr[" + i + "] was NULL");
            }
            ids[i] = getId(detectionsPtr[i]);
        }
        freeDetectionList(detectionArrayPtr);
        return ids;
    }

    public static double CONTRAST = 2.0;
    public static int BRIGHTNESS = 0;

    @Override
    public Mat processFrame(Mat input)
    {
        Mat grey = GlobalMatPool.get();

        // Convert to greyscale
        Imgproc.cvtColor(input, grey, Imgproc.COLOR_RGBA2GRAY);
        Core.convertScaleAbs(grey, grey, CONTRAST, BRIGHTNESS);

        this.ids = detectTags(grey);

        Imgproc.cvtColor(grey, input, Imgproc.COLOR_GRAY2RGBA);

        if (this.ids != null) {
            Imgproc.drawMarker(input, new Point(10,10), new Scalar(0,255,0));
        }

        GlobalMatPool.returnAll();
        return input;
    }

    public int[] getIds() {
        return this.ids;
    }
}
