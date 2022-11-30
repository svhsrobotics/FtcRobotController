package org.firstinspires.ftc.teamcode.vision;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Arrays;

public class DoubleThresholdPipeline extends OpenCvPipeline {
    // This constructor is used by the simulator to pass a Telemetry object
    // It's not actually defined by the parent class, it's just convention
    private final Telemetry telemetry;
    public DoubleThresholdPipeline(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    enum ColorFormat {
        RGB,
        Lab,
        HSV,
        YCrCb
    }

    private Mat convertColor(Mat input, ColorFormat format) {
        Mat output = GlobalMatPool.get();
        switch (format) {
            case RGB:
                output = input;
                break;
            case Lab:
                Imgproc.cvtColor(input, output, Imgproc.COLOR_RGB2Lab);
                break;
            case HSV:
                Imgproc.cvtColor(input, output, Imgproc.COLOR_RGB2HSV);
                break;
            case YCrCb:
                Imgproc.cvtColor(input, output, Imgproc.COLOR_RGB2YCrCb);
                break;
        }
        return output;
    }

    private Mat extractChannel(Mat input, int channel) {
        Mat output = GlobalMatPool.get();
        Core.extractChannel(input, output, channel);
        return output;
    }

    // White pixels are in range, black pixels are out of range
    private Mat hueRange(Mat input, double min, double max) {
        input = convertColor(input, ColorFormat.HSV);
        input = extractChannel(input, 0);
        Mat output = GlobalMatPool.get();
        Core.inRange(input, new Scalar(min), new Scalar(max), output);
        return output;
    }

    // White is selected
    private Mat blueRange(Mat input, double min, double max) {
        input = convertColor(input, ColorFormat.Lab);
        input = extractChannel(input, 2);
        Mat output = GlobalMatPool.get();
        Core.inRange(input, new Scalar(min), new Scalar(max), output);
        return output;
    }

    // White pixels are in range, black pixels are out of range
    private Mat hueRangeLuma(Mat input, double min, double max, Mat lumaCorrection) {
        input = convertColor(input, ColorFormat.HSV);
        input = extractChannel(input, 0);

        Mat output = GlobalMatPool.get();
        input.copyTo(output);

        Mat[] lumaBlocks = split(lumaCorrection, 20, 20);
        Mat[] outputBlocks = split(output, 20, 20);
        for (int i = 0; i < outputBlocks.length; i++) {
            telemetry.log().add("Test:" + (lumaBlocks[i].get(0, 0)[0] - 100));
            Core.inRange(outputBlocks[i], new Scalar(min + (lumaBlocks[i].get(0,0)[0] / 2)), new Scalar(max + (lumaBlocks[i].get(0,0)[0] / 2)), outputBlocks[i]);
        }

        return output;
    }

    // White is selected
    private Mat blueRangeLuma(Mat input, double min, double max) {
        input = convertColor(input, ColorFormat.Lab);
        input = extractChannel(input, 2);
        Mat output = GlobalMatPool.get();
        Core.inRange(input, new Scalar(min), new Scalar(max), output);
        return output;
    }

    private Mat luma(Mat input) {
        Mat output = convertColor(input, ColorFormat.Lab);
        output = extractChannel(output, 0);
        return output;
    }

    private Mat and(Mat input1, Mat input2) {
        Mat output = GlobalMatPool.get();
        Core.bitwise_and(input1, input2, output);
        return output;
    }

    private Mat merge(Mat input1, Mat input2, Mat input3) {
        Mat output = GlobalMatPool.get();
        Core.merge(Arrays.asList(input1, input2, input3), output);
        return output;
    }

    // Dilates to reduce small imperfections around edges
    private Mat fuzz(Mat input) {
        Mat output = GlobalMatPool.get();
        Imgproc.dilate(input, output, new Mat(), new Point(-1, -1), 2);
        //Imgproc.erode(output, output, new Mat(), new Point(-1, -1), 2);
        return output;
    }

    private ArrayList<MatOfPoint> contours(Mat input) {
        ArrayList<MatOfPoint> contoursList = new ArrayList<>();
        Imgproc.findContours(input, contoursList, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
        return contoursList;
    }

    private ArrayList<MatOfPoint> filterContours(ArrayList<MatOfPoint> contours) {
        ArrayList<MatOfPoint> filteredContours = new ArrayList<>();
        for (MatOfPoint contour : contours) {
            if (Imgproc.contourArea(contour) > 100 && Imgproc.contourArea(contour) < 10000) {
                filteredContours.add(contour);
            }
        }
        return filteredContours;
    }

    // Split the image into blocks
    private Mat[] split(Mat input, int height, int width) {
        // Make sure the image is divisible by the block size
        if (input.height() % height != 0 || input.width() % width != 0) {
            throw new IllegalArgumentException("Image size must be divisible by block size");
        }

        int rows = input.height() / height;
        int cols = input.width() / width;

        telemetry.addData("cols", cols);
        telemetry.addData("rows", rows);

        // This should not actually allocate any new memory
        // Java will allocate the array with null Mat references
        // and the submat function will return special "sub-mats" which are views
        // into the original Mat
        // I think. It doesn't crash with an OOME, at least.
        Mat[] output = new Mat[rows * cols];

        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
                output[i * cols + j] = input.submat(i * height, (i + 1) * height, j * width, (j + 1) * width);
            }
        }
        return output;
    }

    private Mat neighborhoodMean(Mat input, int size) {
        Mat output = GlobalMatPool.get();
        input.copyTo(output);

        Mat[] blocks = split(output, size, size);

        for (Mat block : blocks) {
            // Calculate the mean of the block
            Scalar mean = Core.mean(block);
            // Set all pixels in the block to the mean
            block.setTo(mean);
        }

        return output;
    }

    private void contourSmooth(ArrayList<MatOfPoint> contours) {
        for (MatOfPoint contour : contours) {
            for (int i = 0; i < contour.cols(); i++) {
                double thisCol;
                for (int j = 0; j < contour.rows(); j++) {
                    double[] point = contour.get(i, j);
                    if (point.length >= 1) {
                        telemetry.log().add("Point: " + point[0]);
                    }

                }
            }
        }
    }



    /* TUNING VARIABLES */
    // The simulator will automatically set these variables to the values you set in the UI
    public double minHue = 18.4;
    public double maxHue = 32.6;

    public double minBlue = 171.4;
    public double maxBlue = 205.4;

    public boolean showContours = false;

    public double minHeight = 20;
    public double maxWidth = 30;

    //public int blockSize = 20;

    @Override
    public Mat processFrame(Mat input) {
        //Mat luma = luma(input);
        // This is the luma correction mat
        //Mat lumaCorrection = neighborhoodMean(luma, blockSize);

        //Mat correctHue = hueRangeLuma(input, minHue, maxHue, lumaCorrection);
        //Mat correctBlue = blueRangeLuma(input, minBlue, maxBlue);
        Mat correctHue = hueRange(input, minHue, maxHue);
        Mat correctBlue = blueRange(input, minBlue, maxBlue);
        Mat selected = and(correctBlue, correctHue);
        selected = fuzz(selected);

        // Creates a RGB image, using correctHue as Red, correctBlue as Green, and selected as Blue
        // The selected pixels will be white because they have Red and Green
        Mat visualize = merge(correctHue, correctBlue, selected);

        ArrayList<MatOfPoint> contours = contours(selected);
        contours = filterContours(contours);
        telemetry.addData("Contours", contours.size());

        Imgproc.drawContours(input, contours, -1, new Scalar(255, 0, 0), 2);
        //contourSmooth(contours);
        for (MatOfPoint contour : contours) {
            Rect rect = Imgproc.boundingRect(contour);
            if (rect.width < maxWidth && rect.height > minHeight) {
                Imgproc.rectangle(input, rect, new Scalar(0, 255, 0), 2);
                Point bottomMiddle = new Point(rect.x + (rect.width / 2.0), rect.y + rect.height);
                Imgproc.drawMarker(input, bottomMiddle, new Scalar(0, 0, 255), Imgproc.MARKER_CROSS, 20, 2);

            }

        }

        GlobalMatPool.returnAll(telemetry);

        // Update the telemetry once per frame
        telemetry.update();

        if (showContours) {
            return input;
        } else {
            return visualize;
        }
    }
}
