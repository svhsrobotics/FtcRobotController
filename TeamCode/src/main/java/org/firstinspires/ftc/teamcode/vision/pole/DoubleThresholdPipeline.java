package org.firstinspires.ftc.teamcode.vision.pole;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicReference;

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
    /*private Mat hueRange(Mat input, double min, double max) {
        input = convertColor(input, ColorFormat.HSV);
        input = extractChannel(input, 0);
        Mat output = GlobalMatPool.get();
        Core.inRange(input, new Scalar(min), new Scalar(max), output);
        return output;
    }*/

    private Mat hueRange(Mat input, double min, double max) {
        input = convertColor(input, ColorFormat.HSV);
        input = extractChannel(input, 0);
        Mat output = GlobalMatPool.get();
        // If the min is greater than the max, we need to split the range
        if (min > max) {
            Mat output1 = GlobalMatPool.get();
            Mat output2 = GlobalMatPool.get();
            Core.inRange(input, new Scalar(min), new Scalar(180.0), output1);
            Core.inRange(input, new Scalar(0.0), new Scalar(max), output2);
            Core.bitwise_or(output1, output2, output);
        } else {
            Core.inRange(input, new Scalar(min), new Scalar(max), output);
        }
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
            if (Imgproc.contourArea(contour) > 100 && Imgproc.contourArea(contour) < 100000) {
                Rect rect = Imgproc.boundingRect(contour);
                if (rect.width < maxWidth && rect.height > minHeight) {
                    filteredContours.add(contour);
                }
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

    private Point[] fitRotatedRect(MatOfPoint contour) {
        // Convert the contour into a MatOfPoint2f
        MatOfPoint2f contour2f = new MatOfPoint2f(contour.toArray());
        // Fit a rotated rectangle to the contour
        RotatedRect rect = Imgproc.minAreaRect(contour2f);
        // Temporary Mat to hold the points of the rotated rectangle
        Mat tempOutput = new Mat();
        // Save the points of the rotated rectangle into the temp Mat
        Imgproc.boxPoints(rect, tempOutput);
        // Convert the points in the temp Mat into an array of points
        Point[] points = new Point[tempOutput.rows()];
        for (int i = 0; i < tempOutput.rows(); i++) {
            points[i] = new Point(tempOutput.get(i, 0)[0], tempOutput.get(i, 1)[0]);
        }
        // Release the temporary Mat and MatOfPoint2f
        tempOutput.release();
        contour2f.release();

        return points;
    }

    private Point[] getTopCorners(Point[] points) {
        Point[] outPoints = Arrays.stream(points).sorted(Comparator.comparingDouble((Point p) -> p.y)).limit(2).toArray(Point[]::new);
        outPoints = Arrays.stream(outPoints).sorted(Comparator.comparingDouble((Point p) -> p.x)).toArray(Point[]::new);
        // Top Left, Top Right
        return outPoints;
    }

    private Point getTopMiddle(Point[] points) {
        Point[] topPoints = getTopCorners(points);
        return new Point((topPoints[0].x + topPoints[1].x) / 2, (topPoints[0].y + topPoints[1].y) / 2);
    }

    private double getPoleWidth(MatOfPoint contour) {
        Point[] points = fitRotatedRect(contour);
        Point[] topPoints = getTopCorners(points);
        return Math.abs(topPoints[0].x - topPoints[1].x);
    }



    /* TUNING VARIABLES */
    // The simulator will automatically set these variables to the values you set in the UI
    public double minHue = 16.4;
    public double maxHue = 36.6;

    public double minBlue = 167.4;
    public double maxBlue = 208.4;

    public boolean showContours = true;

    public double minHeight = 20;
    public double maxWidth = 255;

    public int targetX = 170;

    //public int blockSize = 20;

    public long lastNanoTime = 0;

    private AtomicInteger frameCount = new AtomicInteger(0);
    //private AtomicReference<Double> widthMovingAverage = new AtomicReference<Double>(0.0);

    @Override
    public Mat processFrame(Mat input) {
        input = input.submat(new Rect(0, 720 / 2, 960, 720 / 2));
        //new Logger(telemetry).debug("ENTERED DOUBLE THRESHOLD PROCESS FRAME");
        telemetry.addData("Loop time" , ((System.nanoTime() - lastNanoTime) / 1000 / 1000));
        lastNanoTime = System.nanoTime();
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

        // Draw the contours on the input image for debugging
        for (MatOfPoint contour : contours) {
            Point[] points = fitRotatedRect(contour);
            // Draw the rotated rectangle
            Imgproc.drawContours(input, Arrays.asList(new MatOfPoint(points)), -1, new Scalar(0, 255, 0), 2);

            Imgproc.drawMarker(input, getTopMiddle(points), new Scalar(0, 0, 255), Imgproc.MARKER_CROSS, 10, 2);
        }

        // TODO: Remove or abstract
        //Imgproc.line(input, new Point(AllAutomovement.RotationPID.TARGET, 0), new Point(AllAutomovement.RotationPID.TARGET, input.height()), new Scalar(0, 0, 255), 2);

        poleX.set(-1);
        poleWidth.set(-1.0);
        // Find the contour with the largest area
        Mat finalInput = input;
        contours.stream().max(Comparator.comparingDouble(this::getPoleWidth)).ifPresent(max -> {
            Point[] points = fitRotatedRect(max);

            //Rect rect = Imgproc.boundingRect(max);

            //Point bottomMiddle = new Point(rect.x + (rect.width / 2.0), rect.y + rect.height);
            //Imgproc.drawMarker(input, bottomMiddle, new Scalar(0, 255, 0), Imgproc.MARKER_CROSS, 20, 2);
            Point topMiddle = getTopMiddle(points);
            Imgproc.drawMarker(finalInput, topMiddle, new Scalar(0, 255, 255), Imgproc.MARKER_CROSS, 10, 2);

            poleX.set((int) Math.floor(topMiddle.x));

            /*poleWidth.getAndUpdate((old) -> {
                double frameCount = this.frameCount.incrementAndGet();
                double modifiedOld = old * ((frameCount - 1) / frameCount);
                double modifiedNew = getPoleWidth(max) / (frameCount);
                return modifiedOld + modifiedNew;
            });*/
            poleWidth.set(getPoleWidth(max));

            //new Logger(telemetry).info(String.format("Pole Width: %f", poleWidth.get()));
            Imgproc.putText(finalInput, String.format("%f", poleWidth.get()), new Point(0, 50), Imgproc.FONT_HERSHEY_PLAIN, 1.5, new Scalar(0, 255, 0), 2);
            Imgproc.putText(finalInput, String.format("%d", poleX.get()), new Point(0, 20), Imgproc.FONT_HERSHEY_PLAIN, 1.5, new Scalar(0, 255, 0), 2);


        });


        //Imgproc.putText(input, String.valueOf(poleX), new Point(0,0), 0);

        GlobalMatPool.returnAll(telemetry);

       // telemetry.addData("Correction", necessaryCorrection());
        // Update the telemetry once per frame
        telemetry.update();

        if (showContours) {
            return input;
        } else {
            return visualize;
        }
    }

    /* PUBLIC INTERFACE */
    private AtomicInteger poleX = new AtomicInteger(-1);
    private AtomicReference<Double> poleWidth = new AtomicReference<>(-1.0);
    public double getPoleWidth() {
        return poleWidth.get();
    }

    public int getPoleX() {
        return poleX.get();
    }

    /*public int necessaryCorrection() {
        int px = poleX.get();
        if (px == -1) { // We haven't ran yet, no correction is necessary
            return -1;
        } else {
            return px - targetX;
        }
    }*/
}
