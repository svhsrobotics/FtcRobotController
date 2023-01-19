package org.firstinspires.ftc.teamcode.robot.hardware;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.util.HardwareNotFoundException;
import org.firstinspires.ftc.teamcode.util.Logger;
import org.firstinspires.ftc.teamcode.util.Timeout;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

public class Webcam {
    // Constants
    private static final OpenCvCameraRotation ROTATION = OpenCvCameraRotation.UPRIGHT;
    private static final int WIDTH = 960;
    private static final int HEIGHT = 720;

    public boolean opened = false;

    private final OpenCvWebcam webcam;

    /**
     * Convenience class for interacting with OpenCV webcams.
     * @param webcam OpenCV webcam to wrap
     */
    public Webcam(@NonNull OpenCvWebcam webcam) {
        this.webcam = webcam;
    }

    /**
     * Convenience class for interacting with OpenCV webcams.
     * @param name name of the device in the hardware map
     * @param hardwareMap hardware map to get the camera from
     */
    public Webcam(String name, HardwareMap hardwareMap) {
        try {
            this.webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, name));
        } catch (IllegalArgumentException e) {
            throw new HardwareNotFoundException(e, "Webcam"); // Use Webcam instead of WebcamName
        }
        FtcDashboard.getInstance().startCameraStream(this.webcam, 30);
    }

    /**
     * Opens the camera device. Will automatically unpause the stream and pipeline.
     * This will prevent Vuforia and Tensorflow from accessing the camera.
     */
    public void openAsync() {
        this.webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                opened = true;
                resume();
            }

            @Override
            public void onError(int errorCode) {
                new Logger().error("Webcam returned error code: " + errorCode);
            }
        });
    }

    public void open() {
        open(new Timeout(10));
    }

    public void open(Timeout timeout) {
        openAsync();
        while (!this.opened && !timeout.expired()) { Thread.yield(); }
    }

    /**
     * Resumes the camera stream. Will continue with the previous pipeline.
     */
    public void resume() {
        this.webcam.startStreaming(WIDTH, HEIGHT, ROTATION);
    }

    /**
     * Pauses the camera stream and pipeline. Note that this will not free the
     * hardware, so Vuforia and TensorFlow will not work.
     */
    public void pause() {
        this.webcam.stopStreaming();
    }

    /**
     * Closes the camera device. This deactivates the hardware so that it can be used
     * by Vuforia or TensorFlow.
     */
    public void close() {
        pause();
        this.webcam.closeCameraDeviceAsync(() -> { });
    }

    /**
     * Changes the image-processing pipeline being used. Note that this can either
     * be called before opening, or on-the-fly while the camera is streaming.
     * @param pipeline
     */
    public void setPipeline(OpenCvPipeline pipeline) {
        this.webcam.setPipeline(pipeline);
    }

}
