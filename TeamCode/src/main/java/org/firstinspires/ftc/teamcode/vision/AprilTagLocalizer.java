package org.firstinspires.ftc.teamcode.vision;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.util.Timeout;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.Arrays;
import java.util.List;

@Config
public class AprilTagLocalizer {
    public static double APRILTAG_CUTOFF = 4 * 12;

    private final VisionPortal visionPortal;
    private final AprilTagProcessor aprilTag;

    /**
     * The array of webcams that will be used in the pipeline
     * You must pass them all so that we can create a SwitchableCamera
     */
    //private WebcamName[] webcams;
    public AprilTagLocalizer(AprilTagCamera[] cameras) {
        aprilTag = new AprilTagProcessor.Builder().build();

        WebcamName[] webcams = new WebcamName[cameras.length];
        for (int i = 0; i < cameras.length; i++) {
            webcams[i] = cameras[i].webcamName;
        }

        CameraName switchableCamera = ClassFactory.getInstance()
                .getCameraManager().nameForSwitchableCamera(webcams);

        // Create the vision portal by using a builder.
        this.visionPortal = new VisionPortal.Builder()
                .setCamera(switchableCamera)
                .addProcessor(aprilTag)
                .build();
    }

    /**
     * Switch to the camera at the given index in `webcams`
     * @param camera AprilTagCamera to switch to
     */
    public void switchCamera(AprilTagCamera camera) {
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            // Wait for the visionPortal status to be STREAMING before we attempt to switch cameras
            android.util.Log.i("AprilTag Test", "Waiting for camera to start STREAMING before switching cameras");
            Timeout timeout = new Timeout(5);
            while (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING && !timeout.expired()) {
                Timeout.sleep(10);
            }
        }
        if (visionPortal.getActiveCamera().getSerialNumber() != camera.webcamName.getSerialNumber()) {
            android.util.Log.i("AprilTag Test", "Camera is STREAMING, switching cameras from " + visionPortal.getActiveCamera().getSerialNumber() + " to " + camera.webcamName.getSerialNumber());
            visionPortal.setActiveCamera(camera.webcamName);
            // Flush any detections from the previous camera
            aprilTag.getFreshDetections();
        }
    }

    private Pose2d estimateCameraPoseFromAprilTags(AprilTagCamera camera) {
        android.util.Log.i("APRILTAG", "Switching cameras?");
        // Switch to the camera
        switchCamera(camera);
        android.util.Log.i("APRILTAG", "Switched camreas");


        android.util.Log.i("APRILTAG", "TEST");

        // Get the detections
        List<AprilTagDetection> detections = aprilTag.getFreshDetections();
        // Wait for detections to not be null
        // TODO: Timeout
        while (detections == null) {
            //android.util.Log.i("APRILTAG", "Waiting for non null return");
            detections = aprilTag.getFreshDetections();
        }
        // If there are no detections, return null
        // TODO: Multiple tries?
        if (detections.size() == 0) {
            android.util.Log.i("APRILTAG", "No detections (0 size)");
            return null;
        }
        // Get the closest AprilTag
        AprilTagDetection detection = null;
        for (AprilTagDetection currentDetection : detections) {
            assert currentDetection != null;
            assert currentDetection.ftcPose != null;

            if (detection == null || currentDetection.ftcPose.range < detection.ftcPose.range) {
                detection = currentDetection;
            }
        }

        // AprilTags further than the cutoff are unreliable, ignore
        if (detection.ftcPose.range > APRILTAG_CUTOFF) {
            android.util.Log.i("APRILTAG", "Range ("+detection.ftcPose.range+") is less than cutoff");
            return null;
        }

        android.util.Log.i("APRILTAG", "CAMERA ID: "+ detection.id);

        double thetaNeed = detection.ftcPose.yaw - detection.ftcPose.bearing;
        double a = detection.ftcPose.range * Math.cos(Math.toRadians(thetaNeed));
        double b = detection.ftcPose.range * Math.sin(Math.toRadians(thetaNeed));

        List<Integer> NEED_TO_ROTATE = Arrays.asList(1, 2, 3, 4, 5, 6);

        double absX, absY, absRot;
        if (NEED_TO_ROTATE.contains(detection.id)) {
            absX = detection.metadata.fieldPosition.get(0) - a; // + for lower side
            absY = detection.metadata.fieldPosition.get(1) + b; // - for lower side
            absRot = 180 - detection.ftcPose.yaw + 180;
        } else {
            absX = detection.metadata.fieldPosition.get(0) + a; // + for lower side
            absY = detection.metadata.fieldPosition.get(1) - b; // - for lower side
            absRot = 180 - detection.ftcPose.yaw;
        }

        absRot = absRot % 360;

        return new Pose2d(absX, absY, Math.toRadians(absRot));
    }
    public Pose2d estimateRobotPoseFromAprilTags(AprilTagCamera camera) {
        android.util.Log.i("APRILTAG", "Estimating robot pose for camera: " + camera.webcamName.getSerialNumber());
        Pose2d cameraPose = estimateCameraPoseFromAprilTags(camera);
        android.util.Log.i("APRILTAG", "Got camera pose");

        if (cameraPose != null) {
            android.util.Log.i("APRILTAG", "TRANSLATING POSE");
            return camera.translatePose(cameraPose);
        }
        android.util.Log.i("APRILTAG", "NULL POSE");
        return null;
    }

    public void close() {
        visionPortal.close();
    }

    public enum Quadrant {
        RED_AUDIENCE,
        RED_BOARD,
        BLUE_AUDIENCE,
        BLUE_BOARD
    }

    public Quadrant whichQuadrant(Pose2d pose) {
        if (pose.getX() < 0) {
            if (pose.getY() < 0) {
                return Quadrant.RED_AUDIENCE;
            } else {
                return Quadrant.BLUE_AUDIENCE;
            }
        } else {
            if (pose.getY() < 0) {
                return Quadrant.RED_BOARD;
            } else {
                return Quadrant.BLUE_BOARD;
            }
        }
    }
}
