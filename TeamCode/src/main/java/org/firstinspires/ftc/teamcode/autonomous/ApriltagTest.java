package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.Arrays;
import java.util.List;

@Autonomous(name = "AprilTag Test")
public class ApriltagTest extends LinearOpMode {
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;

    /**
     * The array of webcams that will be used in the pipeline
     */
    private WebcamName[] webcams;


    /**
     * Transform the angle into the first quadrant
     */
    private double quadrantTransform(double angle) {
        if (angle < 0 || angle > 360) {
            throw new IllegalArgumentException("Angle must be between 0 and 360");
        }

        if (angle < 90) {
            return angle;
        } else if (angle < 180) {
            return 180 - angle;
        } else if (angle < 270) {
            return angle - 180;
        } else {
            return 360 - angle;
        }
    }

    /**
     * Switch to the camera at the given index in `webcams`
     * @param cameraIndex the index of the camera to switch to
     */
    private void switchCamera(int cameraIndex) {
        // Wait for the visionPortal status to be STREAMING before we attempt to switch cameras
        android.util.Log.i("AprilTag Test", "Waiting for camera to start STREAMING before switching cameras");
        while (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            sleep(10);
        }
        android.util.Log.i("AprilTag Test", "Camera is STREAMING, switching cameras...");
        // Get the current camera
        CameraName currentCamera = visionPortal.getActiveCamera();
        // Find the index of the current camera in the array
        int currentCameraIndex = -1;
        for (int i = 0; i < webcams.length; i++) {
            if (webcams[i] == currentCamera) {
                currentCameraIndex = i;
                break;
            }
        }
        // If the current camera is not the one we want to switch to, switch to it
        if (currentCameraIndex != cameraIndex) {
            visionPortal.setActiveCamera(webcams[cameraIndex]);
            // Flush any detections from the previous camera
            aprilTag.getFreshDetections();
        }
    }

    private Pose2d estimateCameraPoseFromAprilTags(int camera) {
        // Switch to the camera
        switchCamera(camera);
        // Get the detections
        List<AprilTagDetection> detections = aprilTag.getFreshDetections();
        // Wait for detections to not be null
        // TODO: Timeout
        while (detections == null) {
            detections = aprilTag.getFreshDetections();
        }
        // If there are no detections, return null
        // TODO: Multiple tries?
        if (detections.size() == 0) {
            return null;
        }
        // Get the first detection
        // TODO: Pick the best detection?
        AprilTagDetection detection = detections.get(0);

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
    @Override
    public void runOpMode() throws InterruptedException {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        aprilTag = new AprilTagProcessor.Builder().build();

        // Add the cameras to the array
        webcams = new WebcamName[3];
        webcams[0] = hardwareMap.get(WebcamName.class, "Webcam 1");
        webcams[1] = hardwareMap.get(WebcamName.class, "Webcam 2");
        webcams[2] = hardwareMap.get(WebcamName.class, "Webcam 3");

        CameraName switchableCamera = ClassFactory.getInstance()
                .getCameraManager().nameForSwitchableCamera(webcams);

        // Create the vision portal by using a builder.
        visionPortal = new VisionPortal.Builder()
                .setCamera(switchableCamera)
                .addProcessor(aprilTag)
                .build();

        android.util.Log.i("AprilTag Test", "Waiting for start...");
        switchCamera(1);
        waitForStart();

        while(opModeIsActive()) {
            Pose2d cameraPose = estimateCameraPoseFromAprilTags(1);

            if (cameraPose != null) {
                android.util.Log.i("AprilTag Test", "Camera pose: " + cameraPose);

                double CAMERA_OFFSET = 10; // Center camera
                double CAMERA_ANGLE = Math.toRadians(50);

                double offsetX = CAMERA_OFFSET * Math.cos(cameraPose.getHeading());
                double offsetY = CAMERA_OFFSET * Math.sin(cameraPose.getHeading());

                android.util.Log.i("AprilTag Test", "Camera offset: " + offsetX + ", " + offsetY);

                double robotX = cameraPose.getX() - offsetX;
                double robotY = cameraPose.getY() - offsetY;
                double robotHeading = cameraPose.getHeading() - CAMERA_ANGLE;

                Pose2d robotPose = new Pose2d(robotX, robotY, robotHeading);

                android.util.Log.i("AprilTag Test", "Robot pose: " + robotPose);

                drive.setPoseEstimate(robotPose);

                drive.update();
            }
        }

        visionPortal.close();
    }
}