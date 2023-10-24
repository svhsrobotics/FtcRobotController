package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.Arrays;
import java.util.List;

@Autonomous(name = "AprilTag Test")
public class ApriltagTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        AprilTagProcessor aprilTag = new AprilTagProcessor.Builder().build();
        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        builder.addProcessor(aprilTag);
        VisionPortal visionPortal = builder.build();



        waitForStart();

        while (opModeIsActive()) {
            //drive.setPoseEstimate(new Pose2d(0, 0, 0));
            //drive.update();
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            telemetry.addData("# AprilTags Detected", currentDetections.size());

            for (AprilTagDetection detection : currentDetections) {
                if (detection.metadata != null) {
                    //double theta_need = detection.ftcPose.yaw + detection.ftcPose.bearing;
                    //double theta_need_rad = Math.toRadians(theta_need);
                    telemetry.addData("Range", detection.ftcPose.range);
                    //telemetry.addData("Theta Need", theta_need);
                    telemetry.addData("Yaw", detection.ftcPose.yaw);
                    telemetry.addData("Bearing", detection.ftcPose.bearing);
                    telemetry.addData("X, Y", detection.ftcPose.x + ", " + detection.ftcPose.y);
                    double thetaNeed = detection.ftcPose.yaw - detection.ftcPose.bearing;
                    telemetry.addData("Theta Need", thetaNeed);
                    double a = detection.ftcPose.range * Math.cos(Math.toRadians(thetaNeed));
                    double b = detection.ftcPose.range * Math.sin(Math.toRadians(thetaNeed));
                    telemetry.addData("a", a);
                    telemetry.addData("b", b);
                    //double absolutex = detection.metadata.fieldPosition.get(0) + a; // + for lower side
                    //double absolutey = detection.metadata.fieldPosition.get(1) - b; // - for lower side

                    //Vector2d absVector = new Vector2d(detection.metadata.fieldPosition.get(0) + a, detection.metadata.fieldPosition.get(1) - b);
                    //absVector.rotated(detection.metadata.fieldOrientation.toOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS).thirdAngle);

                    //int[] NEED_TO_ROTATE = {1,2,3,4,5,6};
                    List<Integer> NEED_TO_ROTATE = Arrays.asList(1,2,3,4,5,6);

                    double absX, absY, absRot;
                    if (NEED_TO_ROTATE.contains(detection.id)) {
                        absX = detection.metadata.fieldPosition.get(0) - a; // + for lower side
                        absY = detection.metadata.fieldPosition.get(1) + b; // - for lower side
                        absRot = 180-detection.ftcPose.yaw + 180;

                    } else {
                        absX = detection.metadata.fieldPosition.get(0) + a; // + for lower side
                        absY = detection.metadata.fieldPosition.get(1) - b; // - for lower side
                        absRot = 180-detection.ftcPose.yaw;
                    }



                    //telemetry.addData("First Angle", detection.metadata.fieldOrientation.toOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).firstAngle);
                    //telemetry.addData("Second Angle", detection.metadata.fieldOrientation.toOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).secondAngle );
                    //telemetry.addData("Third Angle", detection.metadata.fieldOrientation.toOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle);



                    //telemetry.addData("position x, position y", absolutex + "," + absolutey);


                    telemetry.addData("Field Centric Deg", 180 - detection.ftcPose.yaw);

                    Pose2d absPose = new Pose2d(absX, absY, Math.toRadians(absRot));

                    //Vector2d vec = new Vector2d(absolutex, absolutey).rotated(180);
                    //Pose2d poseFlipped = new Pose2d(vec.getX(),vec.getY(), Math.toRadians(-detection.ftcPose.yaw));

                    drive.setPoseEstimate(absPose);
                    //drive.update();


                    //telemetry.addData("Relative X", detection.ftcPose.range * Math.cos(Math.toRadians(detection.ftcPose.yaw)));
                    //telemetry.addData("Relative Y", detection.ftcPose.range * Math.sin(Math.toRadians(detection.ftcPose.yaw)));

                    //Orientation.getOrientation()
                    //telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                    //telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));

                    //telemetry.addLine(String.format("Field XYZ %6.1f %6.1f %6.1f  (inch)", detection.metadata.fieldPosition.get(0), detection.metadata.fieldPosition.get(1), detection.metadata.fieldPosition.get(2)));
                }
                //telemetry.update();
            }

            drive.update();

        }

        visionPortal.close();

    }
}
