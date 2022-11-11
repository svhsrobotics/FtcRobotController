package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.Shared.Drive2;
import org.firstinspires.ftc.teamcode.robot.CompetitionRobot;
import org.firstinspires.ftc.teamcode.robot.hardware.Arm.HubPosition;
import org.firstinspires.ftc.teamcode.util.ExMath;
import org.firstinspires.ftc.teamcode.robot.Robot;

@Disabled
@TeleOp(name = "Competition TeleOp", group = "Competition")
public class CompetitionTeleOp extends LinearOpMode {
    //private final String TAG = getClass().getName();
    CompetitionRobot robot;
    DcMotor Arm = null;
    Servo Wrist = null;
    CRServo Collector = null;
    double pivotCollectorFactor = 0.17 / 2;
    double pivotCollectorDifference = (0.17 / 2) + 0.36;
    DcMotor leftFrontDrive, rightFrontDrive, leftBackDrive, rightBackDrive = null;
    CRServo rightCarousel, leftCarousel=null;
    Servo cap=null;
    double carouselTrim = 0;
    int rightSign;
    int leftSign;
    int carouselPower=0;
    String currentColor;
    Drive2 drive = null;



    public int offset = org.firstinspires.ftc.teamcode.robot.hardware.Arm.ARM_OFFSET;
    @Override
    public void runOpMode() {
        //final Drive2 drive = new Drive2(this);

        //drive.init();

        Arm = hardwareMap.get(DcMotor.class, "Arm");
        Wrist = hardwareMap.get(Servo.class, "pivotCollector");
        Collector = hardwareMap.get(CRServo.class, "spinCollector");
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftFrontDrive = hardwareMap.get(DcMotor.class, "FL");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "FR");
        leftBackDrive = hardwareMap.get(DcMotor.class, "BL");
        rightBackDrive = hardwareMap.get(DcMotor.class, "BR");
        rightCarousel = hardwareMap.get(CRServo.class, "rightCarousel");
        leftCarousel = hardwareMap.get(CRServo.class, "leftCarousel");
        cap = hardwareMap.get(Servo.class, "cap");
        robot = new CompetitionRobot(hardwareMap);
        robot.initHardware();
        drive = new Drive2(robot, this);
        cap.setPosition(.5);

        RevBlinkinLedDriver lights = hardwareMap.get(RevBlinkinLedDriver.class, "LED");
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
        currentColor = "default";

        waitForStart();
        double startMillis = System.currentTimeMillis();

        long carouselRStart = 0;
        long carouselLStart = 0;
        long carouselTimout = 2000 * 1000 * 1000;

        double currentCap = .5;
        boolean lowering = false;
        boolean raising = false;

        while (opModeIsActive()) {

            // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)
            double frontRightPowerFactor, frontLeftPowerFactor, backRightPowerFactor, backLeftPowerFactor;
            double magRight = Math.hypot(gamepad1.right_stick_x, gamepad1.right_stick_y);
            double thetaRight = Math.atan2(-gamepad1.right_stick_y, gamepad1.right_stick_x);
            double magLeft = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
            double thetaLeft = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x);
            double pi = Math.PI;

            if (gamepad1.left_trigger > 0.5) {
                magRight = ExMath.square_with_sign(magRight) / 2;
                magLeft = ExMath.square_with_sign(magLeft) / 2;
            }

            if (thetaRight > 0 && thetaRight < pi / 2) {
                frontRightPowerFactor = -Math.cos(2 * thetaRight);
            } else if (thetaRight >= -pi && thetaRight < -pi / 2) {
                frontRightPowerFactor = Math.cos(2 * thetaRight);
            } else if (thetaRight >= pi / 2 && thetaRight <= pi) {
                frontRightPowerFactor = 1;
            } else {
                frontRightPowerFactor = -1;
            }

            if (thetaLeft > 0 && thetaLeft < pi / 2) {
                backLeftPowerFactor = -Math.cos(2 * thetaLeft);
            } else if (thetaLeft >= -pi && thetaLeft < -pi / 2) {
                backLeftPowerFactor = Math.cos(2 * thetaLeft);
            } else if (thetaLeft >= pi / 2 && thetaLeft <= pi) {
                backLeftPowerFactor = 1;
            } else {
                backLeftPowerFactor = -1;
            }

            if (thetaRight > -pi / 2 && thetaRight < 0) {
                backRightPowerFactor = Math.cos(2 * thetaRight);
            } else if (thetaRight > pi / 2 && thetaRight < pi) {
                backRightPowerFactor = -Math.cos(2 * thetaRight);
            } else if (thetaRight >= 0 && thetaRight <= pi / 2) {
                backRightPowerFactor = 1;
            } else {
                backRightPowerFactor = -1;
            }

            if (thetaLeft > -pi / 2 && thetaLeft < 0) {
                frontLeftPowerFactor = Math.cos(2 * thetaLeft);
            } else if (thetaLeft > pi / 2 && thetaLeft < pi) {
                frontLeftPowerFactor = -Math.cos(2 * thetaLeft);
            } else if (thetaLeft >= 0 && thetaLeft <= pi / 2) {
                frontLeftPowerFactor = 1;
            } else {
                frontLeftPowerFactor = -1;
            }
            if(frontLeftPowerFactor < 0){
            leftSign=-1;
            }else if(frontLeftPowerFactor > 0){
                leftSign=1;
            }else if(frontRightPowerFactor<0){
                rightSign=-1;
            }else if(frontRightPowerFactor>0){
                rightSign=1;
            }
           // leftFrontDrive.setPower((frontLeftPowerFactor * magLeft)*(frontLeftPowerFactor * magLeft));
           // rightFrontDrive.setPower(-(frontRightPowerFactor * magRight)*(frontRightPowerFactor * magRight));
            //leftBackDrive.setPower((backLeftPowerFactor * magLeft)*(backLeftPowerFactor * magLeft));
            //rightBackDrive.setPower(-(backRightPowerFactor * magRight)*(backRightPowerFactor * magRight));

            leftFrontDrive.setPower(((ExMath.square_with_sign(frontLeftPowerFactor) * magLeft)));
            rightFrontDrive.setPower((ExMath.square_with_sign(frontRightPowerFactor) * magRight));
            leftBackDrive.setPower(((ExMath.square_with_sign(backLeftPowerFactor) * magLeft)));
            rightBackDrive.setPower((ExMath.square_with_sign(backRightPowerFactor) * magRight));

            if (gamepad2.y) {
                if(gamepad2.dpad_down){
                    robot.arm.goToPosition(org.firstinspires.ftc.teamcode.robot.hardware.Arm.HubPosition.PARK);
                //Arm.setTargetPosition(0);
                //Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                //Arm.setPower(0.5);
                //Wrist.setPosition(.38);
            } else if (gamepad2.dpad_up){
                    robot.arm.goToPosition(org.firstinspires.ftc.teamcode.robot.hardware.Arm.HubPosition.COLLECT);
                    //Arm.setTargetPosition(-400+offset);//-300
                    //Arm.setTargetPosition(-432);//-300
                    //Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    //Arm.setPower(0.5);
                    //Wrist.setPosition(.68);
                }}
                else if (gamepad2.back) {
                //GoToHubLevel(1);
            } else if (gamepad2.x) {
                    if (gamepad2.dpad_down) {
                        robot.arm.goToBackPosition(HubPosition.TOP);
                    } else {
                        robot.arm.goToPosition(HubPosition.TOP);
                    }
                    //robot.arm.goToPosition(org.firstinspires.ftc.teamcode.robot.hardware.Arm.HubPosition.TOP);
                //GoToHubLevel(2);
            } else if (gamepad2.b) {
                    if (gamepad2.dpad_down) {
                        robot.arm.goToBackPosition(HubPosition.MIDDLE);
                    } else {
                        robot.arm.goToPosition(HubPosition.MIDDLE);
                    }
                    //robot.arm.goToPosition(org.firstinspires.ftc.teamcode.robot.hardware.Arm.HubPosition.MIDDLE);
                //GoToHubLevel(3);
            } else if (gamepad2.a) {
                    if (gamepad2.dpad_down) {
                        robot.arm.goToBackPosition(HubPosition.BOTTOM);
                    } else {
                        robot.arm.goToPosition(HubPosition.BOTTOM);
                    }
                    //robot.arm.goToPosition(org.firstinspires.ftc.teamcode.robot.hardware.Arm.HubPosition.BOTTOM);
                //GoToHubLevel(4);
          //  } else if (gamepad2.dpad_right) {
          //      Wrist.setPosition(-gamepad2.right_stick_y * pivotCollectorFactor + pivotCollectorDifference);
          //  } else if (gamepad2.dpad_up) {
          //  Arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
         //       Arm.setPower((gamepad2.right_trigger + (-gamepad2.left_trigger)) / 2);
            }else if (gamepad2.right_stick_y==1){
                    //manual driver control of arm
                Arm.setPower(1);
                Arm.setTargetPosition(Arm.getCurrentPosition()+100);
                Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }else if (gamepad2.right_stick_y==-1) {
                    Arm.setPower(1);
                Arm.setTargetPosition(Arm.getCurrentPosition()-100);
                Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }else if (gamepad2.right_stick_x==1){
                    //manual driver control of wrist
                    Wrist.setPosition(Wrist.getPosition()-.01);
                    sleep(100);
            }else if (gamepad2.right_stick_x==-1){
                Wrist.setPosition(Wrist.getPosition()+.01);
                sleep(100);
                //TODO: Make fail safe so that it doesn't go too far
            } else if (gamepad1.right_bumper){/*
                if (carouselRRunning) {
                    if ((System.nanoTime() - carouselRStart) > 100000) {
                        carouselRRunning = false;
                        rightCarousel.setPower(0);
                    }
                } else {
                    carouselRRunning = true;
                    rightCarousel.setPower(-80-carouselTrim);
                    carouselRStart = System.nanoTime();
                }*/
                carouselRStart = System.nanoTime();
                carouselLStart = System.nanoTime();
                carouselPower = 80;
                    //right carousel
                    /*rightCarousel.setPower(-80-carouselTrim);
                    sleep(2000);
                    rightCarousel.setPower(0);*/
            }
            else if (gamepad1.left_bumper){
                /*
                if (carouselLRunning) {
                    if ((System.nanoTime() - carouselLStart) > 100000) {
                        carouselLRunning = false;
                        leftCarousel.setPower(0);
                    }
                } else {
                    carouselLRunning = true;
                    leftCarousel.setPower(-80-carouselTrim);
                    carouselLStart = System.nanoTime();
                }*/
                carouselRStart = System.nanoTime();
                carouselLStart = System.nanoTime();
                carouselPower = -80;
                    //left carousel
                    /*leftCarousel.setPower(-80-carouselTrim);
                    sleep(2000);
                    leftCarousel.setPower(0);*/
            }else if (gamepad1.dpad_down){
                carouselTrim = carouselTrim -5;
                sleep(100);
            }else if (gamepad1.dpad_up){
                carouselTrim = carouselTrim +5;
                sleep(100);
            }else if (gamepad1.y){
                robot.arm.goToBackPosition(HubPosition.PARK);
            }

            // Cap arm
            if (gamepad1.dpad_up && currentCap <= 1.0) {
                lowering = raising = false;
                currentCap += 0.01;
            } else if (gamepad1.dpad_down && currentCap >= 0.0) {
                lowering = raising = false;
                currentCap -= 0.01;
            }

            double pickup = 0.05;
            double top = 0.31;
            double drop_off = 0.225;
            double half = 0.5;

            if (gamepad1.x) { // Pickup
                lowering = raising = false;
                currentCap = pickup;
            } else if (gamepad1.dpad_right) {
                lowering = raising = false;
                currentCap = half;
            } else if (gamepad1.dpad_left) { // Max
                lowering = false;
                raising = true;
                //currentCap = 0.31;
            } else if (gamepad1.a) { // Drop off + auto
                raising = false;
                lowering = true;
                //currentCap = 0.23;
            }


            if (lowering && currentCap > drop_off) {
                currentCap -= ((top - drop_off) / 300);
            } else if (lowering) {
                backoff_hub();
                lowering = false;
            }

            if (raising && currentCap < top) {
                currentCap += ((top - pickup) / 300);
            } else {
                raising = false;
            }

            cap.setPosition(currentCap);

            if (carouselRStart != 0) {
                if ((System.nanoTime() - carouselRStart) > carouselTimout) {
                    carouselRStart = 0;
                    rightCarousel.setPower(0);
                } else {
                    rightCarousel.setPower(carouselPower-carouselTrim);
                }
            }

            if (carouselLStart != 0) {
                if ((System.nanoTime() - carouselLStart) > carouselTimout) {
                    carouselLStart = 0;
                    leftCarousel.setPower(0);
                } else {
                    leftCarousel.setPower(carouselPower-carouselTrim);
                }
            }
            if (gamepad1.b){
                carouselRStart = -carouselTimout;
                carouselLStart = -carouselTimout;
            }

            if (gamepad2.left_stick_y < -0.1) {
                Collector.setPower(-0.2);
            } else if (gamepad2.left_stick_y > 0.1) {
                Collector.setPower(1);
            } else if (gamepad2.left_stick_button) {
                Collector.setPower(-1);
            } else {
                Collector.setPower(0);
            }
            if (System.currentTimeMillis() - startMillis >= 85000) {
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
                currentColor = "yellow";
            }

            telemetry.addData("Ticks", Arm.getCurrentPosition());
            telemetry.addData("WristPos", Wrist.getPosition());
            telemetry.addData("Power", Collector.getPower());
            telemetry.addData("ArmPower", gamepad2.right_trigger + (-gamepad2.left_trigger));
            telemetry.addData("front right power ", ((float) Math.round(rightFrontDrive.getPower() * 100)) / 100);
            telemetry.addData("front left power ", ((float) Math.round(leftFrontDrive.getPower() * 100)) / 100);
            telemetry.addData("back right power ", ((float) Math.round(rightBackDrive.getPower() * 100)) / 100);
            telemetry.addData("back left power ", ((float) Math.round(leftBackDrive.getPower() * 100)) / 100);
            telemetry.addData("left joystick x", ((float) Math.round(gamepad1.left_stick_x * 100)) / 100);
            telemetry.addData("left joystick y", ((float) Math.round(-gamepad1.left_stick_y * 100)) / 100);
            telemetry.addData("magnitude left", ((float) Math.round(magLeft * 100)) / 100);
            telemetry.addData("thetaLeft", ((float) Math.round(thetaLeft / pi * 100)) / 100);
            telemetry.addData("Trim", carouselTrim);
            telemetry.addData("Current Color", currentColor);
            telemetry.addData("Cap", currentCap);
            telemetry.update();


        }

        // Only do this in simulator; real robot needs time to stop.
        //drive.ceaseMotion();
    }

    private void backoff_hub() {
        this.drive = new Drive2(robot, this);
        double angle = drive.getImuAngle();
        drive.navigationMonitorTicksPhi(10, 0, 10, angle, 10);
        drive.ceaseMotion();
    }

    /*private void GoToHubLevel(int hubLevel) {

        if (hubLevel == 1) { // Cap
            Arm.setTargetPosition(-3720+offset);
            Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Arm.setPower(1);
            Wrist.setPosition(0.38);
        }

        if (hubLevel == 2) { // Top
            if (gamepad2.dpad_down) {
                Arm.setTargetPosition(-2019+offset); //TODO: Need backload later
                Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Wrist.setPosition(.71);
                Arm.setPower(1);
            } else if (gamepad2.dpad_up) {
                //Arm.setTargetPosition(-2180+offset);//-350
                Arm.setTargetPosition(-2019+offset);//-350
                Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Wrist.setPosition(.71);
                Arm.setPower(1);
            }
        }
        if (hubLevel == 3) { // Mid
            if (gamepad2.dpad_down) {
                Arm.setTargetPosition(-1270+offset);
                Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Wrist.setPosition(.67);
                Arm.setPower(1);
            } else if (gamepad2.dpad_up) {
                //Arm.setTargetPosition(-1157+offset);//-250
                Arm.setTargetPosition(-1270+offset);//-250
                Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Wrist.setPosition(.67);
                Arm.setPower(1);
            }
        }

        if (hubLevel == 4) {
            if (gamepad2.dpad_down) {
                Arm.setTargetPosition(-6353+offset);
                Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Wrist.setPosition(.61);
                Arm.setPower(1);
            } else if (gamepad2.dpad_up) {
                //Arm.setTargetPosition(-398+offset);//-430-250
                Arm.setTargetPosition(-369+offset);//-430-250
                Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Wrist.setPosition(.57);
                Arm.setPower(1);
            }

        }
    } */
}
