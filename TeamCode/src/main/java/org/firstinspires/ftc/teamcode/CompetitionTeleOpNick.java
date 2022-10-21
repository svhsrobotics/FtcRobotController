package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robot.CompetitionRobot;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.hardware.Arm;

@Disabled
@TeleOp(name = "Nick's Competition TeleOp", group = "Competition")
public class CompetitionTeleOpNick extends LinearOpMode {
    //private final String TAG = getClass().getName();
    DcMotor arm = null;
    Servo wrist = null;
    CRServo collector = null;
    double pivotCollectorFactor = 0.17 / 2;
    double pivotCollectorDifference = (0.17 / 2) + 0.36;
    DcMotor leftFrontDrive, rightFrontDrive, leftBackDrive, rightBackDrive = null;
    CRServo rightCarousel, leftCarousel=null;
    double carouselTrim = 0;
    int rightSign;
    int leftSign;
    double ydrive;
    double xdrive;

    public int offset = org.firstinspires.ftc.teamcode.robot.hardware.Arm.ARM_OFFSET;
    @Override
    public void runOpMode() {
        //final Drive2 drive = new Drive2(this);

        //drive.init();

        arm = hardwareMap.get(DcMotor.class, "Arm");
        wrist = hardwareMap.get(Servo.class, "pivotCollector");
        collector = hardwareMap.get(CRServo.class, "spinCollector");
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftFrontDrive = hardwareMap.get(DcMotor.class, "FL");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "FR");
        leftBackDrive = hardwareMap.get(DcMotor.class, "BL");
        rightBackDrive = hardwareMap.get(DcMotor.class, "BR");
        rightCarousel = hardwareMap.get(CRServo.class, "rightCarousel");
        leftCarousel = hardwareMap.get(CRServo.class, "leftCarousel");
        //Hardware map the imu
        CompetitionRobot robot = new CompetitionRobot(hardwareMap);
        robot.initArm();
        robot.initIMU();


        waitForStart();

        long carouselRStart = 0;
        long carouselLStart = 0;
        long carouselTimout = 2000 * 1000 * 1000;



        while (opModeIsActive()) {

            // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)

            //Bot Centric
            //leftBackDrive.setPower(-(gamepad1.right_trigger-(0.75*gamepad1.left_trigger)-gamepad1.left_stick_y - (0.75*gamepad1.left_stick_x))); //
            //leftFrontDrive.setPower(-((0.75*gamepad1.right_trigger)-(0.75*gamepad1.left_trigger)-gamepad1.left_stick_y + gamepad1.left_stick_x)); //
            //rightFrontDrive.setPower((0.75*-gamepad1.right_trigger)+(0.75*gamepad1.left_trigger)-gamepad1.left_stick_y - gamepad1.left_stick_x); //
            //rightBackDrive.setPower((0.75*-gamepad1.right_trigger)+(0.75*gamepad1.left_trigger)-gamepad1.left_stick_y + gamepad1.left_stick_x);

            //Rough Field Centric
            if (java.lang.Math.abs(robot.imu.getAngularOrientation().firstAngle) < 45){
                ydrive = -gamepad1.left_stick_y * Math.abs(gamepad1.left_stick_y);
                xdrive = -gamepad1.left_stick_x * Math.abs(gamepad1.left_stick_x);
            } else if (java.lang.Math.abs(robot.imu.getAngularOrientation().firstAngle) > 135){
                xdrive = gamepad1.left_stick_x * Math.abs(gamepad1.left_stick_x);
                ydrive = gamepad1.left_stick_y * Math.abs(gamepad1.left_stick_y);
            } else if (robot.imu.getAngularOrientation().firstAngle < 135 && robot.imu.getAngularOrientation().firstAngle > 45) {
                ydrive = -gamepad1.left_stick_x * Math.abs(gamepad1.left_stick_x);
                xdrive = gamepad1.left_stick_y * Math.abs(gamepad1.left_stick_y);
            } else {
                ydrive = gamepad1.left_stick_x * Math.abs(gamepad1.left_stick_x);
                xdrive = -gamepad1.left_stick_y * Math.abs(gamepad1.left_stick_y);
            }
            leftBackDrive.setPower(-(gamepad1.right_trigger - gamepad1.left_trigger - ydrive - xdrive));
            leftFrontDrive.setPower(-(gamepad1.right_trigger - gamepad1.left_trigger - ydrive + xdrive));
            rightFrontDrive.setPower((-gamepad1.right_trigger + gamepad1.left_trigger - ydrive - xdrive));
            rightBackDrive.setPower((-gamepad1.right_trigger + gamepad1.left_trigger - ydrive + xdrive));



            if (gamepad2.y) {
                if(gamepad2.dpad_down){
                    robot.arm.goToPosition(Arm.HubPosition.PARK);
                //Arm.setTargetPosition(0);
                //Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                //Arm.setPower(0.5);
                //Wrist.setPosition(.38);
            } else if (gamepad2.dpad_up){
                    robot.arm.goToPosition(Arm.HubPosition.COLLECT);
                    //Arm.setTargetPosition(-400+offset);//-300
                    //arm.setTargetPosition(-363+offset);//-300
                    //arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    //arm.setPower(0.5);
                    //wrist.setPosition(.68);
                }}
                else if (gamepad2.back) {
                //GoToHubLevel(1);
            } else if (gamepad2.x) {
                    robot.arm.goToPosition(Arm.HubPosition.TOP);
                //GoToHubLevel(2);
            } else if (gamepad2.b) {
                    robot.arm.goToPosition(Arm.HubPosition.MIDDLE);
                //GoToHubLevel(3);
            } else if (gamepad2.a) {
                    robot.arm.goToPosition(Arm.HubPosition.BOTTOM);
               // GoToHubLevel(4);
          //  } else if (gamepad2.dpad_right) {
          //      Wrist.setPosition(-gamepad2.right_stick_y * pivotCollectorFactor + pivotCollectorDifference);
          //  } else if (gamepad2.dpad_up) {
          //  Arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
         //       Arm.setPower((gamepad2.right_trigger + (-gamepad2.left_trigger)) / 2);
            }else if (gamepad2.right_stick_y==1){
                    //manual driver control of arm
                arm.setPower(1);
                arm.setTargetPosition(arm.getCurrentPosition()+100);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }else if (gamepad2.right_stick_y==-1) {
                    arm.setPower(1);
                arm.setTargetPosition(arm.getCurrentPosition()-100);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }else if (gamepad2.right_stick_x==1){
                    //manual driver control of wrist
                    wrist.setPosition(wrist.getPosition()-.01);
                    sleep(100);
            }else if (gamepad2.right_stick_x==-1){
                wrist.setPosition(wrist.getPosition()+.01);
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
                carouselLStart = System.nanoTime();
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
                arm.setTargetPosition(-3204+offset);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                wrist.setPosition(.59);
            }

            if (carouselRStart != 0) {
                if ((System.nanoTime() - carouselRStart) > carouselTimout) {
                    carouselRStart = 0;
                    rightCarousel.setPower(0);
                } else {
                    rightCarousel.setPower(-80-carouselTrim);
                }
            }

            if (carouselLStart != 0) {
                if ((System.nanoTime() - carouselLStart) > carouselTimout) {
                    carouselLStart = 0;
                    leftCarousel.setPower(0);
                } else {
                    leftCarousel.setPower(-80-carouselTrim);
                }
            }
            if (gamepad1.b){
                carouselRStart = -carouselTimout;
                carouselLStart = -carouselTimout;
            }

            if (gamepad2.left_stick_y < -0.1) {
                collector.setPower(-0.2);
            } else if (gamepad2.left_stick_y > 0.1) {
                collector.setPower(1);
            } else if (gamepad2.left_stick_button) {
                collector.setPower(-1);
            } else {
                collector.setPower(0);
            }

            telemetry.addData("Ticks", arm.getCurrentPosition());
            telemetry.addData("WristPos", wrist.getPosition());
            telemetry.addData("Power", collector.getPower());
            telemetry.addData("ArmPower", gamepad2.right_trigger + (-gamepad2.left_trigger));
            telemetry.addData("front right power ", ((float) Math.round(rightFrontDrive.getPower() * 100)) / 100);
            telemetry.addData("front left power ", ((float) Math.round(leftFrontDrive.getPower() * 100)) / 100);
            telemetry.addData("back right power ", ((float) Math.round(rightBackDrive.getPower() * 100)) / 100);
            telemetry.addData("back left power ", ((float) Math.round(leftBackDrive.getPower() * 100)) / 100);
            telemetry.addData("left joystick x", ((float) Math.round(gamepad1.left_stick_x * 100)) / 100);
            telemetry.addData("left joystick y", ((float) Math.round(-gamepad1.left_stick_y * 100)) / 100);
            //telemetry.addData("magnitude left", ((float) Math.round(magLeft * 100)) / 100);
            //telemetry.addData("thetaLeft", ((float) Math.round(thetaLeft / pi * 100)) / 100);
            telemetry.addData("Trim", carouselTrim);

            telemetry.update();


        }

        // Only do this in simulator; real robot needs time to stop.
        //drive.ceaseMotion();
    }


    /*private void GoToHubLevel(int hubLevel) {

        if (hubLevel == 1) { // Cap
            arm.setTargetPosition(-3720+offset);
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm.setPower(1);
            wrist.setPosition(0.38);
        }

        if (hubLevel == 2) { // Top
            if (gamepad2.dpad_down) {
                arm.setTargetPosition(-2030+offset); //TODO: Need backload later
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                wrist.setPosition(.71);
                arm.setPower(1);
            } else if (gamepad2.dpad_up) {
                //Arm.setTargetPosition(-2180+offset);//-350
                arm.setTargetPosition(-2030+offset);//-350
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                wrist.setPosition(.71);
                arm.setPower(1);
            }
        }
        if (hubLevel == 3) { // Mid
            if (gamepad2.dpad_down) {
                arm.setTargetPosition(-1270+offset);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                wrist.setPosition(.67);
                arm.setPower(1);
            } else if (gamepad2.dpad_up) {
                //Arm.setTargetPosition(-1157+offset);//-250
                arm.setTargetPosition(-1270+offset);//-250
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                wrist.setPosition(.67);
                arm.setPower(1);
            }
        }

        if (hubLevel == 4) {
            if (gamepad2.dpad_down) {
                arm.setTargetPosition(-6353+offset);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                wrist.setPosition(.61);
                arm.setPower(1);
            } else if (gamepad2.dpad_up) {
                //Arm.setTargetPosition(-398+offset);//-430-250
                arm.setTargetPosition(-369+offset);//-430-250
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                wrist.setPosition(.57);
                arm.setPower(1);
            }

        }
    } */
}
