package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.PowerPlayBotV2;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.util.Logger;

@TeleOp(name = "TeleOp", group = "TeleOp")
public class ArmTest extends LinearOpMode {
    Logger logger = new Logger(telemetry);

    @Override
    public void runOpMode() {
        PowerPlayBotV2 robot = new PowerPlayBotV2(hardwareMap, logger);
        robot.initHardware();

        robot.arm.pincher.expand();

        waitForStart();

        while (!isStopRequested()) {
            // BEGIN DRIVE
            if (gamepad1.dpad_up || gamepad1.dpad_down || gamepad1.dpad_left || gamepad1.dpad_right){
                if (gamepad1.dpad_up) {
                    robot.drives.get(Robot.DrivePos.FRONT_LEFT).setPower(.1);
                    robot.drives.get(Robot.DrivePos.FRONT_RIGHT).setPower(.1);
                    robot.drives.get(Robot.DrivePos.BACK_LEFT).setPower(.1);
                    robot.drives.get(Robot.DrivePos.BACK_RIGHT).setPower(.1);
                }
                if (gamepad1.dpad_down) {
                    robot.drives.get(Robot.DrivePos.FRONT_LEFT).setPower(-.1);
                    robot.drives.get(Robot.DrivePos.FRONT_RIGHT).setPower(-.1);
                    robot.drives.get(Robot.DrivePos.BACK_LEFT).setPower(-.1);
                    robot.drives.get(Robot.DrivePos.BACK_RIGHT).setPower(-.1);
                }
                if (gamepad1.dpad_left) {
                    robot.drives.get(Robot.DrivePos.FRONT_LEFT).setPower(-.1);
                    robot.drives.get(Robot.DrivePos.FRONT_RIGHT).setPower(.1);
                    robot.drives.get(Robot.DrivePos.BACK_LEFT).setPower(.1);
                    robot.drives.get(Robot.DrivePos.BACK_RIGHT).setPower(-.1);
                }
                if (gamepad1.dpad_right) {
                    robot.drives.get(Robot.DrivePos.FRONT_LEFT).setPower(.1);
                    robot.drives.get(Robot.DrivePos.FRONT_RIGHT).setPower(-.1);
                    robot.drives.get(Robot.DrivePos.BACK_LEFT).setPower(-.1);
                    robot.drives.get(Robot.DrivePos.BACK_RIGHT).setPower(.1);
                }
            } else {
                robot.drives.get(Robot.DrivePos.FRONT_LEFT).setPower(
                        (gamepad1.right_trigger - gamepad1.left_trigger + Math.pow(gamepad1.left_stick_x, 1.5) + gamepad1.right_stick_x));
                robot.drives.get(Robot.DrivePos.FRONT_RIGHT).setPower(
                        (gamepad1.right_trigger - gamepad1.left_trigger - Math.pow(gamepad1.left_stick_x, 1.5) - gamepad1.right_stick_x));
                robot.drives.get(Robot.DrivePos.BACK_LEFT).setPower(
                        (gamepad1.right_trigger - gamepad1.left_trigger + Math.pow(gamepad1.left_stick_x, 1.5) - gamepad1.right_stick_x));
                robot.drives.get(Robot.DrivePos.BACK_RIGHT).setPower(
                        (gamepad1.right_trigger - gamepad1.left_trigger - Math.pow(gamepad1.left_stick_x, 1.5) + gamepad1.right_stick_x));
                if (gamepad2.a) {
                    robot.drives.get(Robot.DrivePos.FRONT_LEFT).setPower(
                            (gamepad2.right_trigger - gamepad2.left_trigger + Math.pow(gamepad2.left_stick_x, 1.5) + gamepad2.right_stick_x));
                    robot.drives.get(Robot.DrivePos.FRONT_RIGHT).setPower(
                            (gamepad2.right_trigger - gamepad2.left_trigger - Math.pow(gamepad2.left_stick_x, 1.5) - gamepad2.right_stick_x));
                    robot.drives.get(Robot.DrivePos.BACK_LEFT).setPower(
                            (gamepad2.right_trigger - gamepad2.left_trigger + Math.pow(gamepad2.left_stick_x, 1.5) - gamepad2.right_stick_x));
                    robot.drives.get(Robot.DrivePos.BACK_RIGHT).setPower(
                            (gamepad2.right_trigger - gamepad2.left_trigger - Math.pow(gamepad2.left_stick_x, 1.5) + gamepad2.right_stick_x));
                }
            }
            // END DRIVE

            // BEGIN ARM
            // Begin Lift
            telemetry.addData("Lift", robot.arm.lift.getCurrentPosition());
            if (gamepad2.right_trigger != 0) {
                robot.arm.lift.setPower(gamepad2.right_trigger);
            } else if (gamepad2.left_trigger != 0) {
                robot.arm.lift.setPower(-gamepad2.left_trigger);
            }
            // End Lift

            // Begin Reacher
            telemetry.addData("Reach", robot.arm.reacher.getCurrentPosition());
            if (gamepad2.right_stick_x != 0) {
                robot.arm.reacher.setPower(gamepad2.right_stick_x);
            }
            // End Reacher

            // Begin Pincher
            if (gamepad2.left_bumper) {
                robot.arm.pincher.expand();
            } else if (gamepad2.right_bumper) {
                robot.arm.pincher.contract();
            }
            // End Pincher


            // END ARM

            telemetry.update(); // Update telemetry

        }

    }
}
