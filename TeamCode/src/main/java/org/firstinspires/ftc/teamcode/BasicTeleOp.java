package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.PowerPlayBotV2;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.hardware.Arm;
import org.firstinspires.ftc.teamcode.util.Debouncer;
import org.firstinspires.ftc.teamcode.util.Logger;
import org.firstinspires.ftc.teamcode.util.Toggle;

@TeleOp(name = "TeleOp")
public class BasicTeleOp extends LinearOpMode {
    Logger logger = new Logger(telemetry);

    Toggle manualOverride = new Toggle();
    Toggle liftHold = new Toggle();
    int liftHoldValue = -500;
    Debouncer aButton = new Debouncer();
    Debouncer bButton = new Debouncer();

    @Override
    public void runOpMode() {
        PowerPlayBotV2 robot = new PowerPlayBotV2(hardwareMap, logger);
        robot.initHardware();

        robot.arm.lift.setPreset(Arm.Lift.Preset.DRIVING);
        robot.arm.reacher.setTargetPosition(0);

        waitForStart();

        while (!isStopRequested()) {
            // BEGIN DRIVE
            if (gamepad1.dpad_up || gamepad1.dpad_down || gamepad1.dpad_left || gamepad1.dpad_right) {
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
                robot.drives.get(Robot.DrivePos.FRONT_LEFT).setPower((gamepad1.right_trigger - gamepad1.left_trigger + (Math.signum(gamepad1.left_stick_x) * Math.pow(Math.abs(gamepad1.left_stick_x), 1.5)) + gamepad1.right_stick_x));
                robot.drives.get(Robot.DrivePos.FRONT_RIGHT).setPower((gamepad1.right_trigger - gamepad1.left_trigger - (Math.signum(gamepad1.left_stick_x) * Math.pow(Math.abs(gamepad1.left_stick_x), 1.5)) - gamepad1.right_stick_x));
                robot.drives.get(Robot.DrivePos.BACK_LEFT).setPower((gamepad1.right_trigger - gamepad1.left_trigger + (Math.signum(gamepad1.left_stick_x) * Math.pow(Math.abs(gamepad1.left_stick_x), 1.5)) - gamepad1.right_stick_x));
                robot.drives.get(Robot.DrivePos.BACK_RIGHT).setPower((gamepad1.right_trigger - gamepad1.left_trigger - (Math.signum(gamepad1.left_stick_x) * Math.pow(Math.abs(gamepad1.left_stick_x), 1.5)) + gamepad1.right_stick_x));
            }
            // END DRIVE

            // BEGIN ARM


            // Begin Reacher
            telemetry.addData("Reach", robot.arm.reacher.getCurrentPosition());

            if (gamepad2.x) {
                robot.arm.reacher.setTargetPosition(0);
            } else if (gamepad2.y) {
                robot.arm.reacher.setTargetPosition(2058);
            } else if (gamepad2.dpad_left) {
                robot.arm.reacher.setPower(-.1);
            } else if (gamepad2.dpad_right) {
                robot.arm.reacher.setPower(.1);
            } else {
                robot.arm.reacher.setPower(0);
            }
            // End Reacher

            // Begin Lift
            telemetry.addData("Lift", robot.arm.lift.getCurrentPosition());

            if (bButton.update(gamepad2.b)) {
                robot.arm.lift.setTargetPosition(robot.arm.lift.getTargetPosition() + 100);
            }

            if (!(gamepad2.right_trigger > 0.5)) {
                if (gamepad2.a) {
                    robot.arm.lift.setPower(0);
                } else if (gamepad1.right_bumper) {
                    robot.arm.lift.setPreset(Arm.Lift.Preset.DRIVING);
                } else if (aButton.update(gamepad1.a)) {
                    robot.arm.pincher.contract();
                    robot.arm.lift.setPreset(Arm.Lift.Preset.GRAB);
                    while (robot.arm.lift.isBusy()) {
                        telemetry.addData("Lift", robot.arm.lift.getCurrentPosition());
                        telemetry.update();
                    }
                    robot.arm.pincher.expand();
                    sleep(100);
                    robot.arm.lift.setPreset(Arm.Lift.Preset.DRIVING);
                } else if (gamepad1.x) {
                    robot.arm.lift.setPreset(Arm.Lift.Preset.LOW_POLE);
                } else if (gamepad1.y) {
                    robot.arm.lift.setPreset(Arm.Lift.Preset.MEDIUM_POLE);
                } else if (gamepad1.b) {
                    robot.arm.lift.setPreset(Arm.Lift.Preset.HIGH_POLE);
                }
            }

            // Begin Pincher
            if (gamepad2.left_bumper || gamepad1.left_bumper) {
                robot.arm.pincher.contract();
            } else if (gamepad2.right_bumper) {
                robot.arm.pincher.expand();
            }
            // End Pincher
            // END ARM

            telemetry.update(); // Update telemetry

        }

    }
}
