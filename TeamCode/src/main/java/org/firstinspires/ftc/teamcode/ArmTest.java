package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.PowerPlayBotV2;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.hardware.Arm;
import org.firstinspires.ftc.teamcode.util.Debouncer;
import org.firstinspires.ftc.teamcode.util.Logger;
import org.firstinspires.ftc.teamcode.util.Toggle;

@TeleOp(name = "ArmTest", group = "TeleOp")
public class ArmTest extends LinearOpMode {
    Logger logger = new Logger(telemetry);

    Toggle manualOverride = new Toggle();
    Toggle liftHold = new Toggle();
    int liftHoldValue = -500;
    Debouncer aButton = new Debouncer();

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
            if (manualOverride.update(gamepad2.x)) {
                // Begin Lift
                telemetry.addData("Lift", robot.arm.lift.getCurrentPosition());
                if (liftHold.update(gamepad2.y)) {
                    if (liftHoldValue != -500) {
                        robot.arm.lift.setTargetPosition(liftHoldValue);
                    }
                } else {
                    liftHoldValue = robot.arm.lift.getCurrentPosition();
                    robot.arm.lift.setPower(gamepad2.right_trigger - gamepad2.left_trigger);
                }
                // End Lift

                // Begin Reacher
                telemetry.addData("Reach", robot.arm.reacher.getCurrentPosition());
                robot.arm.reacher.setPower(gamepad2.right_stick_x);
                // End Reacher
            } else {
                // Begin Lift
                telemetry.addData("Lift", robot.arm.lift.getCurrentPosition());
                if (gamepad1.right_bumper) {
                    robot.arm.lift.setPreset(Arm.Lift.Preset.DRIVING);
                } else if (aButton.update(gamepad1.a)) {
                    robot.arm.pincher.contract();
                    robot.arm.lift.setPreset(Arm.Lift.Preset.GRAB);
                    robot.arm.pincher.expand();
                } else if (gamepad1.x) {
                    robot.arm.lift.setPreset(Arm.Lift.Preset.LOW_POLE);
                } else if (gamepad1.y) {
                    robot.arm.lift.setPreset(Arm.Lift.Preset.MEDIUM_POLE);
                } else if (gamepad1.b) {
                    robot.arm.lift.setPreset(Arm.Lift.Preset.HIGH_POLE);
                } else if (gamepad1.left_bumper && gamepad1.a) {
                    robot.arm.pincher.contract();
                }
            }

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
