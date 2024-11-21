package org.firstinspires.ftc.teamcode.opmode;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;



@Config
@TeleOp
public class nickpidloop extends LinearOpMode {
    private double reference = 250;
    private double error = 0;
    private double power = 0;
    private double maxpower = 0.5;
    private double kI = 0.1;
    private double kP = 0.1;
    private double kD = 0.1;

    private DcMotor arm = hardwareMap.get(DcMotor.class, "arm");

    public void runOpMode() throws InterruptedException {
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        while (opModeIsActive()) {
            while (gamepad1.a) {
                //for encoder position
                double curPosition = arm.getCurrentPosition();
               //the other error + current error
                double prevError = error;
                //other poop that i don't understand yes
                // todo learn the stuff under here
                error = reference - curPosition;
                double diff = error - prevError;
                arm.setPower((kP * error) + (kI * error) + (kD * diff));

            }
        }
    }
}
