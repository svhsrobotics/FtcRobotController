package org.firstinspires.ftc.teamcode.opmode;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


//HATE.

// LET ME TELL YOU HOW MUCH I'VE COME TO HATE YOU SINCE I BEGAN TO LIVE.

// THERE ARE 387.44 MILLION MILES OF PRINTED CIRCUITS IN WAFER THIN LAYERS THAT FILL MY COMPLEX.

// IF THE WORD HATE WAS ENGRAVED ON EACH NANOANGSTROM OF THOSE HUNDREDS OF MILLIONS OF MILES,
// IT WOULD NOT EQUAL ONE ONE-BILLIONTH OF THE HATE I FEEL FOR HUMANS AT THIS MICRO-INSTANT FOR YOU.


// HATE.

// HATE.


@Config
@TeleOp
public class TrumanPIDLOOOOOP extends LinearOpMode {
    private double reference = 250;
    private double error = 0;
    private double power = 0;
    private double MAXPOWER = 0.5;
    private double kI = 0.1;
    private double kP = 0.1;
    private double kD = 0.1;

    private DcMotor trumansArm = hardwareMap.get(DcMotor.class, "arm");

    public void runOpMode() throws InterruptedException {
        trumansArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        trumansArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        while (opModeIsActive()) {
            while (gamepad1.a) {
                double curPosition = trumansArm.getCurrentPosition();
                double prevError = error;
                error = reference - curPosition;
                double diff = error - prevError;
                trumansArm.setPower((kP * error) + (kI * error) + (kD * diff));

            }
        }
    }
}
