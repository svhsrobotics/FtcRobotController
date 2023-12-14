package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.psi.PsiDrive;
import org.firstinspires.ftc.teamcode.drive.testbot.TestBotDrive;

public class Robot {
    public enum BotType {
        ROBOTICA,
        PSI,
        TEST
    }

    public BotType botType;

    public MecanumDrive drive;

    public Robot(HardwareMap hardwareMap) {
        // Detect which bot this is:
        // If there is "pdw_intake" in the hardware map, then this is the robotica bot
        // If there is just "pdw" in the hardware map, then this is the psi bot
        // If there is "left_front_left_dw" in the hardware map, then this is the test bot
        if (hardwareMap.tryGet(DcMotorEx.class, "pdw_intake") != null) {
            // This is the robotica bot
            botType = BotType.ROBOTICA;
            //drive = new RoboticaDrive(hardwareMap);
        } else if (hardwareMap.tryGet(DcMotorEx.class, "pdw") != null) {
            // This is the psi bot
            botType = BotType.PSI;
            drive = new PsiDrive(hardwareMap);
        } else if (hardwareMap.tryGet(DcMotorEx.class, "left_front_left_dw") != null) {
            // This is the test bot
            botType = BotType.TEST;
            drive = new TestBotDrive(hardwareMap);
        }
    }
}
