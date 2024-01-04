package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.psi.PsiDrive;
import org.firstinspires.ftc.teamcode.drive.testbot.TestBotDrive;

public abstract class Robot {

    public abstract TrajectoryDrive getDrive();
    protected Robot(HardwareMap hardwareMap) {



    }



}
