package org.firstinspires.ftc.teamcode.robot.hardware;

import com.qualcomm.robotcore.hardware.Servo;

public class Hand {
    private Servo wrist;
    private Servo pinch;

    public Hand(Servo wrist, Servo pinch) {
        this.wrist = wrist;
        this.pinch = pinch;
    }

    public void setWristPosition(double wrist) {
        this.wrist.setPosition(wrist);
    }

    public void setPinchPosition(double pinch) {
        this.pinch.setPosition(pinch);
    }
}
