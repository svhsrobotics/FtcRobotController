package org.firstinspires.ftc.teamcode.robot.hardware;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;

public class Grabber {
    private DcMotor slide;
    private DcMotor pitch;
    private Servo wrist;
    private CRServo pinch;
    private static final int Slide_Max = 1000;
    public static final int Slide_Offset = 0;
    private static final int Pitch_Max = 1000;
    public static final int Pitch_Offset = 0;

    public Grabber(DcMotor slide, DcMotor pitch, Servo wrist, CRServo pinch) {
        this.slide = slide;
        this.pitch = pitch;
        this.wrist = wrist;
        this.pinch = pinch;
    }

    public enum Positions {
        START,
        ONECONE,
        TWOCONE,
        THREECONE,
        FOURCONE,
        COLLECT,
    }
    public void setGrabberPosition(Positions mode){
        switch(mode){
            case START:
                setPosition(3000,3000);
        }
    }

    public void reset() {
        this.slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.pitch.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void setPositions(int slide, double wrist) {
        this.wrist.setPosition(wrist);
        setPosition(this.slide, slide + Slide_Offset, Slide_Max);
    }
    private void setPosition(DcMotor slide, int position, double power, int max) {
        this.slide = slide;
        slide.setTargetPosition(position);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide.setPower(power);
        slide.getCurrentPosition();
    }
    public void setSlidePosition(int slide) {
        setPosition(this.slide, slide, Slide_Max);
    }
    public void setPivotPosition(int pivot) {
        setPosition(this.pitch, pivot, Pitch_Max);
    }
    public void setWristPosition(double wrist) {
        this.wrist.setPosition(wrist);
    }

    private void setPosition(DcMotor motor, int position, int max) {
        setPosition(motor, position, 1.0, max);
    }


    public enum CollectorModes{
        COLLECT,
        STOP,
        REMOVE,
    }


    public void setCollectorModes (CollectorModes mode){
        switch (mode) {
            case COLLECT:
                pinch.setPower(0.5);
                break;
            case STOP:
                pinch.setPower(0);
                break;
            case REMOVE:
                pinch.setPower(-0.25);
                break;
        }
    }
}