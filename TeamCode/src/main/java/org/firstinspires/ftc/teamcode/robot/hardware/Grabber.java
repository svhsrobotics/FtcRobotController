package org.firstinspires.ftc.teamcode.robot.hardware;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;

class Lift {
    private DcMotor slide;
    private DcMotor pitch;

    public Lift(DcMotor slide, DcMotor pitch) {
        this.slide = slide;
        this.pitch = pitch;
        reset();
    }

    public void reset() {
        this.slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.pitch.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        this.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.pitch.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void setPositions(int slide, int pitch) {
        this.slide.setTargetPosition(slide);
        this.pitch.setTargetPosition(pitch);
    }

    public boolean done() {
        return (this.slide.getCurrentPosition() == this.slide.getTargetPosition()) &&
                (this.pitch.getCurrentPosition() == this.pitch.getTargetPosition());
    }
}

class Hand {
    private Servo wrist;
    private Servo pinch;

    public Hand(Servo wrist, Servo pinch){
        this.wrist = wrist;
        this.pinch = pinch;
    }
    public void setWristPosition(double wrist) {
        this.wrist.setPosition(wrist);
    }
    public void setPinchPosition(double pinch){
        this.pinch.setPosition(pinch);
    }
}

public class Grabber {
    private Hand hand;
    private Lift lift;

    public Grabber(DcMotor slide, DcMotor pitch,
                   Servo wrist, Servo pinch) {
        this.hand = new Hand(wrist, pinch);
        this.lift = new Lift(slide, pitch);
    }

    public Grabber(Hand hand, Lift lift) {
        this.hand = hand;
        this.lift = lift;
    }

    public enum Positions{
        START,
        SETONECONE,
        INTOONECONE,
        SETTWOCONE,
        INTOTWOCONE,
        SETTHREECONE,
        INTOTHREECONE,
        SETFOURCONE,
        INTOFOURCONE,
        COLLECT,
        GROUND,
        LOWPOLE,
        MEDIUMPOLE,
        HIGHPOLE,
        RELEASE,
    }
    public void setGrabberPosition(Positions mode){
        switch(mode){
            case START:
                lift.setPositions(100,100);
                hand.setPinchPosition(0);
                hand.setWristPosition(1000);
                break;

            case SETONECONE:
                lift.setPositions(100,100);
                hand.setWristPosition(1000);
                break;

            case INTOONECONE:
                lift.setPositions(100,100);
                hand.setWristPosition(1000);
                break;

            case SETTWOCONE:
                lift.setPositions(100,100);
                hand.setWristPosition(1000);
                break;

            case INTOTWOCONE:
                lift.setPositions(100,100);
                hand.setWristPosition(1000);
                break;

            case SETTHREECONE:
                lift.setPositions(100,100);
                hand.setWristPosition(1000);
                break;

            case INTOTHREECONE:
                lift.setPositions(100,100);
                hand.setWristPosition(1000);
                break;

            case SETFOURCONE:
                lift.setPositions(100,100);
                hand.setWristPosition(1000);
                break;

            case INTOFOURCONE:
                lift.setPositions(100,100);
                hand.setWristPosition(1000);
                break;

            case COLLECT:
                hand.setPinchPosition(10);
                break;

            case GROUND:
                lift.setPositions(100,100);
                hand.setWristPosition(1000);
                break;

            case LOWPOLE:
                lift.setPositions(100,100);
                hand.setWristPosition(1000);
                break;

            case MEDIUMPOLE:
                lift.setPositions(100,100);
                hand.setWristPosition(1000);
                break;

            case HIGHPOLE:
                lift.setPositions(100,100);
                hand.setWristPosition(1000);
                break;

            case RELEASE:
                hand.setPinchPosition(0);
                break;

        }
    }
}