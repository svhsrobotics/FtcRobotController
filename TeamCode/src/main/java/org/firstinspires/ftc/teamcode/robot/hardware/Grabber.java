package org.firstinspires.ftc.teamcode.robot.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class Grabber {
    public Hand hand;
    public Lift lift;

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
                hand.setWristPosition(1);
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
                hand.setWristPosition(0.25);
                break;

            case INTOFOURCONE:
                lift.setPositions(100,100);
                hand.setWristPosition(1000);
                break;

            case COLLECT:
                hand.setPinchPosition(1);
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