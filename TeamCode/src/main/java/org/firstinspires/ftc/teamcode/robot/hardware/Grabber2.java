package org.firstinspires.ftc.teamcode.robot.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

public class slides{
        public DcMotor bottommotor;
        public DcMotor middlemotor;
        public DcMotor topmotor;
        public DcMotor hslide;

        public slides(DcMotor bottommotor, DcMotor middlemotor, DcMotor topmotor, DcMotor hslide) {
            this.bottommotor = bottommotor;
            this.middlemotor = middlemotor;
            this.topmotor = topmotor;
            this.hslide = hslide;
            reset();
        }


        public void reset() {
            this.bottommotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            this.middlemotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            this.topmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            this.hslide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            this.bottommotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            this.middlemotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            this.topmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            this.hslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            middlemotor.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        public void setSlidePositions(int y, int x) {
            this.bottommotor.setTargetPosition(y);
            this.middlemotor.setTargetPosition(y);
            this.topmotor.setTargetPosition(y);
            this.hslide.setTargetPosition(x);
        }

        public boolean done() {
            return (this.topmotor.getCurrentPosition() == this.topmotor.getTargetPosition()) &&
                    (this.middlemotor.getCurrentPosition() == this.middlemotor.getTargetPosition()) &&
                    (this.hslide.getCurrentPosition() == this.hslide.getTargetPosition()) &&
                    (this.bottommotor.getCurrentPosition() == this.bottommotor.getTargetPosition());
        }
    }

class pincher {
    public Servo pinch;

    public pincher(Servo pinch){
        this.pinch = pinch;
    }
    public void setPincher(double pinch){
        this.pinch.setPosition(pinch);
    }
}


    public class Grabber2 {
        public pincher pincher ;
        public slides slides;

        public Grabber2(DcMotor topmotor, DcMotor middlemotor, DcMotor bottommotor, DcMotor hslide, Servo pinch) {
            this.pincher = new pincher(pinch);
            this.slides = new slides(topmotor, middlemotor, bottommotor, hslide);
        }

        public enum Positions{
            START,
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
                    slides.setSlidePositions(0,0);
                    pincher.setPincher(0);
                    break;

                case SETFOURCONE:
                    slides.setSlidePositions(100,100);
                    pincher.setPincher(1000);
                    break;

                case INTOFOURCONE:
                    slides.setSlidePositions(100,100);
                    pincher.setPincher(1000);
                    break;

                case GROUND:
                    slides.setSlidePositions(100,100);
                    pincher.setPincher(1000);
                    break;

                case LOWPOLE:
                    slides.setSlidePositions(100,100);
                    pincher.setPincher(1000);
                    break;

                case MEDIUMPOLE:
                    slides.setSlidePositions(100,100);
                    pincher.setPincher(1000);
                    break;

                case HIGHPOLE:
                    slides.setSlidePositions(101,100);
                    pincher.setPincher(1000);
                    break;

                case COLLECT:
                    pincher.setPincher(0.45);
                    break;

                case RELEASE:
                    pincher.setPincher(0);
                    break;

            }
        }
    }
