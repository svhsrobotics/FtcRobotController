package org.firstinspires.ftc.teamcode.robot.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

class slides{
        public DcMotor leftslide;
        public DcMotor rightslide;
        public DcMotor hslide;

        public slides(DcMotor leftslide, DcMotor rightslide, DcMotor hslide) {
            this.leftslide = leftslide;
            this.rightslide = rightslide;
            this.hslide = hslide;
            reset();
        }

        public void reset() {
            this.leftslide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            this.rightslide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            this.hslide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            this.leftslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            this.rightslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            this.hslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        public void setSlidePositions(int leftslide, int hslide) {
            this.leftslide.setTargetPosition(leftslide);
            this.rightslide.setTargetPosition(leftslide);
            this.hslide.setTargetPosition(hslide);
        }

        public boolean done() {
            return (this.leftslide.getCurrentPosition() == this.leftslide.getTargetPosition()) &&
                    (this.rightslide.getCurrentPosition() == this.rightslide.getTargetPosition()) &&
                    (this.hslide.getCurrentPosition() == this.hslide.getTargetPosition());
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
        private pincher pincher ;
        private slides slides;

        public Grabber2(DcMotor leftslide, DcMotor rightslide, DcMotor hslide, Servo pinch) {
            this.pincher = new pincher(pinch);
            this.slides = new slides(leftslide, rightslide, hslide);
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
                    slides.setSlidePositions(100,100);
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
