package org.firstinspires.ftc.teamcode.robot.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;

public class Lift {
    public DcMotor slide;
    public DcMotor pitch;

    public Lift(DcMotor slide, DcMotor pitch) {
        this.slide = slide;
        this.pitch = pitch;
        reset();
    }

    public void reset() {
        this.slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.pitch.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void setPositions(int slide, int pitch) {
        this.slide.setTargetPosition(slide);
        this.pitch.setTargetPosition(pitch);

        this.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.pitch.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    // use .busy() instead??
    public boolean done() {
        return (this.slide.getCurrentPosition() == this.slide.getTargetPosition()) &&
                (this.pitch.getCurrentPosition() == this.pitch.getTargetPosition());
    }
}
