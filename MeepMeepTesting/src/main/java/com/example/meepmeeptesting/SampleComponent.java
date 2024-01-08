package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequence;

//import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class SampleComponent extends Component {
    public SampleComponent(Robot robot) {
        super(robot);
    }

    private TrajectorySequence buildSequence() {
        return getRobot().getDrive().trajectorySequenceBuilder(getRobot().getDrive().getPoseEstimate())
                .lineTo(new Vector2d(10, 10))
                .build();
    }
    @Override
    public void drive() {
        getRobot().getDrive().followTrajectorySequence(buildSequence());
    }
}
