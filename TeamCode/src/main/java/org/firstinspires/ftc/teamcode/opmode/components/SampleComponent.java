package org.firstinspires.ftc.teamcode.opmode.components;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.drive.Robot;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class SampleComponent extends Component {
    public SampleComponent(Robot robot) {
        super(robot);
    }

    private TrajectorySequence buildSequence() {
        return getRobot().getDrive().trajectorySequenceBuilder(new Pose2d())
                .lineTo(new Vector2d(10, 10))
                .build();
    }
    @Override
    public void drive() {
        getRobot().getDrive().followTrajectorySequence(buildSequence());
    }
}
