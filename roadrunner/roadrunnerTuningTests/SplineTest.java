package org.firstinspires.ftc.teamcode.pandara506.roadrunner.roadrunnerTuningTests;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pandara506.roadrunner.Hardware;
import org.firstinspires.ftc.teamcode.pandara506.trajectorysequence.TrajectorySequence;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive")
@Disabled
public class SplineTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Hardware drive = new Hardware(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

        TrajectorySequence traj = drive.trajectorySequenceBuilder(new Pose2d())
                .setVelConstraint(Hardware.getVelocityConstraint(30, Math.toRadians(304), 11.19))
                .splineTo(new Vector2d(24, 24), 0)
                .build();

        drive.followTrajectorySequence(traj);

        sleep(2000);

        drive.followTrajectory(
                drive.trajectoryBuilder(traj.end(), true)
                        .splineTo(new Vector2d(0, 0), Math.toRadians(180))
                        .build()
        );
    }
}
