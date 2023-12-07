package org.firstinspires.ftc.teamcode.pandara506;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(group = "drive")
public class Auto extends LinearOpMode{
    int spikeNum = 3;
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(-36,  -63.625, Math.toRadians(90));

        drive.setPoseEstimate(startPose);

        waitForStart();

        if (isStopRequested()) return;
        //1
        TrajectorySequence trajSeq1a = drive.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(-36, -37))
                .turn(Math.toRadians(90))
                .build();
        TrajectorySequence trajSeq1b = drive.trajectorySequenceBuilder(trajSeq1a.end())
                .lineTo(new Vector2d(49, -37))
                .strafeRight(6)
                .build();
        //2
        TrajectorySequence trajSeq2a = drive.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(-36, -30))
                //.waitSeconds(3)
                .build();
        TrajectorySequence trajSeq2b = drive.trajectorySequenceBuilder(trajSeq2a.end())
                .forward(-7)
                .turn(Math.toRadians(90))
                .lineTo(new Vector2d(50, -37))
                .build();
        //3
        TrajectorySequence trajSeq3a = drive.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(-36, -37))
                .turn(Math.toRadians(-90))
                .forward(1)
                .waitSeconds(0.2)
                .build();
        TrajectorySequence trajSeq3b = drive.trajectorySequenceBuilder(trajSeq3a.end())
                .forward(-1)
                .turn(Math.toRadians(180))
                .lineTo(new Vector2d(-36, -60))
                .lineTo(new Vector2d(40, -60))
                .waitSeconds(0.1)
                .lineTo(new Vector2d(50, -42))
                .build();

        if(spikeNum == 1) {
            drive.followTrajectorySequence(trajSeq1a);
            drive.followTrajectorySequence(trajSeq1b);
        }
        if(spikeNum == 2){
            drive.followTrajectorySequence(trajSeq2a);
            drive.followTrajectorySequence(trajSeq2b);
        }
        if(spikeNum == 3){
            drive.followTrajectorySequence(trajSeq3a);
            drive.followTrajectorySequence(trajSeq3b);
        }
    }
}
