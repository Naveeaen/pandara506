package org.firstinspires.ftc.teamcode.pandara506.automus;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.pandara506.roadrunner.Hardware;
import org.firstinspires.ftc.teamcode.pandara506.camera.PipelineRedFront;
import org.firstinspires.ftc.teamcode.pandara506.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

/*
 * This is a simple routine to test turning capabilities.
 */
@Config
@Autonomous(group = "autoRed")
public class automusRedFrontGolden extends LinearOpMode {

    OpenCvCamera webCam;
    public PipelineRedFront detector;
    public String position = "Insert Here";
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    @Override
    public void runOpMode() throws InterruptedException {
        Hardware drive = new Hardware(hardwareMap);
        double leftOpenPos = drive.clawLeftOpenPos;
        double leftClosePos = drive.clawLeftClosePos;
        double rightOpenPos = drive.clawRightOpenPos;
        double rightClosePos = drive.clawRightClosePos;
        drive.clawLeft.setPosition(leftClosePos);
        drive.clawRight.setPosition(rightClosePos);

        //vision cam
        //cameraqa
        int cameraMotionViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        detector = new PipelineRedFront();
        webCam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "wc1"), cameraMotionViewId);
        webCam.openCameraDevice();
        FtcDashboard.getInstance().startCameraStream(webCam, 0);
        webCam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
        webCam.setPipeline(detector);
        while (!isStarted() && !isStopRequested()) {
            position = detector.position;
            telemetry.addData("position", position);
            telemetry.addData("leftRed", detector.leftRed);
            telemetry.addData("centerRed", detector.centerRed);
            telemetry.addData("rightRed", detector.rightRed);
            telemetry.update();

            dashboardTelemetry.addData("position", position);
            dashboardTelemetry.addData("leftRed", detector.leftRed);
            dashboardTelemetry.addData("centerRed", detector.centerRed);
            dashboardTelemetry.addData("rightRed", detector.rightRed);
            dashboardTelemetry.update();

            dashboardTelemetry.addData("position", position);
        }

        //left
        Trajectory traj1a = drive.trajectoryBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(31, 5, Math.toRadians(90)))
                .build();
        Trajectory traj2a = drive.trajectoryBuilder(traj1a.end())
                .lineToLinearHeading(new Pose2d(33, -38.5, Math.toRadians(-90)))
                .build();
        TrajectorySequence traj5a = drive.trajectorySequenceBuilder(traj2a.end())
                .lineToLinearHeading(new Pose2d(1.8, 0, Math.toRadians(90)))
                .lineTo(new Vector2d(1.8, 72))
                .strafeTo(new Vector2d(21.7, 72))
                .forward(6)
                .build();
        TrajectorySequence trajPixel2aa = drive.trajectorySequenceBuilder(traj5a.end())
                .back(5)
                .strafeRight(4.5)
                .build();
        Trajectory trajPixel2ab = drive.trajectoryBuilder(trajPixel2aa.end())
                .forward(5)
                .build();
        TrajectorySequence traj6a = drive.trajectorySequenceBuilder(trajPixel2ab.end())
                .back(5)
                .lineToLinearHeading(new Pose2d(1, 60, Math.toRadians(-90)))
                .lineTo(new Vector2d(1, 0))
                .build();
        Trajectory traj7a = drive.trajectoryBuilder(traj6a.end())
                .lineToLinearHeading(new Pose2d(21, -37.5, Math.toRadians(-90)))
                .build();
        Trajectory traj8a = drive.trajectoryBuilder(traj7a.end())
                .back(10)
                .build();
        TrajectorySequence traj9a = drive.trajectorySequenceBuilder(traj8a.end())
                .lineToLinearHeading(new Pose2d(1, -37, Math.toRadians(90)))
                .back(5)
                .build();

        //center
        Trajectory traj1b = drive.trajectoryBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(31, -3, Math.toRadians(30)))
                .build();
        Trajectory traj2b = drive.trajectoryBuilder(traj1b.end())
                .lineToLinearHeading(new Pose2d(25, -38, Math.toRadians(-90)))
                .build();
        TrajectorySequence traj5b = drive.trajectorySequenceBuilder(traj2b.end())
                .lineToLinearHeading(new Pose2d(1, 0, Math.toRadians(90)))
                .lineTo(new Vector2d(1, 72))
                .strafeTo(new Vector2d(22, 72))
                .forward(6)
                .build();
        TrajectorySequence trajPixel2ba = drive.trajectorySequenceBuilder(traj5b.end())
                .back(5)
                .strafeRight(5)
                .build();
        Trajectory trajPixel2bb = drive.trajectoryBuilder(trajPixel2ba.end())
                .forward(6)
                .build();
        TrajectorySequence traj6b = drive.trajectorySequenceBuilder(trajPixel2bb.end())
                .back(5)
                .lineToLinearHeading(new Pose2d(0.5, 60, Math.toRadians(-90)))
                .lineTo(new Vector2d(1, 0))
                .build();
        Trajectory traj7b = drive.trajectoryBuilder(traj6b.end())
                .lineToLinearHeading(new Pose2d(28, -37, Math.toRadians(-90)))
                .build();
        Trajectory traj8b = drive.trajectoryBuilder(traj7b.end())
                .back(10)
                .build();
        TrajectorySequence traj9b = drive.trajectorySequenceBuilder(traj8b.end())
                .lineToLinearHeading(new Pose2d(1, -36.5, Math.toRadians(90)))
                .back(5)
                .build();

        //right
        Trajectory traj1c = drive.trajectoryBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(25, -14, Math.toRadians(30)))
                .build();
        Trajectory traj2c = drive.trajectoryBuilder(traj1c.end())
                .lineToLinearHeading(new Pose2d(22.5, -38, Math.toRadians(-90)))
                .build();
        TrajectorySequence traj5c = drive.trajectorySequenceBuilder(traj2c.end())
                .lineToLinearHeading(new Pose2d(1, 0, Math.toRadians(90)))
                .lineTo(new Vector2d(1, 72))
                .strafeTo(new Vector2d(21.5, 72))
                .forward(6)
                .build();
        TrajectorySequence trajPixel2ca = drive.trajectorySequenceBuilder(traj5c.end())
                .back(5)
                .strafeRight(5)
                .build();
        Trajectory trajPixel2cb = drive.trajectoryBuilder(trajPixel2ca.end())
                .forward(5)
                .build();
        TrajectorySequence traj6c = drive.trajectorySequenceBuilder(trajPixel2cb.end())
                .back(5)
                .lineToLinearHeading(new Pose2d(0.5, 60, Math.toRadians(-90)))
                .lineTo(new Vector2d(1, 0))
                .build();
        Trajectory traj7c = drive.trajectoryBuilder(traj6a.end())
                .lineToLinearHeading(new Pose2d(28, -37, Math.toRadians(-90)))
                .build();
        Trajectory traj8c = drive.trajectoryBuilder(traj7c.end())
                .back(10)
                .build();
        TrajectorySequence traj9c = drive.trajectorySequenceBuilder(traj8c.end())
                .lineToLinearHeading(new Pose2d(1, -37, Math.toRadians(90)))
                .back(5)
                .build();

        waitForStart();
        switch (position) {
            case "Left":
                drive.followTrajectory(traj1a);
                drive.clawLeft.setPosition(leftOpenPos);
                drive.wrist.setPosition(0.34);
                //drive.followTrajectory(traj3a);
                drive.slide.setPower(0.9);
                drive.slide.setTargetPosition(1300);
                drive.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                drive.followTrajectory(traj2a);
                drive.clawRight.setPosition(rightOpenPos);
                sleep(200);
                drive.slide.setPower(0.7);
                drive.slide.setTargetPosition(300);
                drive.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                drive.wrist.setPosition(0.169);
                drive.followTrajectorySequence(traj5a);
                drive.clawRight.setPosition(rightClosePos);
                sleep(300);
                drive.slide.setPower(0.7);
                drive.slide.setTargetPosition(305);
                drive.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                drive.followTrajectorySequence(trajPixel2aa);
                drive.slide.setTargetPosition(210);
                drive.slide.setTargetPosition(210);
                drive.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                drive.followTrajectory(trajPixel2ab);
                drive.clawLeft.setPosition(leftClosePos);
                drive.followTrajectorySequence(traj6a);
                drive.wrist.setPosition(0.34);
                drive.slide.setPower(0.9);
                drive.slide.setTargetPosition(1300);
                drive.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                drive.followTrajectory(traj7a);
                drive.clawRight.setPosition(rightOpenPos);
                drive.clawLeft.setPosition(leftOpenPos);
                sleep(200);
                drive.wrist.setPosition(0.169);
                drive.slide.setPower(0.9);
                drive.slide.setTargetPosition(0);
                drive.followTrajectory(traj8a);
                drive.followTrajectorySequence(traj9a);
                break;
            case "Center":
                drive.followTrajectory(traj1b);
                drive.clawLeft.setPosition(leftOpenPos);
                drive.wrist.setPosition(0.34);
                drive.slide.setPower(0.9);
                drive.slide.setTargetPosition(1300);
                drive.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                drive.followTrajectory(traj2b);
                drive.clawRight.setPosition(rightOpenPos);
                sleep(200);
                drive.slide.setPower(0.7);
                drive.slide.setTargetPosition(330);
                drive.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                drive.wrist.setPosition(0.169);
                drive.followTrajectorySequence(traj5b);
                drive.clawRight.setPosition(rightClosePos);
                sleep(300);
                drive.slide.setPower(0.7);
                drive.slide.setTargetPosition(335);
                drive.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                drive.followTrajectorySequence(trajPixel2ba);
                drive.slide.setTargetPosition(230);
                drive.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                drive.followTrajectory(trajPixel2bb);
                drive.clawLeft.setPosition(leftClosePos);
                drive.followTrajectorySequence(traj6b);
                drive.wrist.setPosition(0.34);
                drive.slide.setPower(0.9);
                drive.slide.setTargetPosition(1300);
                drive.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                drive.followTrajectory(traj7b);
                drive.clawRight.setPosition(rightOpenPos);
                drive.clawLeft.setPosition(leftOpenPos);
                sleep(200);
                drive.wrist.setPosition(0.169);
                drive.slide.setPower(0.9);
                drive.slide.setTargetPosition(0);
                drive.followTrajectory(traj8b);
                drive.followTrajectorySequence(traj9b);
                break;
            default:
                drive.followTrajectory(traj1c);
                drive.clawLeft.setPosition(leftOpenPos);
                drive.wrist.setPosition(0.34);
                drive.slide.setPower(0.9);
                drive.slide.setTargetPosition(1300);
                drive.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                drive.followTrajectory(traj2c);
                drive.clawRight.setPosition(rightOpenPos);
                sleep(200);
                drive.slide.setPower(0.7);
                drive.slide.setTargetPosition(330);
                drive.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                drive.wrist.setPosition(0.169);
                drive.followTrajectorySequence(traj5c);
                drive.clawRight.setPosition(rightClosePos);
                sleep(300);
                drive.slide.setPower(0.7);
                drive.slide.setTargetPosition(335);
                drive.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                drive.followTrajectorySequence(trajPixel2ca);
                drive.slide.setTargetPosition(230);
                drive.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                drive.followTrajectory(trajPixel2cb);
                drive.clawLeft.setPosition(leftClosePos);
                drive.followTrajectorySequence(traj6c);
                drive.wrist.setPosition(0.34);
                drive.slide.setPower(0.9);
                drive.slide.setTargetPosition(1300);
                drive.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                drive.followTrajectory(traj7c);
                drive.clawRight.setPosition(rightOpenPos);
                drive.clawLeft.setPosition(leftOpenPos);
                sleep(200);
                drive.wrist.setPosition(0.169);
                drive.slide.setPower(0.9);
                drive.slide.setTargetPosition(0);
                drive.followTrajectory(traj8c);
                drive.followTrajectorySequence(traj9c);
        }
    }
}