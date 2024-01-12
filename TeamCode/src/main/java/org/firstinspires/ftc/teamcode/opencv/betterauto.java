package org.firstinspires.ftc.teamcode.pandara506.opencv;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.pandara506.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.pandara506.opencv.PipelineRedBack;
import org.firstinspires.ftc.teamcode.pandara506.opencv.PipelineRedFront;
import org.firstinspires.ftc.teamcode.pandara506.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.Vector;
import java.util.concurrent.TimeUnit;

/*
 * This is a simple routine to test turning capabilities.
 */
@Config
@Autonomous(group = "autoRed")
public class betterauto extends LinearOpMode {

    OpenCvCamera webCam;
    public PipelineRedFront detector;
    public  String position = "Insert Here";
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
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
        webCam.startStreaming(640, 480, OpenCvCameraRotation.UPSIDE_DOWN);
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

        Trajectory traj1a = drive.trajectoryBuilder(new Pose2d())
                .splineTo(new Vector2d(30, 9), Math.toRadians(90))
                .build();
        Trajectory traj2a = drive.trajectoryBuilder(traj1a.end())
                .back(15)
                .build();
        Trajectory traj3a = drive.trajectoryBuilder(traj2a.end())
                .splineTo(new Vector2d(36, -28.5), Math.toRadians(-90))
                .build();
        TrajectorySequence traj4a = drive.trajectorySequenceBuilder(traj3a.end())
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(10, Math.toRadians(364.63481051106623), 12.35))
                .forward(8)
                .build();
        Trajectory traj5a = drive.trajectoryBuilder(traj4a.end())
                .back(8)
                .build();
        Trajectory traj6a = drive.trajectoryBuilder(traj5a.end())
                .back(8)
                .build();

        Trajectory traj1b = drive.trajectoryBuilder(new Pose2d())
                .forward(30)
                .build();
        Trajectory traj2b = drive.trajectoryBuilder(traj1b.end())
                .back(15)
                .build();
        Trajectory traj3b = drive.trajectoryBuilder(traj2b.end())
                .splineTo(new Vector2d(29, -29), Math.toRadians(-90))
                .build();
        Trajectory traj4b = drive.trajectoryBuilder(traj3b.end())
                .forward(7)
                .build();
        Trajectory traj5b = drive.trajectoryBuilder(traj4a.end())
                .back(8)
                .build();
        Trajectory traj6b = drive.trajectoryBuilder(traj5a.end())
                .back(8)
                .build();

        Trajectory traj1c = drive.trajectoryBuilder(new Pose2d())
                .splineTo(new Vector2d(23, -6), Math.toRadians(-20))
                .build();
        Trajectory traj2c = drive.trajectoryBuilder(traj1c.end())
                .back(10)
                .build();
        Trajectory traj3c = drive.trajectoryBuilder(traj2c.end())
                .splineTo(new Vector2d(22, -29), Math.toRadians(-90))
                .build();
        Trajectory traj4c = drive.trajectoryBuilder(traj3c.end())
                .forward(6)
                .build();
        Trajectory traj5c = drive.trajectoryBuilder(traj4a.end())
                .back(8)
                .build();
        Trajectory traj6c = drive.trajectoryBuilder(traj5a.end())
                .back(8)
                .build();

        //Cycle trajectories
        Pose2d end = new Pose2d();
        Trajectory trajCycle1 = drive.trajectoryBuilder(end)
                .splineTo(new Vector2d(56, 20), Math.toRadians(90))
                .build();

        Vector2d end3 = new Vector2d(49, 70);
        double end3Heding = Math.toRadians(92);
        Trajectory trajCycle2 = drive.trajectoryBuilder(trajCycle1.end())
                .splineTo(end3, end3Heding)
                .build();
        TrajectorySequence trajCycle22 = drive.trajectorySequenceBuilder(trajCycle2.end())
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(10, Math.toRadians(364.63481051106623), 12.35))
                .forward(7.5)
                .build();
        Trajectory trajCycle3 = drive.trajectoryBuilder(trajCycle22.end())
                .back(8)
                .build();
        //over to next claw
        double strafeDist = 4;
        TrajectorySequence trajCycle3b = drive.trajectorySequenceBuilder(trajCycle3.end())
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(10, Math.toRadians(364.63481051106623), 12.35))
                .strafeLeft(strafeDist)
                .forward(8)
                .build();
        //Cycle backwards
        TrajectorySequence trajCycle4 = drive.trajectorySequenceBuilder(trajCycle3b.end())
                .back(10)
                .splineTo(new Vector2d(53, -25), Math.toRadians(-90))
                .build();

        Vector2d end2 = new Vector2d(31, -29);
        TrajectorySequence trajCycle5 = drive.trajectorySequenceBuilder(trajCycle4.end())
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(70, Math.toRadians(100), 12.35))
                .splineTo(end2, Math.toRadians(-90))
                .build();
        TrajectorySequence trajCycle6 = drive.trajectorySequenceBuilder(trajCycle5.end())
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(10, Math.toRadians(364.63481051106623), 12.35))
                .forward(7)
                .build();
        Trajectory trajCycle7 = drive.trajectoryBuilder(trajCycle6.end())
                .back(4)
                .build();
        TrajectorySequence trajCycle8 = drive.trajectorySequenceBuilder(trajCycle7.end())
                .forward(1)
                .build();

        waitForStart();
        switch (position) {
            case "Left":
                drive.followTrajectory(traj1a);
                drive.clawLeft.setPosition(leftOpenPos);
                drive.wrist.setPosition(0.34);
                drive.followTrajectory(traj2a);
                drive.followTrajectory(traj3a);
                drive.slide.setPower(0.9);
                drive.slide.setTargetPosition(1300);
                drive.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                while (opModeIsActive() && drive.slide.isBusy()) {
                }
                drive.followTrajectorySequence(traj4a);
                drive.clawRight.setPosition(rightOpenPos);
                drive.followTrajectory(traj5a);
                drive.slide.setPower(0.7);
                drive.slide.setTargetPosition(0);
                drive.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                while (opModeIsActive() && drive.slide.isBusy()) {
                }
                drive.wrist.setPosition(0.169);
                drive.followTrajectory(traj6a);
                strafeDist = 4.5;
                end = traj6a.end();
                end3 = new Vector2d(50.38, 70);
                end3Heding = Math.toRadians(92);
                end2 = new Vector2d(22, -28);
                break;
            case "Center":
                drive.followTrajectory(traj1b);
                drive.clawLeft.setPosition(0.2);
                drive.wrist.setPosition(0.34);
                drive.followTrajectory(traj2b);
                drive.followTrajectory(traj3b);
                drive.slide.setPower(0.9);
                drive.slide.setTargetPosition(1300);
                drive.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                while (opModeIsActive() && drive.slide.isBusy()) {
                }
                drive.followTrajectory(traj4b);
                drive.clawRight.setPosition(0.27);
                drive.followTrajectory(traj5b);
                drive.slide.setPower(0.7);
                drive.slide.setTargetPosition(0);
                drive.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                while (opModeIsActive() && drive.slide.isBusy()) {
                }
                drive.wrist.setPosition(0.169);
                drive.followTrajectory(traj6b);
                strafeDist = 4;
                end = traj6b.end();
                end3 = new Vector2d(49.5, 70);
                end3Heding = Math.toRadians(92);
                end2 = new Vector2d(29.5, -28);
                break;
            default:
                drive.followTrajectory(traj1c);
                drive.clawLeft.setPosition(0.2);
                drive.wrist.setPosition(0.34);
                drive.followTrajectory(traj2c);
                drive.followTrajectory(traj3c);
                drive.slide.setPower(0.9);
                drive.slide.setTargetPosition(1300);
                drive.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                while (opModeIsActive() && drive.slide.isBusy()) {
                }
                drive.followTrajectory(traj4c);
                drive.clawRight.setPosition(0.27);
                drive.followTrajectory(traj5c);
                drive.slide.setPower(0.7);
                drive.slide.setTargetPosition(0);
                drive.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                while (opModeIsActive() && drive.slide.isBusy()) {
                }
                drive.wrist.setPosition(0.169);
                drive.followTrajectory(traj6c);
                strafeDist = 0.1;
                end = traj6c.end();
                end3 = new Vector2d(52, 70);
                end3Heding = Math.toRadians(92);
                end2 = new Vector2d(30, -28);
                break;
        }
        //Cycling
        trajCycle1 = drive.trajectoryBuilder(end)
                .splineTo(new Vector2d(53, 20), Math.toRadians(90))
                .build();
        drive.followTrajectory(trajCycle1);
        trajCycle2 = drive.trajectoryBuilder(trajCycle1.end())
                .splineTo(end3, end3Heding)
                .build();
        drive.followTrajectory(trajCycle2);
        drive.slide.setTargetPosition(390);
        drive.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (opModeIsActive() && drive.slide.isBusy()) {
        }
        trajCycle22 = drive.trajectorySequenceBuilder(trajCycle2.end())
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(10, Math.toRadians(364.63481051106623), 12.35))
                .forward(7)
                .build();
        drive.followTrajectorySequence(trajCycle22);
        sleep(200);
        //grab and move left
        drive.clawLeft.setPosition(leftClosePos);
        sleep(200);
        drive.followTrajectory(trajCycle3);
        drive.slide.setPower(0.7);
        drive.slide.setTargetPosition(320);
        drive.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (opModeIsActive() && drive.slide.isBusy()) {
        }
        trajCycle3b = drive.trajectorySequenceBuilder(trajCycle3.end())
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(10, Math.toRadians(364.63481051106623), 12.35))
                .strafeLeft(strafeDist)
                .forward(8)
                .build();
        drive.followTrajectorySequence(trajCycle3b);
        drive.clawRight.setPosition(rightClosePos);
        sleep(200);
        drive.wrist.setPosition(0.34);
        drive.followTrajectorySequence(trajCycle4);
        trajCycle5 = drive.trajectorySequenceBuilder(trajCycle4.end())
                .splineTo(end2, Math.toRadians(-90))
                .build();
        drive.followTrajectorySequence(trajCycle5);
        trajCycle6 = drive.trajectorySequenceBuilder(trajCycle5.end())
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(10, Math.toRadians(364.63481051106623), 12.35))
                .forward(8)
                .build();
        //cycle slidesd up
        drive.slide.setPower(0.9);
        drive.slide.setTargetPosition(1300);
        drive.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (opModeIsActive() && drive.slide.isBusy()) {
        }
        //forward and drop
        drive.followTrajectorySequence(trajCycle6);
        drive.clawLeft.setPosition(leftOpenPos);
        drive.clawRight.setPosition(rightOpenPos);
        //back and slide down
        drive.slide.setPower(0.7);
        drive.slide.setTargetPosition(1);
        drive.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (opModeIsActive() && drive.slide.isBusy()) {
        }
        drive.wrist.setPosition(0.169);
        drive.followTrajectory(trajCycle7);
    }

}