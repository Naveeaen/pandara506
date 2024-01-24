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
import org.firstinspires.ftc.teamcode.pandara506.camera.PipelineRedBack;
import org.firstinspires.ftc.teamcode.pandara506.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

/*
 * This is a simple routine to test turning capabilities.
 */
@Config
@Autonomous(group = "autoRed")
public class automusRedBack extends LinearOpMode {
    OpenCvCamera webCam;
    public PipelineRedBack detector;
    public  String position = "Insert Here";
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();



    @Override
    public void runOpMode() throws InterruptedException {

        Hardware drive = new Hardware(hardwareMap);
        double leftOpenPos = drive.clawLeftOpenPos;
        double leftClosePos = drive.clawLeftClosePos;
        double rightOpenPos = drive.clawRightOpenPos;
        double rightClosePos = drive.clawRightClosePos;

        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));
        drive.setPoseEstimate(startPose);

        //cameraqa
        int cameraMotionViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        detector = new PipelineRedBack();
        webCam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "wc1"), cameraMotionViewId);
        webCam.openCameraDevice();
        FtcDashboard.getInstance().startCameraStream(webCam, 0);
        webCam.startStreaming(640,480, OpenCvCameraRotation.UPSIDE_DOWN);
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
                .splineTo(new Vector2d(30,6),Math.toRadians(0))
                .build();
        Trajectory traj2a = drive.trajectoryBuilder(traj1a.end())
                .back(15)
                .build();
        Trajectory traj3a = drive.trajectoryBuilder(traj2a.end())
                .strafeRight(11)
                .build();
        Trajectory traj4a = drive.trajectoryBuilder(traj3a.end())
                .splineTo(new Vector2d(65,-9),Math.toRadians(-90))
                .build();
        Trajectory traj5a = drive.trajectoryBuilder(traj4a.end())
                .splineTo(new Vector2d(65,-40),Math.toRadians(-90))
                .build();
        Trajectory traj6a = drive.trajectoryBuilder(traj5a.end())
                .splineTo(new Vector2d(39,-83), Math.toRadians(-90))
                .build();
        Trajectory traj7a = drive.trajectoryBuilder(traj6a.end())
                .splineTo(new Vector2d(39,-90), Math.toRadians(-90))
                .build();
        Trajectory traj8a = drive.trajectoryBuilder(traj7a.end())
                .back(8)
                .build();
        TrajectorySequence traj9a = drive.trajectorySequenceBuilder(traj8a.end())
                .strafeLeft(15)
                .forward(20)
                .build();

        Trajectory traj1b = drive.trajectoryBuilder(new Pose2d())
                .splineTo(new Vector2d(43, 0), Math.toRadians(-180))
                .build();
        Trajectory traj2b = drive.trajectoryBuilder(traj1b.end())
                .back(20)
                .build();
        Trajectory traj3b = drive.trajectoryBuilder(traj2b.end())
                .splineTo(new Vector2d(60, -60), Math.toRadians(-90))
                .build();
        Trajectory traj4b = drive.trajectoryBuilder(traj3b.end())
                .splineTo(new Vector2d(28,-85), Math.toRadians(-90))
                .build();
        Trajectory traj5b = drive.trajectoryBuilder(traj4b.end())
                .forward(8)
                .build();
        Trajectory traj6b = drive.trajectoryBuilder(traj5b.end())
                .back(8)
                .build();
        TrajectorySequence traj7b = drive.trajectorySequenceBuilder(traj6b.end())
                .strafeLeft(20)
                .forward(20)
                .build();

        Trajectory traj1c = drive.trajectoryBuilder(new Pose2d())
                .splineTo(new Vector2d(33, -6), Math.toRadians(-90))
                .build();
        Trajectory traj2c = drive.trajectoryBuilder(traj1c.end())
                .back(10)
                .build();
        Trajectory traj3c = drive.trajectoryBuilder(traj2c.end())
                .strafeLeft(20)
                .build();
        Trajectory traj4c = drive.trajectoryBuilder(traj3c.end())
                .splineTo(new Vector2d(55,-60),Math.toRadians(-90))
                .build();
        Trajectory traj5c = drive.trajectoryBuilder(traj4c.end())
                .splineTo(new Vector2d(25,-83), Math.toRadians(-90))
                .build();
        Trajectory traj6c = drive.trajectoryBuilder(traj5c.end())
                .forward(9)
                .build();
        Trajectory traj7c = drive.trajectoryBuilder(traj6c.end())
                .back(8)
                .build();
        TrajectorySequence traj8c = drive.trajectorySequenceBuilder((traj7c.end()))
                .strafeLeft(25)
                .forward(20)
                .build();

        drive.clawLeft.setPosition(leftClosePos);
        drive.clawRight.setPosition(rightClosePos);

        waitForStart();
        int cameraZone = 2;
        switch (position) {
            case "Left":
                drive.followTrajectory(traj1a);
                drive.clawLeft.setPosition(leftOpenPos);
                drive.wrist.setPosition(0.34);
                drive.followTrajectory(traj2a);
                drive.followTrajectory(traj3a);
                drive.followTrajectory(traj4a);
                drive.followTrajectory(traj5a);
                drive.followTrajectory(traj6a);
                drive.slide.setPower(0.9);
                drive.slide.setTargetPosition(1300);
                drive.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                while (opModeIsActive() && drive.slide.isBusy()) {
                }
                drive.followTrajectory(traj7a);
                sleep(200);
                drive.clawRight.setPosition(rightOpenPos);
                sleep(200);
                drive.followTrajectory(traj8a);
                drive.slide.setPower(0.7);
                drive.slide.setTargetPosition(1);
                drive.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                while (opModeIsActive() && drive.slide.isBusy()){
                }
                drive.clawLeft.setPosition(leftClosePos);
                drive.clawRight.setPosition(rightClosePos);
                drive.wrist.setPosition(0.169);
                drive.followTrajectorySequence(traj9a);
                break;
            case "Center":
                drive.followTrajectory(traj1b);
                drive.clawLeft.setPosition(leftOpenPos);
                drive.wrist.setPosition(0.34);
                drive.followTrajectory(traj2b);
                drive.followTrajectory(traj3b);
                drive.followTrajectory(traj4b);
                drive.slide.setPower(0.9);
                drive.slide.setTargetPosition(1300);
                drive.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                while (opModeIsActive() && drive.slide.isBusy()) {
                }
                drive.followTrajectory(traj5b);
                sleep(200);
                drive.clawRight.setPosition(rightOpenPos);
                sleep(200);
                drive.followTrajectory(traj6b);
                drive.slide.setPower(0.7);
                drive.slide.setTargetPosition(1);
                drive.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                while (opModeIsActive() && drive.slide.isBusy()){
                }
                drive.clawLeft.setPosition(leftClosePos);
                drive.clawRight.setPosition(rightClosePos);
                drive.wrist.setPosition(0.169);
                drive.followTrajectorySequence(traj7b);
                break;
            default:
                drive.followTrajectory(traj1c);
                drive.clawLeft.setPosition(leftOpenPos);
                drive.wrist.setPosition(0.34);
                drive.followTrajectory(traj2c);
                drive.followTrajectory(traj3c);
                drive.followTrajectory(traj4c);
                drive.followTrajectory(traj5c);
                drive.slide.setPower(0.9);
                drive.slide.setTargetPosition(1300);
                drive.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                while (opModeIsActive() && drive.slide.isBusy()) {
                }
                drive.followTrajectory(traj6c);
                sleep(200);
                drive.clawRight.setPosition(rightOpenPos);
                sleep(200);
                drive.followTrajectory(traj7c);
                drive.slide.setPower(0.7);
                drive.slide.setTargetPosition(1);
                drive.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                while (opModeIsActive() && drive.slide.isBusy()){
                }
                drive.clawLeft.setPosition(leftClosePos);
                drive.clawRight.setPosition(rightClosePos);
                drive.wrist.setPosition(0.169);
                drive.followTrajectorySequence(traj8c);
                /*drive.slide.setPower(0.9);
                drive.slide.setTargetPosition(1300);
                drive.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                while (opModeIsActive() && drive.slide.isBusy()) {
                }
                drive.followTrajectory(traj4c);
                drive.clawRight.setPosition(0.27);*/
                break;
        }
    }
}