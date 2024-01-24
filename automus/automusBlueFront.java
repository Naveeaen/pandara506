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
import org.firstinspires.ftc.teamcode.pandara506.camera.PipelineBlueFront;
import org.firstinspires.ftc.teamcode.pandara506.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

/*
 * This is a simple routine to test turning capabilities.
 */
@Config
@Autonomous(group = "autoBlue")
public class automusBlueFront extends LinearOpMode {

    OpenCvCamera webCam;
    public PipelineBlueFront detector;
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
        drive.clawLeft.setPosition(leftClosePos);
        drive.clawRight.setPosition(rightClosePos);

        //vision cam
        //cameraqa
        int cameraMotionViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        detector = new PipelineBlueFront();
        webCam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "wc1"), cameraMotionViewId);
        webCam.openCameraDevice();
        FtcDashboard.getInstance().startCameraStream(webCam, 0);
        webCam.startStreaming(640,480, OpenCvCameraRotation.UPSIDE_DOWN);
        webCam.setPipeline(detector);
        while (!isStarted() && !isStopRequested()) {
            position = detector.position;
            telemetry.addData("position", position);
            telemetry.addData("leftBlue", detector.leftBlue);
            telemetry.addData("centerBlue", detector.centerBlue);
            telemetry.addData("rightBlue", detector.rightBlue);
            telemetry.update();

            dashboardTelemetry.addData("position", position);
            dashboardTelemetry.addData("leftBlue", detector.leftBlue);
            dashboardTelemetry.addData("centerBlue", detector.centerBlue);
            dashboardTelemetry.addData("rightBlue", detector.rightBlue);
            dashboardTelemetry.update();

            dashboardTelemetry.addData("position", position);
        }

        Trajectory traj1a = drive.trajectoryBuilder(new Pose2d())
                .splineTo(new Vector2d(25,1),Math.toRadians(30))
                .build();
        Trajectory traj2a = drive.trajectoryBuilder(traj1a.end())
                .back(10)
                .splineTo(new Vector2d(16.5,28.5),Math.toRadians(-90))
                .build();
        Trajectory traj3a = drive.trajectoryBuilder(traj2a.end())
                .forward(8)
                .build();
        Trajectory traj4a = drive.trajectoryBuilder(traj3a.end())
                .back(10)
                .build();
        TrajectorySequence traj5a = drive.trajectorySequenceBuilder(traj4a.end())
                //.strafeRight(30)
                .strafeLeft(15)
                .forward(20)
                .build();

        Trajectory traj1b = drive.trajectoryBuilder(new Pose2d())
                .splineTo(new Vector2d(28,-5),Math.toRadians(0))
                .build();
        Trajectory traj2b = drive.trajectoryBuilder(traj1b.end())
                .back(10)
                .splineTo(new Vector2d(24.2,28.5),Math.toRadians(-90))
                .build();
        Trajectory traj3b = drive.trajectoryBuilder(traj2b.end())
                .forward(8)
                .build();
        Trajectory traj4b = drive.trajectoryBuilder(traj3b.end())
                .back(10)
                .build();
        TrajectorySequence traj5b = drive.trajectorySequenceBuilder(traj4b.end())
                //.strafeRight(20)
                .strafeLeft(20)
                .forward(20)
                .build();

        Trajectory traj1c = drive.trajectoryBuilder(new Pose2d())
                .splineTo(new Vector2d(25,-13),Math.toRadians(-30))
                .build();
        Trajectory traj2c = drive.trajectoryBuilder(traj1c.end())
                .back(10)
                .splineTo(new Vector2d(30,28.5),Math.toRadians(-90))
                .build();
        Trajectory traj3c = drive.trajectoryBuilder(traj2c.end())
                .forward(8.5)
                .build();
        Trajectory traj4c = drive.trajectoryBuilder(traj3c.end())
                .back(10)
                .build();
        TrajectorySequence traj5c = drive.trajectorySequenceBuilder(traj4c.end())
                //.strafeRight(15)
                .strafeLeft(30)
                .forward(20)
                .build();

        //Cycle trajectories
        /*Pose2d end = new Pose2d();
        Trajectory trajCycle1 = drive.trajectoryBuilder(end)
                .splineTo(new Vector2d(60, 20), Math.toRadians(-80))
                .build();

        double stackDist = 50;
        Trajectory trajCycle2 = drive.trajectoryBuilder(trajCycle1.end())
                .forward(stackDist)
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
                .build();*/

        waitForStart();
        int cameraZone = 1;
        switch (position) {
            case "Left":
                drive.followTrajectory(traj1a);
                drive.clawLeft.setPosition(leftOpenPos);
                drive.wrist.setPosition(0.34);
                drive.followTrajectory(traj2a);
                drive.slide.setPower(0.9);
                drive.slide.setTargetPosition(1300);
                drive.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                while (opModeIsActive() && drive.slide.isBusy()) {
                }
                drive.followTrajectory(traj3a);
                sleep(200);
                drive.clawRight.setPosition(rightOpenPos);
                sleep(200);
                drive.followTrajectory(traj4a);
                drive.slide.setPower(0.7);
                drive.slide.setTargetPosition(1);
                drive.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                while (opModeIsActive() && drive.slide.isBusy()){
                }
                drive.clawLeft.setPosition(leftClosePos);
                drive.clawRight.setPosition(rightClosePos);
                drive.wrist.setPosition(0.169);
                drive.followTrajectorySequence(traj5a);
                //end = traj5a.end();
                break;
            case "Center":
                drive.followTrajectory(traj1b);
                drive.clawLeft.setPosition(leftOpenPos);
                drive.wrist.setPosition(0.34);
                drive.followTrajectory(traj2b);
                drive.slide.setPower(0.9);
                drive.slide.setTargetPosition(1300);
                drive.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                while (opModeIsActive() && drive.slide.isBusy()) {
                }
                drive.followTrajectory(traj3b);
                sleep(200);
                drive.clawRight.setPosition(rightOpenPos);
                sleep(200);
                drive.slide.setPower(0.7);
                drive.slide.setTargetPosition(1);
                drive.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                while (opModeIsActive() && drive.slide.isBusy()){
                }
                drive.clawLeft.setPosition(leftClosePos);
                drive.clawRight.setPosition(rightClosePos);
                drive.wrist.setPosition(0.169);
                drive.followTrajectorySequence(traj5b);
                break;
            default:
                drive.followTrajectory(traj1c);
                drive.clawLeft.setPosition(leftOpenPos);
                drive.wrist.setPosition(0.34);
                drive.followTrajectory(traj2c);
                drive.slide.setPower(0.9);
                drive.slide.setTargetPosition(1300);
                drive.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                while (opModeIsActive() && drive.slide.isBusy()) {
                }
                drive.followTrajectory(traj3c);
                sleep(200);
                drive.clawRight.setPosition(rightOpenPos);
                sleep(200);
                drive.followTrajectory(traj4c);
                drive.slide.setPower(0.7);
                drive.slide.setTargetPosition(1);
                drive.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                while (opModeIsActive() && drive.slide.isBusy()){
                }
                drive.clawLeft.setPosition(leftClosePos);
                drive.clawRight.setPosition(rightClosePos);
                drive.wrist.setPosition(0.169);
                drive.followTrajectorySequence(traj5c);
        }
        //Cycling
        /*trajCycle1 = drive.trajectoryBuilder(end)
                .splineTo(new Vector2d(51, 20), Math.toRadians(-87))
                .build();
        drive.followTrajectory(trajCycle1);
        trajCycle2 = drive.trajectoryBuilder(trajCycle1.end())
                .forward(stackDist)
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
        drive.followTrajectory(trajCycle7);*/
    }
}