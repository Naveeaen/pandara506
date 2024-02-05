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
import org.firstinspires.ftc.teamcode.pandara506.roadrunner.DriveConstants;
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
    public String position = "Insert Here";
    int timeout = 0;
    String cycle = "";
    String park = "";
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
        webCam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
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

        //left
        TrajectorySequence traj1a = drive.trajectorySequenceBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(22, 10, Math.toRadians(-30)))
                .build();
        Trajectory traj1aa = drive.trajectoryBuilder(traj1a.end())
                .back(2)
                .build();
        TrajectorySequence traj2a = drive.trajectorySequenceBuilder(traj1aa.end())
                .lineToLinearHeading(new Pose2d(17.2, 35.5, Math.toRadians(90)))
                .setVelConstraint(Hardware.getVelocityConstraint(0.07 * DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .forward(6.4)
                .build();
        TrajectorySequence traj2a2 = drive.trajectorySequenceBuilder(traj2a.end())
                .back(6)
                .strafeRight(30)
                .forward(6)
                .build();
        TrajectorySequence traj3a = drive.trajectorySequenceBuilder(traj2a2.end()) //stack 1
                .lineToLinearHeading(new Pose2d(54, 15, Math.toRadians(-90)))
                .build();
        TrajectorySequence traj3a2 = drive.trajectorySequenceBuilder(traj3a.end())
                .lineToLinearHeading(new Pose2d(58, -70, Math.toRadians(-90)))
                .build();
        TrajectorySequence traj3a22 = drive.trajectorySequenceBuilder(traj3a2.end())
                .setVelConstraint(Hardware.getVelocityConstraint(0.1 * DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .forward(5.5)
                .build();
        Trajectory traj3a3 = drive.trajectoryBuilder(traj3a22.end())
                .back(7)
                .build();
        TrajectorySequence traj4a = drive.trajectorySequenceBuilder(traj3a3.end())
                .lineToLinearHeading(new Pose2d(54, 0, Math.toRadians(-90)))
                .build();
        TrajectorySequence traj5a = drive.trajectorySequenceBuilder(traj4a.end())
                .lineToLinearHeading(new Pose2d(23, 35.5, Math.toRadians(90)))
                .setVelConstraint(Hardware.getVelocityConstraint(0.15 * DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .forward(5)
                .build();
        TrajectorySequence traj7a = drive.trajectorySequenceBuilder(traj5a.end())
                .setVelConstraint(Hardware.getVelocityConstraint(0.25 * DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .lineTo(new Vector2d(29, 36.9)) // drop 2
                .build();

        TrajectorySequence traj3aa = drive.trajectorySequenceBuilder(traj7a.end())
                .lineToLinearHeading(new Pose2d(54, 5, Math.toRadians(-90)))
                .build();
        TrajectorySequence traj3aa2 = drive.trajectorySequenceBuilder(traj3aa.end())
                .lineToLinearHeading(new Pose2d(62, -73, Math.toRadians(-85)))
                .build();
        TrajectorySequence traj3aa22 = drive.trajectorySequenceBuilder(traj3aa2.end())
                .setVelConstraint(Hardware.getVelocityConstraint(0.1 * DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .forward(4)
                .build();
        Trajectory traj3aa3 = drive.trajectoryBuilder(traj3aa22.end())
                .back(6)
                .build();
        TrajectorySequence traj4aa = drive.trajectorySequenceBuilder(traj3aa3.end())
                .lineToLinearHeading(new Pose2d(54, 0, Math.toRadians(-90)))
                .build();
        TrajectorySequence traj5aa = drive.trajectorySequenceBuilder(traj4aa.end())
                .lineToLinearHeading(new Pose2d(23, 35.5, Math.toRadians(90)))
                .setVelConstraint(Hardware.getVelocityConstraint(0.15 * DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .forward(5)
                .build();
        TrajectorySequence traj6aa = drive.trajectorySequenceBuilder(traj5aa.end())
                .setVelConstraint(Hardware.getVelocityConstraint(0.25 * DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .lineTo(new Vector2d(29, 36.5)) // drop 3.2
                .build();
        TrajectorySequence traj6aaa = drive.trajectorySequenceBuilder(traj6aa.end())
                .back(3)
                .build();
        TrajectorySequence traj7aa = drive.trajectorySequenceBuilder(traj6aaa.end())
                .back(3)
                .strafeRight(20)
                .build();
        //center
        Trajectory traj1b = drive.trajectoryBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(28, 3, Math.toRadians(-30)))
                .build();
        Trajectory traj1bb = drive.trajectoryBuilder(traj1b.end())
                .back(2)
                .build();
        TrajectorySequence traj2b = drive.trajectorySequenceBuilder(traj1b.end())
                .lineToLinearHeading(new Pose2d(24, 35.5, Math.toRadians(90)))
                .setVelConstraint(Hardware.getVelocityConstraint(0.07 * DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .forward(6)
                .build();
        TrajectorySequence traj2b2 = drive.trajectorySequenceBuilder(traj2b.end())
                .back(6)
                .strafeRight(25)
                .forward(6)
                .build();
        TrajectorySequence traj3b = drive.trajectorySequenceBuilder(traj2b2.end()) //thuis b
                .lineToLinearHeading(new Pose2d(54, 5, Math.toRadians(-90)))
                .build();
        TrajectorySequence traj3b2 = drive.trajectorySequenceBuilder(traj3b.end())
                .lineToLinearHeading(new Pose2d(58, -73, Math.toRadians(-90)))
                .build();
        TrajectorySequence traj3b22 = drive.trajectorySequenceBuilder(traj3a2.end())
                .setVelConstraint(Hardware.getVelocityConstraint(0.1 * DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .forward(5)
                .build();
        Trajectory traj3b3 = drive.trajectoryBuilder(traj3b22.end())
                .back(6.5)
                .build();
        TrajectorySequence traj4b = drive.trajectorySequenceBuilder(traj3b3.end())
                .lineToLinearHeading(new Pose2d(54, 0, Math.toRadians(-90)))
                .build();
        TrajectorySequence traj5b = drive.trajectorySequenceBuilder(traj4b.end())
                .lineToLinearHeading(new Pose2d(23, 35.5, Math.toRadians(90)))
                .setVelConstraint(Hardware.getVelocityConstraint(0.075 * DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .forward(5)
                .build();
        TrajectorySequence traj7b = drive.trajectorySequenceBuilder(traj5b.end())
                .setVelConstraint(Hardware.getVelocityConstraint(0.25 * DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .lineTo(new Vector2d(29, 37.5)) // drop 2
                .build();

        TrajectorySequence traj3bb = drive.trajectorySequenceBuilder(traj7b.end())
                .lineToLinearHeading(new Pose2d(54, 5, Math.toRadians(-90)))
                .build();
        TrajectorySequence traj3bb2 = drive.trajectorySequenceBuilder(traj3bb.end())
                .lineToLinearHeading(new Pose2d(59, -74, Math.toRadians(-90)))
                .build();
        TrajectorySequence traj3bb22 = drive.trajectorySequenceBuilder(traj3bb2.end())
                .setVelConstraint(Hardware.getVelocityConstraint(0.15 * DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .forward(4)
                .build();
        Trajectory traj3bb3 = drive.trajectoryBuilder(traj3bb22.end())
                .back(6)
                .build();
        TrajectorySequence traj4bb = drive.trajectorySequenceBuilder(traj3bb3.end())
                .lineToLinearHeading(new Pose2d(54, 0, Math.toRadians(-90)))
                .build();
        TrajectorySequence traj5bb = drive.trajectorySequenceBuilder(traj4bb.end())
                .lineToLinearHeading(new Pose2d(23, 35.5, Math.toRadians(90)))
                .setVelConstraint(Hardware.getVelocityConstraint(0.15 * DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .forward(6)
                .build();
        TrajectorySequence traj6bb = drive.trajectorySequenceBuilder(traj5bb.end())
                .setVelConstraint(Hardware.getVelocityConstraint(0.25 * DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .lineTo(new Vector2d(29, 35.5)) // drop 3
                .build();
        TrajectorySequence traj6bbb = drive.trajectorySequenceBuilder(traj6bb.end())
                .back(3)
                .build();
        TrajectorySequence traj7bb = drive.trajectorySequenceBuilder(traj6bbb.end())
                .back(3)
                .strafeLeft(20)
                .build();


        //right
        Trajectory traj1c = drive.trajectoryBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(29, -4, Math.toRadians(-90)))
                .build();
        Trajectory traj1cc = drive.trajectoryBuilder(traj1c.end())
                .back(2)
                .build();
        TrajectorySequence traj2c = drive.trajectorySequenceBuilder(traj1c.end())
                .lineToLinearHeading(new Pose2d(29, 35.5, Math.toRadians(90)))
                .setVelConstraint(Hardware.getVelocityConstraint(0.07 * DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .forward(6)
                .build();
        TrajectorySequence traj2c2 = drive.trajectorySequenceBuilder(traj2c.end())
                .back(6)
                .strafeRight(20)
                .forward(6)
                .build();
        TrajectorySequence traj3c = drive.trajectorySequenceBuilder(traj2c2.end()) //thuis b
                .lineToLinearHeading(new Pose2d(55, 5, Math.toRadians(-90)))
                .build();
        TrajectorySequence traj3c2 = drive.trajectorySequenceBuilder(traj3c.end())
                .lineToLinearHeading(new Pose2d(55.9, -73, Math.toRadians(-90)))
                .build();
        TrajectorySequence traj3c22 = drive.trajectorySequenceBuilder(traj3c2.end())
                .setVelConstraint(Hardware.getVelocityConstraint(0.1 * DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .forward(4)
                .build();
        Trajectory traj3c3 = drive.trajectoryBuilder(traj3c22.end())
                .back(6.5)
                .build();
        TrajectorySequence traj4c = drive.trajectorySequenceBuilder(traj3c3.end())
                .lineToLinearHeading(new Pose2d(55, 0, Math.toRadians(-90)))
                .build();
        TrajectorySequence traj5c = drive.trajectorySequenceBuilder(traj4c.end())
                .lineToLinearHeading(new Pose2d(23, 35.5, Math.toRadians(90)))
                .setVelConstraint(Hardware.getVelocityConstraint(0.15 * DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .forward(5)
                .build();
        TrajectorySequence traj7c = drive.trajectorySequenceBuilder(traj5c.end())
                .setVelConstraint(Hardware.getVelocityConstraint(0.25 * DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .lineTo(new Vector2d(29, 37.5)) // drop 2
                .build();

        TrajectorySequence traj3cc = drive.trajectorySequenceBuilder(traj7c.end())
                .lineToLinearHeading(new Pose2d(54, 5, Math.toRadians(-90)))
                .build();
        TrajectorySequence traj3cc2 = drive.trajectorySequenceBuilder(traj3cc.end())
                .lineToLinearHeading(new Pose2d(60, -74, Math.toRadians(-90)))
                .build();
        TrajectorySequence traj3cc22 = drive.trajectorySequenceBuilder(traj3cc2.end())
                .setVelConstraint(Hardware.getVelocityConstraint(0.15 * DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .forward(4)
                .build();
        Trajectory traj3cc3 = drive.trajectoryBuilder(traj3cc22.end())
                .back(6)
                .build();
        TrajectorySequence traj4cc = drive.trajectorySequenceBuilder(traj3cc3.end())
                .lineToLinearHeading(new Pose2d(54, 0, Math.toRadians(-90)))
                .build();
        TrajectorySequence traj5cc = drive.trajectorySequenceBuilder(traj4cc.end())
                .lineToLinearHeading(new Pose2d(23, 35.5, Math.toRadians(90)))
                .setVelConstraint(Hardware.getVelocityConstraint(0.15 * DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .forward(6)
                .build();
        TrajectorySequence traj6cc = drive.trajectorySequenceBuilder(traj5cc.end())
                .setVelConstraint(Hardware.getVelocityConstraint(0.25 * DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .lineTo(new Vector2d(29, 35.5)) // drop 3
                .build();
        TrajectorySequence traj6ccc = drive.trajectorySequenceBuilder(traj6cc.end())
                .back(3)
                .build();
        TrajectorySequence traj7cc = drive.trajectorySequenceBuilder(traj6ccc.end())
                .back(3)
                .strafeLeft(20)
                .build();

        waitForStart();
        sleep(timeout);
        switch (position) {
            case "Left":
                drive.followTrajectorySequence(traj1a);
                drive.clawLeft.setPosition(leftOpenPos);
                drive.followTrajectory(traj1aa);
                drive.clawLeft.setPosition(leftClosePos);
                sleep(96);
                drive.wrist.setPosition(0.34);
                drive.slide.setPower(0.9);
                drive.slide.setTargetPosition(1300);
                drive.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                drive.followTrajectorySequence(traj2a); // drive to backboard 1
                drive.clawRight.setPosition(rightOpenPos);
                drive.slide.setTargetPosition(0);
                drive.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                sleep(300);
                drive.wrist.setPosition(0.169);
                drive.followTrajectorySequence(traj2a2);
                drive.clawRight.setPosition(rightClosePos);
                /*drive.followTrajectorySequence(traj3a); // drive to stack 1
                drive.clawRight.setPosition(rightOpenPos);
                drive.clawLeft.setPosition(leftOpenPos);
                drive.followTrajectorySequence(traj3a2); // drive to stack 1.2
                drive.slide.setTargetPosition(235);
                drive.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                drive.followTrajectorySequence(traj3a22);
                drive.clawLeft.setPosition(leftClosePos);
                drive.clawRight.setPosition(rightClosePos);
                sleep(100);
                drive.slide.setTargetPosition(245);
                drive.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                drive.wrist.setPosition(0.34);
                drive.followTrajectory(traj3a3);
                drive.slide.setTargetPosition(0);
                drive.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                drive.followTrajectorySequence(traj4a);
                drive.slide.setTargetPosition(1690);
                drive.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                drive.followTrajectorySequence(traj5a); // drive to backboard 2
                sleep(100);
                drive.clawRight.setPosition(rightOpenPos);
                drive.clawLeft.setPosition(leftOpenPos);
                sleep(200);
                drive.followTrajectorySequence(traj7a);
                drive.clawLeft.setPosition(leftClosePos);
                drive.clawRight.setPosition(rightClosePos);
                drive.slide.setTargetPosition(0);
                drive.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                drive.wrist.setPosition(0.169);

                drive.followTrajectorySequence(traj3aa);
                drive.clawRight.setPosition(rightOpenPos);
                drive.clawLeft.setPosition(leftOpenPos);
                drive.followTrajectorySequence(traj3aa2);
                drive.slide.setTargetPosition(90);
                drive.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                drive.followTrajectorySequence(traj3aa22);
                drive.clawLeft.setPosition(leftClosePos); // drive to stack 2
                drive.clawRight.setPosition(rightClosePos);
                sleep(120);
                drive.slide.setTargetPosition(100);
                drive.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                drive.wrist.setPosition(0.34);
                drive.followTrajectory(traj3aa3);
                drive.slide.setTargetPosition(0);
                drive.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                drive.followTrajectorySequence(traj4aa);
                drive.slide.setTargetPosition(1700);
                drive.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                drive.followTrajectorySequence(traj5aa);
                sleep(100);
                drive.clawRight.setPosition(rightOpenPos);
                drive.clawLeft.setPosition(leftOpenPos);
                drive.followTrajectorySequence(traj6aa);
                drive.slide.setTargetPosition(0);
                drive.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                sleep(300);
                drive.followTrajectorySequence(traj6aaa);
                drive.clawLeft.setPosition(leftClosePos);
                drive.clawRight.setPosition(rightClosePos);
                drive.wrist.setPosition(0.169);
                drive.followTrajectorySequence(traj7aa);*/

                break;
            case "Center":
                drive.followTrajectory(traj1b);
                drive.clawLeft.setPosition(leftOpenPos);
                drive.followTrajectory(traj1bb);
                drive.clawLeft.setPosition(leftClosePos);
                sleep(96);
                drive.wrist.setPosition(0.34);
                drive.slide.setPower(0.9);
                drive.slide.setTargetPosition(1300);
                drive.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                drive.followTrajectorySequence(traj2b); // drive to backboard 1
                drive.clawRight.setPosition(rightOpenPos);
                drive.slide.setTargetPosition(0);
                drive.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                sleep(300);
                drive.wrist.setPosition(0.169);
                drive.followTrajectorySequence(traj2b2);
                drive.clawRight.setPosition(rightClosePos);
                /*drive.followTrajectorySequence(traj3b); // drive to stack 1
                drive.clawRight.setPosition(rightOpenPos);
                drive.clawLeft.setPosition(leftOpenPos);
                drive.followTrajectorySequence(traj3b2); // drive to stack 1.2
                drive.slide.setTargetPosition(250);
                drive.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                drive.followTrajectorySequence(traj3b22);
                drive.clawLeft.setPosition(leftClosePos);
                drive.clawRight.setPosition(rightClosePos);
                sleep(100);
                drive.slide.setTargetPosition(260);
                drive.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                drive.wrist.setPosition(0.34);
                drive.followTrajectory(traj3b3);
                drive.slide.setTargetPosition(0);
                drive.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                drive.followTrajectorySequence(traj4b);
                drive.slide.setTargetPosition(1690);
                drive.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                drive.followTrajectorySequence(traj5b); // drive to backboard 2
                sleep(100);
                drive.clawRight.setPosition(rightOpenPos);
                drive.clawLeft.setPosition(leftOpenPos);
                sleep(200);
                drive.followTrajectorySequence(traj7b);
                drive.clawLeft.setPosition(leftClosePos);
                drive.clawRight.setPosition(rightClosePos);
                drive.slide.setTargetPosition(0);
                drive.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                drive.wrist.setPosition(0.169);

                drive.followTrajectorySequence(traj3bb);
                drive.clawRight.setPosition(rightOpenPos);
                drive.clawLeft.setPosition(leftOpenPos);
                drive.followTrajectorySequence(traj3bb2);
                drive.slide.setTargetPosition(90);
                drive.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                drive.followTrajectorySequence(traj3bb22);
                drive.clawLeft.setPosition(leftClosePos); // drive to stack 2
                drive.clawRight.setPosition(rightClosePos);
                sleep(120);
                drive.slide.setTargetPosition(100);
                drive.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                drive.wrist.setPosition(0.34);
                drive.followTrajectory(traj3bb3);
                drive.slide.setTargetPosition(0);
                drive.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                drive.followTrajectorySequence(traj4bb);
                drive.slide.setTargetPosition(1700);
                drive.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                drive.followTrajectorySequence(traj5bb);
                sleep(100);
                drive.clawRight.setPosition(rightOpenPos);
                drive.clawLeft.setPosition(leftOpenPos);
                drive.followTrajectorySequence(traj6bb);
                drive.slide.setTargetPosition(0);
                drive.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                sleep(300);
                drive.followTrajectorySequence(traj6bbb);
                drive.clawLeft.setPosition(leftClosePos);
                drive.clawRight.setPosition(rightClosePos);
                drive.wrist.setPosition(0.169);
                drive.followTrajectorySequence(traj7bb);*/

                break;
            default:
                drive.followTrajectory(traj1c);
                drive.clawLeft.setPosition(leftOpenPos);
                drive.followTrajectory(traj1cc);
                drive.clawLeft.setPosition(leftClosePos);
                sleep(96);
                drive.wrist.setPosition(0.34);
                drive.slide.setPower(0.9);
                drive.slide.setTargetPosition(1300);
                drive.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                drive.followTrajectorySequence(traj2c); // drive to backboard 1
                drive.clawRight.setPosition(rightOpenPos);
                drive.slide.setTargetPosition(0);
                drive.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                sleep(300);
                drive.wrist.setPosition(0.169);
                drive.followTrajectorySequence(traj2c2);
                drive.clawRight.setPosition(rightClosePos);
                /*drive.followTrajectorySequence(traj3c); // drive to stack 1
                drive.clawRight.setPosition(rightOpenPos);
                drive.clawLeft.setPosition(leftOpenPos);
                drive.followTrajectorySequence(traj3c2); // drive to stack 1.2
                drive.slide.setTargetPosition(235);
                drive.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                drive.followTrajectorySequence(traj3c22);
                drive.clawLeft.setPosition(leftClosePos);
                drive.clawRight.setPosition(rightClosePos);
                sleep(100);
                drive.slide.setTargetPosition(245);
                drive.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                drive.wrist.setPosition(0.34);
                drive.followTrajectory(traj3c3);
                drive.slide.setTargetPosition(0);
                drive.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                drive.followTrajectorySequence(traj4c);
                drive.slide.setTargetPosition(1690);
                drive.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                drive.followTrajectorySequence(traj5c); // drive to backboard 2
                sleep(100);
                drive.clawRight.setPosition(rightOpenPos);
                drive.clawLeft.setPosition(leftOpenPos);
                sleep(200);
                drive.followTrajectorySequence(traj7c);
                drive.clawLeft.setPosition(leftClosePos);
                drive.clawRight.setPosition(rightClosePos);
                drive.slide.setTargetPosition(0);
                drive.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                drive.wrist.setPosition(0.169);

                drive.followTrajectorySequence(traj3cc);
                drive.clawRight.setPosition(rightOpenPos);
                drive.clawLeft.setPosition(leftOpenPos);
                drive.followTrajectorySequence(traj3cc2);
                drive.slide.setTargetPosition(90);
                drive.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                drive.followTrajectorySequence(traj3cc22);
                drive.clawLeft.setPosition(leftClosePos); // drive to stack 2
                drive.clawRight.setPosition(rightClosePos);
                sleep(120);
                drive.slide.setTargetPosition(100);
                drive.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                drive.wrist.setPosition(0.34);
                drive.followTrajectory(traj3cc3);
                drive.slide.setTargetPosition(0);
                drive.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                drive.followTrajectorySequence(traj4cc);
                drive.slide.setTargetPosition(1700);
                drive.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                drive.followTrajectorySequence(traj5cc);
                sleep(100);
                drive.clawRight.setPosition(rightOpenPos);
                drive.clawLeft.setPosition(leftOpenPos);
                drive.followTrajectorySequence(traj6cc);
                drive.slide.setTargetPosition(0);
                drive.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                sleep(300);
                drive.followTrajectorySequence(traj6ccc);
                drive.clawLeft.setPosition(leftClosePos);
                drive.clawRight.setPosition(rightClosePos);
                drive.wrist.setPosition(0.169);
                drive.followTrajectorySequence(traj7cc);*/
        }
    }
}