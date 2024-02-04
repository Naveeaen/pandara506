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
public class automusRedBackJava extends LinearOpMode {
    OpenCvCamera webCam;
    public PipelineRedBack detector;
    public  String position = "Insert Here";
    public int timeout = 0;
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

        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));
        drive.setPoseEstimate(startPose);

        //cameraqa
        int cameraMotionViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        detector = new PipelineRedBack();
        webCam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "wc1"), cameraMotionViewId);
        webCam.openCameraDevice();
        FtcDashboard.getInstance().startCameraStream(webCam, 0);
        webCam.startStreaming(640,480, OpenCvCameraRotation.UPRIGHT);
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
                .lineToLinearHeading(new Pose2d(29, 2.5, Math.toRadians(23)))
                .build();
        TrajectorySequence traj1aa = drive.trajectorySequenceBuilder(traj1a.end())
                .lineToLinearHeading(new Pose2d(25, -5, Math.toRadians(90)))
                .lineTo(new Vector2d(20, 17.5))
                .lineTo(new Vector2d(29, 17.5))
                .setVelConstraint(Hardware.getVelocityConstraint(0.1 * DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .forward(4.6)
                .build();
        TrajectorySequence traj2a = drive.trajectorySequenceBuilder(traj1aa.end())
                .lineToLinearHeading(new Pose2d(2.5, 0, Math.toRadians(90)))
                .build();
        Trajectory traj2aa = drive.trajectoryBuilder(traj2a.end())
                .lineTo(new Vector2d(4, -72))
                .build();
        TrajectorySequence traj3a = drive.trajectorySequenceBuilder(traj2aa.end())
                .lineToLinearHeading(new Pose2d(37.5, -87.5, Math.toRadians(-90)))
                .setVelConstraint(Hardware.getVelocityConstraint(0.1 * DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .forward(6.5)
                .build();
        TrajectorySequence traj3aa = drive.trajectorySequenceBuilder(traj3a.end())
                .back(3.4)
                .setVelConstraint(Hardware.getVelocityConstraint(0.2 * DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .strafeRight(7)
                .setVelConstraint(Hardware.getVelocityConstraint(0.1 * DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .forward(5.2)
                .build();
        Trajectory traj3aaab = drive.trajectoryBuilder(traj3aa.end())
                .back(5)
                .build();
        Trajectory traj3aaa = drive.trajectoryBuilder(traj3aaab.end())
                .lineToLinearHeading(new Pose2d(3.5, -72, Math.toRadians(90)))
                .build();
        TrajectorySequence traj4a = drive.trajectorySequenceBuilder(traj3aaa.end())
                .lineTo(new Vector2d(3.5, 17))
                .strafeRight(19.2)
                .setVelConstraint(Hardware.getVelocityConstraint(0.1 * DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .forward(5.7)
                .build();
        TrajectorySequence traj7a = drive.trajectorySequenceBuilder(traj4a.end())
                .back(5)
                .lineToLinearHeading(new Pose2d(3, 0, Math.toRadians(90)))
                .lineTo(new Vector2d(3, -72))
                .build();
        TrajectorySequence traj8a = drive.trajectorySequenceBuilder(traj7a.end())
                .lineToLinearHeading(new Pose2d(29, -87, Math.toRadians(-90)))
                .setVelConstraint(Hardware.getVelocityConstraint(0.1 * DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .forward(7)
                .build();
        TrajectorySequence traj8aa = drive.trajectorySequenceBuilder(traj8a.end())
                .setVelConstraint(Hardware.getVelocityConstraint(0.15 * DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .lineTo(new Vector2d(23, -92))
                .build();
        TrajectorySequence traj9a = drive.trajectorySequenceBuilder(traj8aa.end())
                .back(10)
                .strafeRight(20)
                .build();

        //center
        Trajectory traj1b = drive.trajectoryBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(31, -4, Math.toRadians(23.789)))
                .build();
        TrajectorySequence traj1bb = drive.trajectorySequenceBuilder(traj1b.end())
                .back(10)
                .lineToLinearHeading(new Pose2d(29, 18.5, Math.toRadians(90)))
                .setVelConstraint(Hardware.getVelocityConstraint(0.1 * DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .forward(4.4)
                .build();
        TrajectorySequence traj2b = drive.trajectorySequenceBuilder(traj1bb.end())
                .back(5)
                .lineTo(new Vector2d(2.5, 0))
                .build();
        TrajectorySequence traj2bb = drive.trajectorySequenceBuilder(traj2b.end())
                .lineTo(new Vector2d(2.5, -72))
                .build();
        TrajectorySequence traj3b = drive.trajectorySequenceBuilder(traj2bb.end())
                .lineToLinearHeading(new Pose2d(30, -89, Math.toRadians(-90)))
                .setVelConstraint(Hardware.getVelocityConstraint(0.1 * DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .forward(3)
                .build();
        Trajectory traj3bb = drive.trajectoryBuilder(traj3b.end())
                .back(5)
                .build();
        Trajectory traj3bbb = drive.trajectoryBuilder(traj3bb.end())
                .lineToLinearHeading(new Pose2d(2.5, -72, Math.toRadians(90)))
                .build();
        TrajectorySequence traj4b = drive.trajectorySequenceBuilder(traj3bbb.end())
                .lineTo(new Vector2d(2.5, 17))
                .strafeRight(19.2)
                .setVelConstraint(Hardware.getVelocityConstraint(0.1 * DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .forward(5.7)
                .build();
        TrajectorySequence traj7b = drive.trajectorySequenceBuilder(traj4b.end())
                .back(5)
                .lineToLinearHeading(new Pose2d(1.8, 0, Math.toRadians(90)))
                .lineTo(new Vector2d(2.5, -72))
                .build();
        TrajectorySequence traj8b = drive.trajectorySequenceBuilder(traj7b.end())
                .lineToLinearHeading(new Pose2d(30, -87, Math.toRadians(-90)))
                .setVelConstraint(Hardware.getVelocityConstraint(0.1 * DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .forward(6.9)
                .build();
        TrajectorySequence traj8bb = drive.trajectorySequenceBuilder(traj8b.end())
                .setVelConstraint(Hardware.getVelocityConstraint(0.1 * DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .lineTo(new Vector2d(24, -92))
                .build();
        TrajectorySequence traj9b = drive.trajectorySequenceBuilder(traj8bb.end())
                .back(10)
                .strafeRight(20)
                .build();

        //right
        Trajectory traj1c = drive.trajectoryBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(28, -13.5, Math.toRadians(-40)))
                .build();
        TrajectorySequence traj1cc = drive.trajectorySequenceBuilder(traj1c.end())
                .back(5)
                .lineToLinearHeading(new Pose2d(29, 18.5, Math.toRadians(90)))
                .setVelConstraint(Hardware.getVelocityConstraint(0.1 * DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .forward(4.4)
                .build();
        TrajectorySequence traj2c = drive.trajectorySequenceBuilder(traj1cc.end())
                .back(5)
                .lineTo(new Vector2d(2.5, 0))
                .build();
        TrajectorySequence traj2cc = drive.trajectorySequenceBuilder(traj2c.end())
                .lineTo(new Vector2d(2.5, -72))
                .build();
        TrajectorySequence traj3c = drive.trajectorySequenceBuilder(traj2cc.end())
                .lineToLinearHeading(new Pose2d(27, -89, Math.toRadians(-90)))
                .setVelConstraint(Hardware.getVelocityConstraint(0.1 * DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .forward(5)
                .build();
        Trajectory traj3cc = drive.trajectoryBuilder(traj3c.end())
                .back(5)
                .build();
        Trajectory traj3ccc = drive.trajectoryBuilder(traj3cc.end())
                .lineToLinearHeading(new Pose2d(2.5, -72, Math.toRadians(90)))
                .build();
        TrajectorySequence traj4c = drive.trajectorySequenceBuilder(traj3ccc.end())
                .lineTo(new Vector2d(2.5, 18.5))
                .strafeRight(17.1)
                .setVelConstraint(Hardware.getVelocityConstraint(0.1 * DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .forward(5.5)
                .build();
        TrajectorySequence traj7c = drive.trajectorySequenceBuilder(traj4c.end())
                .back(5)
                .lineToLinearHeading(new Pose2d(1.8, 0, Math.toRadians(90)))
                .lineTo(new Vector2d(2.5, -72))
                .build();
        TrajectorySequence traj8c = drive.trajectorySequenceBuilder(traj7c.end())
                .lineToLinearHeading(new Pose2d(33, -87, Math.toRadians(-90)))
                .setVelConstraint(Hardware.getVelocityConstraint(0.1 * DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .forward(6.9)
                .build();
        TrajectorySequence traj8cc = drive.trajectorySequenceBuilder(traj8c.end())
                .setVelConstraint(Hardware.getVelocityConstraint(0.1 * DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .lineTo(new Vector2d(26, -87))
                .build();
        TrajectorySequence traj9c = drive.trajectorySequenceBuilder(traj8cc.end())
                .back(5)
                .strafeRight(20)
                .build();

        waitForStart();
        sleep(timeout);
        switch (position) {
            case "Left":
                drive.followTrajectory(traj1a);
                drive.wrist.setPosition(0.169);
                drive.clawLeft.setPosition(leftOpenPos); // purple pixel drop
                drive.slide.setPower(0.7); // extra pixel slides
                drive.slide.setTargetPosition(340);
                drive.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                drive.followTrajectorySequence(traj1aa);
                drive.clawLeft.setPosition(leftClosePos);
                sleep(300);
                drive.followTrajectorySequence(traj2a);
                drive.slide.setTargetPosition(10);
                drive.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                drive.followTrajectory(traj2aa);
                drive.slide.setPower(0.7); // raise slides 1
                drive.slide.setTargetPosition(1300);
                drive.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                drive.wrist.setPosition(0.34);
                drive.followTrajectorySequence(traj3a);
                drive.clawRight.setPosition(rightOpenPos); // drop yellow pixel
                sleep(150);
                drive.followTrajectorySequence(traj3aa);
                drive.clawLeft.setPosition(leftOpenPos);
                drive.slide.setPower(0.7); //reset slides 1
                drive.slide.setTargetPosition(180);
                drive.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                drive.wrist.setPosition(0.169);
                drive.followTrajectory(traj3aaab);
                sleep(150);
                drive.clawRight.setPosition(rightClosePos);
                drive.clawLeft.setPosition(leftClosePos);

                drive.followTrajectory(traj3aaa);
                drive.clawRight.setPosition(rightOpenPos);
                drive.clawLeft.setPosition(leftOpenPos);
                drive.followTrajectorySequence(traj4a);
                drive.clawRight.setPosition(rightClosePos);
                drive.clawLeft.setPosition(leftClosePos);

                sleep(300);
                drive.followTrajectorySequence(traj7a); // cycle to backboard
                drive.wrist.setPosition(0.34);
                drive.slide.setPower(0.7); // raise slides 2
                drive.slide.setTargetPosition(1500);
                drive.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                drive.followTrajectorySequence(traj8a);
                drive.clawRight.setPosition(rightOpenPos); // drop cycle pixels 1
                drive.clawLeft.setPosition(leftOpenPos);
                drive.followTrajectorySequence(traj8aa);
                drive.clawRight.setPosition(rightClosePos);
                drive.clawLeft.setPosition(leftClosePos);
                drive.slide.setPower(0.7); //reset slides 2
                drive.slide.setTargetPosition(0);
                drive.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                drive.wrist.setPosition(0.169);
                drive.followTrajectorySequence(traj9a);
                break;
            case "Center":
                drive.followTrajectory(traj1b);
                drive.wrist.setPosition(0.169);
                drive.clawLeft.setPosition(leftOpenPos); // purple pixel drop
                drive.slide.setPower(0.7); // extra pixel slides
                drive.slide.setTargetPosition(330);
                drive.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                drive.followTrajectorySequence(traj1bb);
                drive.clawLeft.setPosition(leftClosePos);
                sleep(200);
                drive.followTrajectorySequence(traj2b);
                drive.slide.setTargetPosition(10);
                drive.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                drive.followTrajectorySequence(traj2bb);
                drive.slide.setPower(0.7); // raise slides 1
                drive.slide.setTargetPosition(1300);
                drive.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                drive.wrist.setPosition(0.34);
                drive.followTrajectorySequence(traj3b);
                drive.clawRight.setPosition(rightOpenPos); // drop yellow pixel
                drive.clawLeft.setPosition(leftOpenPos);
                sleep(300);
                drive.slide.setPower(0.7); //reset slides 1
                drive.slide.setTargetPosition(190);
                drive.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                sleep(300);
                drive.wrist.setPosition(0.169);
                drive.followTrajectory(traj3bb);
                drive.clawRight.setPosition(rightClosePos);
                drive.clawLeft.setPosition(leftClosePos);
                drive.followTrajectory(traj3bbb);
                drive.clawRight.setPosition(rightOpenPos);
                drive.clawLeft.setPosition(leftOpenPos);
                drive.followTrajectorySequence(traj4b);
                drive.clawRight.setPosition(rightClosePos);
                drive.clawLeft.setPosition(leftClosePos);

                sleep(300);
                drive.followTrajectorySequence(traj7b); // cycle to backboard
                drive.wrist.setPosition(0.34);
                drive.slide.setPower(0.7); // raise slides 2
                drive.slide.setTargetPosition(1500);
                drive.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                drive.followTrajectorySequence(traj8b);
                drive.clawRight.setPosition(rightOpenPos); // drop cycle pixels 1
                drive.clawLeft.setPosition(leftOpenPos);
                drive.followTrajectorySequence(traj8bb);
                drive.clawRight.setPosition(rightClosePos);
                drive.clawLeft.setPosition(leftClosePos);
                drive.slide.setPower(0.7); //reset slides 2
                drive.slide.setTargetPosition(0);
                drive.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                drive.wrist.setPosition(0.169);
                drive.followTrajectorySequence(traj9b);
                break;
            case "Right":
                drive.followTrajectory(traj1c);
                drive.wrist.setPosition(0.169);
                drive.clawLeft.setPosition(leftOpenPos); // purple pixel drop
                drive.slide.setPower(0.7); // extra pixel slides
                drive.slide.setTargetPosition(340);
                drive.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                drive.followTrajectorySequence(traj1cc);
                drive.clawLeft.setPosition(leftClosePos);
                sleep(200);
                drive.followTrajectorySequence(traj2c);
                drive.slide.setTargetPosition(10);
                drive.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                drive.followTrajectorySequence(traj2cc);
                drive.slide.setPower(0.7); // raise slides 1
                drive.slide.setTargetPosition(1300);
                drive.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                drive.wrist.setPosition(0.34);
                drive.followTrajectorySequence(traj3c);
                drive.clawRight.setPosition(rightOpenPos); // drop yellow pixel
                drive.clawLeft.setPosition(leftOpenPos);
                sleep(300);
                drive.followTrajectory(traj3cc);
                drive.clawRight.setPosition(rightClosePos);
                drive.clawLeft.setPosition(leftClosePos);
                drive.slide.setPower(0.7); //reset slides 1
                drive.slide.setTargetPosition(190);
                drive.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                drive.wrist.setPosition(0.169);
                drive.followTrajectory(traj3ccc);
                drive.clawRight.setPosition(rightOpenPos);
                drive.clawLeft.setPosition(leftOpenPos);
                drive.followTrajectorySequence(traj4b);
                drive.clawRight.setPosition(rightClosePos);
                drive.clawLeft.setPosition(leftClosePos);

                sleep(300);
                drive.followTrajectorySequence(traj7c); // cycle to backboard
                drive.wrist.setPosition(0.34);
                drive.slide.setPower(0.7); // raise slides 2
                drive.slide.setTargetPosition(1500);
                drive.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                drive.followTrajectorySequence(traj8c);
                drive.clawRight.setPosition(rightOpenPos); // drop cycle pixels 1
                drive.clawLeft.setPosition(leftOpenPos);
                drive.followTrajectorySequence(traj8cc);
                drive.clawRight.setPosition(rightClosePos);
                drive.clawLeft.setPosition(leftClosePos);
                drive.slide.setPower(0.7); //reset slides 2
                drive.slide.setTargetPosition(0);
                drive.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                drive.wrist.setPosition(0.169);
                drive.followTrajectorySequence(traj9c);

        }
    }
}