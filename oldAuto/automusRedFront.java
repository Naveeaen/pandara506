package org.firstinspires.ftc.teamcode.pandara506.oldAuto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pandara506.roadrunner.DriveConstants;
import org.firstinspires.ftc.teamcode.pandara506.mainPrograms.Hardware;
import org.firstinspires.ftc.teamcode.pandara506.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;

@Config
@Disabled
@Autonomous(group = "autoRed")
public class automusRedFront extends LinearOpMode {

    OpenCvCamera webCam;
    //public PipelineRedFront detector;
    public String position = "Left";
    int timeout = 0;
    String cycle = "yes";
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
        /*int cameraMotionViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
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
        }*/

        //left
        TrajectorySequence traj1a = drive.trajectorySequenceBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(40, 5.4, Math.toRadians(90)))
                .build();
        Trajectory traj1aa = drive.trajectoryBuilder(traj1a.end())
                .back(2)
                .build();
        TrajectorySequence traj2a = drive.trajectorySequenceBuilder(traj1aa.end())
                .lineToLinearHeading(new Pose2d(34,-35.5, Math.toRadians(-90)))
                .setVelConstraint(Hardware.getVelocityConstraint(0.07 * DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .forward(3.6)
                .build();
        TrajectorySequence traj2a2 = drive.trajectorySequenceBuilder(traj2a.end())
                .back(5)
                .build();
        if(cycle.equals("yes")) {
            traj2a2 = drive.trajectorySequenceBuilder(traj2a.end())
                    .back(5)
                    .build();
        } else if(park.equals("left")){
            traj2a2 = drive.trajectorySequenceBuilder(traj2a.end())
                    .back(5)
                    .strafeLeft(20)
                    .forward(5)
                    .build();
        } else if(park.equals("right")){
            traj2a2 = drive.trajectorySequenceBuilder(traj2a.end())
                    .back(5)
                    .strafeRight(20)
                    .forward(5)
                    .build();
        }
        TrajectorySequence traj3a = drive.trajectorySequenceBuilder(traj2a2.end()) //stack 1
                .lineToLinearHeading(new Pose2d(48.3,-5, Math.toRadians(90)))
                .build();
        TrajectorySequence traj3a2 = drive.trajectorySequenceBuilder(traj3a.end())
                .lineToLinearHeading(new Pose2d(49.9, 74.8, Math.toRadians(90)))
                .build();
        TrajectorySequence traj3a22 = drive.trajectorySequenceBuilder(traj3a2.end())
                .setVelConstraint(Hardware.getVelocityConstraint(0.15 * DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .forward(4.5)
                .build();
        Trajectory traj3a3 = drive.trajectoryBuilder(traj3a22.end())
                .back(6.5)
                .build();
        TrajectorySequence traj4a = drive.trajectorySequenceBuilder(traj3a3.end())
                .lineToLinearHeading(new Pose2d(48.3, 0, Math.toRadians(90)))
                .build();
        TrajectorySequence traj5a = drive.trajectorySequenceBuilder(traj4a.end())
                .lineToLinearHeading(new Pose2d(26,-35.5, Math.toRadians(-90)))
                .setVelConstraint(Hardware.getVelocityConstraint(0.15 * DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .forward(6)
                .build();
        TrajectorySequence traj7a = drive.trajectorySequenceBuilder(traj5a.end())
                .setVelConstraint(Hardware.getVelocityConstraint(0.15 * DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .lineTo(new Vector2d(18, -35)) // drop 2
                .build();

        TrajectorySequence traj3aa = drive.trajectorySequenceBuilder(traj7a.end())
                .lineToLinearHeading(new Pose2d(45.4,-5, Math.toRadians(90)))
                .build();
        TrajectorySequence traj3aa2 = drive.trajectorySequenceBuilder(traj3aa.end())
                .lineToLinearHeading(new Pose2d(47.3, 77.4, Math.toRadians(90)))
                .build();
        TrajectorySequence traj3aa22 = drive.trajectorySequenceBuilder(traj3aa2.end())
                .setVelConstraint(Hardware.getVelocityConstraint(0.15 * DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .forward(2.5)
                .build();
        Trajectory traj3aa3 = drive.trajectoryBuilder(traj3aa22.end())
                .back(6)
                .build();
        TrajectorySequence traj4aa = drive.trajectorySequenceBuilder(traj3aa3.end())
                .lineToLinearHeading(new Pose2d(45.4, 0, Math.toRadians(90)))
                .build();
        TrajectorySequence traj5aa = drive.trajectorySequenceBuilder(traj4aa.end())
                .lineToLinearHeading(new Pose2d(22,-34.5, Math.toRadians(-88)))
                .setVelConstraint(Hardware.getVelocityConstraint(0.1 * DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .forward(6)
                .build();
        TrajectorySequence traj6aa = drive.trajectorySequenceBuilder(traj5aa.end())
                .setVelConstraint(Hardware.getVelocityConstraint(0.15 * DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .lineTo(new Vector2d(15, -35.5)) // drop 3.2
                .build();
        TrajectorySequence traj6aaa = drive.trajectorySequenceBuilder(traj6aa.end())
                .back(3)
                .build();
        TrajectorySequence traj7aa = drive.trajectorySequenceBuilder(traj6aaa.end())
                .back(3)
                .strafeLeft(20)
                .build();
        //center
        Trajectory traj1b = drive.trajectoryBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(31, -3, Math.toRadians(30)))
                .build();
        Trajectory traj1bb = drive.trajectoryBuilder(traj1b.end())
                .back(2)
                .build();
        TrajectorySequence traj2b = drive.trajectorySequenceBuilder(traj1bb.end())
                .lineToLinearHeading(new Pose2d(28,-35.5, Math.toRadians(-90)))
                .setVelConstraint(Hardware.getVelocityConstraint(0.07 * DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .forward(3.6)
                .build();
        TrajectorySequence traj2b2 = drive.trajectorySequenceBuilder(traj2b.end())
                .back(5)
                .build();
        if(cycle.equals("yes")) {
            traj2b2 = drive.trajectorySequenceBuilder(traj2b.end())
                    .back(5)
                    .build();
        } else if(park.equals("left")){
            traj2b2 = drive.trajectorySequenceBuilder(traj2b.end())
                    .back(5)
                    .strafeLeft(25)
                    .forward(5)
                    .build();
        } else if(park.equals("right")){
            traj2b2 = drive.trajectorySequenceBuilder(traj2b.end())
                    .back(5)
                    .strafeRight(25)
                    .forward(5)
                    .build();
        }
        TrajectorySequence traj3b = drive.trajectorySequenceBuilder(traj2b2.end()) //stack 1
                .lineToLinearHeading(new Pose2d(48.3,-5, Math.toRadians(90)))
                .build();
        TrajectorySequence traj3b2 = drive.trajectorySequenceBuilder(traj3b.end())
                .lineToLinearHeading(new Pose2d(49.9, 74.8, Math.toRadians(90)))
                .setVelConstraint(Hardware.getVelocityConstraint(0.15 * DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .forward(4.5)
                .build();
        TrajectorySequence traj4b = drive.trajectorySequenceBuilder(traj3b2.end())
                .back(6.5)
                .lineToLinearHeading(new Pose2d(48.3, 0, Math.toRadians(90)))
                .build();
        TrajectorySequence traj5b = drive.trajectorySequenceBuilder(traj4b.end())
                .lineToLinearHeading(new Pose2d(26,-35.5, Math.toRadians(-90)))
                .setVelConstraint(Hardware.getVelocityConstraint(0.15 * DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .forward(6)
                .build();
        TrajectorySequence traj7b = drive.trajectorySequenceBuilder(traj5b.end())
                .setVelConstraint(Hardware.getVelocityConstraint(0.15 * DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .lineTo(new Vector2d(23, -35)) // drop 2
                .build();

        TrajectorySequence traj3bb = drive.trajectorySequenceBuilder(traj7b.end())
                .lineToLinearHeading(new Pose2d(45.4,-5, Math.toRadians(90)))
                .build();
        TrajectorySequence traj3bb2 = drive.trajectorySequenceBuilder(traj3bb.end())
                .lineToLinearHeading(new Pose2d(47.3, 77.4, Math.toRadians(90)))
                .setVelConstraint(Hardware.getVelocityConstraint(0.15 * DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .forward(2.5)
                .build();
        TrajectorySequence traj4bb = drive.trajectorySequenceBuilder(traj3bb2.end())
                .back(6)
                .lineToLinearHeading(new Pose2d(45.4, 0, Math.toRadians(90)))
                .build();
        TrajectorySequence traj5bb = drive.trajectorySequenceBuilder(traj4bb.end())
                .lineToLinearHeading(new Pose2d(22,-34.5, Math.toRadians(-88)))
                .setVelConstraint(Hardware.getVelocityConstraint(0.1 * DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .forward(6)
                .build();
        TrajectorySequence traj6bb = drive.trajectorySequenceBuilder(traj5bb.end())
                .setVelConstraint(Hardware.getVelocityConstraint(0.15 * DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .lineTo(new Vector2d(19, -35.5)) // drop 3.2
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
                .lineToLinearHeading(new Pose2d(22, -13, Math.toRadians(30)))
                .build();
        Trajectory traj1cc = drive.trajectoryBuilder(traj1c.end())
                .back(2)
                .build();
        TrajectorySequence traj2c = drive.trajectorySequenceBuilder(traj1cc.end())
                .lineToLinearHeading(new Pose2d(20,-35.5, Math.toRadians(-90)))
                .setVelConstraint(Hardware.getVelocityConstraint(0.07 * DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .forward(3.6)
                .build();
        TrajectorySequence traj2c2 = drive.trajectorySequenceBuilder(traj2c.end())
                .back(5)
                .build();
        if(cycle.equals("yes")) {
            traj2c2 = drive.trajectorySequenceBuilder(traj2c.end())
                    .back(5)
                    .build();
        } else if(park.equals("left")){
            traj2c2 = drive.trajectorySequenceBuilder(traj2c.end())
                    .back(5)
                    .strafeLeft(30)
                    .forward(5)
                    .build();
        } else if(park.equals("right")){
            traj2c2 = drive.trajectorySequenceBuilder(traj2c.end())
                    .back(5)
                    .strafeRight(30)
                    .forward(5)
                    .build();
        }
        TrajectorySequence traj3c = drive.trajectorySequenceBuilder(traj2c2.end()) //stack 1
                .lineToLinearHeading(new Pose2d(48.3,-5, Math.toRadians(90)))
                .build();
        TrajectorySequence traj3c2 = drive.trajectorySequenceBuilder(traj3c.end())
                .lineToLinearHeading(new Pose2d(49.9, 74.8, Math.toRadians(90)))
                .setVelConstraint(Hardware.getVelocityConstraint(0.15 * DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .forward(4.5)
                .build();
        TrajectorySequence traj4c = drive.trajectorySequenceBuilder(traj3c2.end())
                .back(6.5)
                .lineToLinearHeading(new Pose2d(48.3, 0, Math.toRadians(90)))
                .build();
        TrajectorySequence traj5c = drive.trajectorySequenceBuilder(traj4c.end())
                .lineToLinearHeading(new Pose2d(26,-35.5, Math.toRadians(-90)))
                .setVelConstraint(Hardware.getVelocityConstraint(0.15 * DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .forward(6)
                .build();
        TrajectorySequence traj7c = drive.trajectorySequenceBuilder(traj5c.end())
                .setVelConstraint(Hardware.getVelocityConstraint(0.15 * DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .lineTo(new Vector2d(18, -35)) // drop 2
                .build();

        TrajectorySequence traj3cc = drive.trajectorySequenceBuilder(traj7c.end())
                .lineToLinearHeading(new Pose2d(45.4,-5, Math.toRadians(90)))
                .build();
        TrajectorySequence traj3cc2 = drive.trajectorySequenceBuilder(traj3cc.end())
                .lineToLinearHeading(new Pose2d(47.3, 77.4, Math.toRadians(90)))
                .setVelConstraint(Hardware.getVelocityConstraint(0.15 * DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .forward(2.5)
                .build();
        TrajectorySequence traj4cc = drive.trajectorySequenceBuilder(traj3cc2.end())
                .back(6)
                .lineToLinearHeading(new Pose2d(45.4, 0, Math.toRadians(90)))
                .build();
        TrajectorySequence traj5cc = drive.trajectorySequenceBuilder(traj4cc.end())
                .lineToLinearHeading(new Pose2d(22,-34.5, Math.toRadians(-88)))
                .setVelConstraint(Hardware.getVelocityConstraint(0.1 * DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .forward(6)
                .build();
        TrajectorySequence traj6cc = drive.trajectorySequenceBuilder(traj5cc.end())
                .setVelConstraint(Hardware.getVelocityConstraint(0.15 * DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .lineTo(new Vector2d(15, -35.5)) // drop 3.2
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
                /*drive.slide.setPower(0.9);
                drive.slide.setTargetPosition(1300);
                drive.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);*/
                sleep(100);
                drive.followTrajectory(traj1aa);
                drive.clawLeft.setPosition(leftClosePos);
                //drive.wrist.setPosition(0.34);
                drive.followTrajectorySequence(traj2a); // drive to backboard 1
                drive.clawRight.setPosition(rightOpenPos);
                sleep(200);
                /*drive.slide.setTargetPosition(0);
                drive.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                sleep(200);
                drive.wrist.setPosition(0.169);*/
                drive.followTrajectorySequence(traj2a2);
                drive.clawRight.setPosition(rightClosePos);
                if(cycle.equals("yes")) {
                    drive.followTrajectorySequence(traj3a); // drive to stack 1
                    drive.clawRight.setPosition(rightOpenPos);
                    drive.clawLeft.setPosition(leftOpenPos);
                    drive.followTrajectorySequence(traj3a2); // drive to stack 1.2
                    //drive.slide.setTargetPosition(232);
                    //drive.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    drive.followTrajectorySequence(traj3a22);
                    drive.clawLeft.setPosition(leftClosePos);
                    drive.clawRight.setPosition(rightClosePos);
                    sleep(200);
                    //drive.slide.setTargetPosition(242);
                    //drive.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    //drive.wrist.setPosition(0.34);
                    drive.followTrajectory(traj3a3);
                    drive.slide.setTargetPosition(0);
                    drive.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    drive.followTrajectorySequence(traj4a);
                    //drive.slide.setTargetPosition(2000);
                    //drive.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    drive.followTrajectorySequence(traj5a); // drive to backboard 2
                    sleep(100);
                    drive.clawRight.setPosition(rightOpenPos);
                    drive.clawLeft.setPosition(leftOpenPos);
                    sleep(200);
                    drive.followTrajectorySequence(traj7a);
                    drive.clawLeft.setPosition(leftClosePos);
                    drive.clawRight.setPosition(rightClosePos);
                    //drive.slide.setTargetPosition(0);
                    //drive.wrist.setPosition(0.169);

                    drive.followTrajectorySequence(traj3aa);
                    drive.clawRight.setPosition(rightOpenPos);
                    drive.clawLeft.setPosition(leftOpenPos);
                    drive.followTrajectorySequence(traj3aa2);
                    //drive.slide.setTargetPosition(90);
                    //drive.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    drive.followTrajectorySequence(traj3aa22);
                    drive.clawLeft.setPosition(leftClosePos); // drive to stack 2
                    drive.clawRight.setPosition(rightClosePos);
                    sleep(120);
                    drive.followTrajectory(traj3aa3);
                    //drive.slide.setTargetPosition(0);
                    //drive.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    //drive.wrist.setPosition(0.34);
                    drive.followTrajectorySequence(traj4aa);
                    //drive.slide.setTargetPosition(2000);
                    //drive.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    drive.followTrajectorySequence(traj5aa);
                    sleep(100);
                    drive.clawRight.setPosition(rightOpenPos);
                    drive.clawLeft.setPosition(leftOpenPos);
                    drive.followTrajectorySequence(traj6aa);
                    sleep(100);
                    drive.followTrajectorySequence(traj6aaa);
                    //drive.slide.setTargetPosition(0);
                    //drive.wrist.setPosition(0.169);
                    drive.clawLeft.setPosition(leftClosePos);
                    drive.clawRight.setPosition(rightClosePos);
                    drive.followTrajectorySequence(traj7aa);
                }

                break;
            case "Center":
                drive.followTrajectory(traj1b);
                drive.clawLeft.setPosition(leftOpenPos);
                drive.followTrajectory(traj1bb);
                drive.clawLeft.setPosition(leftClosePos);
                drive.wrist.setPosition(0.34);
                drive.slide.setPower(0.9);
                drive.slide.setTargetPosition(1300);
                drive.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                drive.followTrajectorySequence(traj2b); // drive to backboard 1
                drive.clawRight.setPosition(rightOpenPos);
                sleep(200);
                drive.slide.setTargetPosition(0);
                drive.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                sleep(200);
                drive.wrist.setPosition(0.169);
                drive.followTrajectorySequence(traj2b2);
                drive.clawRight.setPosition(rightClosePos);
                if(cycle.equals("yes")) {
                    drive.followTrajectorySequence(traj3a); // drive to stack 1
                    drive.clawRight.setPosition(rightOpenPos);
                    drive.clawLeft.setPosition(leftOpenPos);
                    drive.followTrajectorySequence(traj3a2); // drive to stack 1.2
                    drive.slide.setTargetPosition(232);
                    drive.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    drive.followTrajectorySequence(traj3a22);
                    drive.clawLeft.setPosition(leftClosePos);
                    drive.clawRight.setPosition(rightClosePos);
                    sleep(200);
                    drive.slide.setTargetPosition(242);
                    drive.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    drive.wrist.setPosition(0.34);
                    drive.followTrajectory(traj3a3);
                    drive.slide.setTargetPosition(0);
                    drive.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    drive.followTrajectorySequence(traj4a);
                    drive.slide.setTargetPosition(2000);
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
                    drive.wrist.setPosition(0.169);

                    drive.followTrajectorySequence(traj3bb);
                    drive.clawRight.setPosition(rightOpenPos);
                    drive.clawLeft.setPosition(leftOpenPos);
                    drive.followTrajectorySequence(traj3bb2);
                    drive.slide.setTargetPosition(90);
                    drive.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    //drive.followTrajectorySequence(traj3bb22);
                    drive.clawLeft.setPosition(leftClosePos); // drive to stack 2
                    drive.clawRight.setPosition(rightClosePos);
                    sleep(120);
                    //drive.followTrajectory(traj3bb3);
                    drive.slide.setTargetPosition(0);
                    drive.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    drive.wrist.setPosition(0.34);
                    drive.followTrajectorySequence(traj4bb);
                    drive.slide.setTargetPosition(2000);
                    drive.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    drive.followTrajectorySequence(traj5bb);
                    sleep(100);
                    drive.clawRight.setPosition(rightOpenPos);
                    drive.clawLeft.setPosition(leftOpenPos);
                    drive.followTrajectorySequence(traj6bb);
                    sleep(100);
                    drive.followTrajectorySequence(traj6bbb);
                    drive.slide.setTargetPosition(0);
                    drive.wrist.setPosition(0.169);
                    drive.clawLeft.setPosition(leftClosePos);
                    drive.clawRight.setPosition(rightClosePos);
                    drive.followTrajectorySequence(traj7bb);
                }

                break;
            default:
                drive.followTrajectory(traj1c);
                drive.clawLeft.setPosition(leftOpenPos);
                drive.followTrajectory(traj1cc);
                drive.clawLeft.setPosition(leftClosePos);
                drive.wrist.setPosition(0.34);
                drive.slide.setPower(0.9);
                drive.slide.setTargetPosition(1300);
                drive.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                drive.followTrajectorySequence(traj2c); // drive to backboard 1
                drive.clawRight.setPosition(rightOpenPos);
                sleep(200);
                drive.slide.setTargetPosition(232);
                drive.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                sleep(200);
                drive.wrist.setPosition(0.169);
                drive.followTrajectorySequence(traj2c2);
                drive.clawRight.setPosition(rightClosePos);
                if(cycle.equals("yes")) {
                    drive.followTrajectorySequence(traj3c); // drive to stack 1
                    drive.clawRight.setPosition(rightOpenPos);
                    drive.clawLeft.setPosition(leftOpenPos);
                    drive.followTrajectorySequence(traj3c2);
                    drive.clawLeft.setPosition(leftClosePos);
                    drive.clawRight.setPosition(rightClosePos);
                    sleep(200);
                    drive.slide.setTargetPosition(242);
                    drive.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    drive.wrist.setPosition(0.34);
                    drive.followTrajectorySequence(traj4c);
                    drive.slide.setTargetPosition(2000);
                    drive.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    drive.followTrajectorySequence(traj5c); // drive to backboard 2
                    sleep(100);
                    drive.clawRight.setPosition(rightOpenPos);
                    drive.clawLeft.setPosition(leftOpenPos);
                    sleep(200);
                    drive.followTrajectorySequence(traj7c);
                    drive.clawLeft.setPosition(leftClosePos);
                    drive.clawRight.setPosition(rightClosePos);
                    drive.slide.setTargetPosition(90);
                    drive.wrist.setPosition(0.169);

                    drive.followTrajectorySequence(traj3cc);
                    drive.clawRight.setPosition(rightOpenPos);
                    drive.clawLeft.setPosition(leftOpenPos);
                    drive.followTrajectorySequence(traj3cc2);
                    drive.clawLeft.setPosition(leftClosePos);
                    drive.clawRight.setPosition(rightClosePos);
                    sleep(120);
                    drive.slide.setTargetPosition(100);
                    drive.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    drive.wrist.setPosition(0.34);
                    drive.followTrajectorySequence(traj4cc);
                    drive.slide.setTargetPosition(2000);
                    drive.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    drive.followTrajectorySequence(traj5cc);
                    sleep(100);
                    drive.clawRight.setPosition(rightOpenPos);
                    drive.clawLeft.setPosition(leftOpenPos);
                    drive.followTrajectorySequence(traj6cc);
                    sleep(100);
                    drive.followTrajectorySequence(traj6ccc);
                    drive.slide.setTargetPosition(0);
                    drive.wrist.setPosition(0.169);
                    drive.clawLeft.setPosition(leftClosePos);
                    drive.clawRight.setPosition(rightClosePos);
                    drive.followTrajectorySequence(traj7cc);
                }
        }
    }
}