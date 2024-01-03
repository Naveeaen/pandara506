package org.firstinspires.ftc.teamcode.pandara506;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.pandara506.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.pandara506.opencv.PipelineRed;
import org.firstinspires.ftc.teamcode.pandara506.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.pandara506.opencv.OpenCV;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(group = "drive")
public class redAuto extends LinearOpMode{
    OpenCvCamera webCam;
    public PipelineRed detector;
    public  String position = "Insert Here";
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(-36,  -63.625, Math.toRadians(90));
        drive.setPoseEstimate(startPose);

        int cameraMotionViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        detector = new PipelineRed();
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
        waitForStart();

        if (isStopRequested()) return;
        //1
        TrajectorySequence trajSeq1a = drive.trajectorySequenceBuilder(startPose)
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(45, Math.toRadians(304), 11.73))
                .strafeRight(6)
                .forward(30)
                .turn(Math.toRadians(90))
                .forward(5)
                .build();
        TrajectorySequence trajSeq1b = drive.trajectorySequenceBuilder(trajSeq1a.end())
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(45, Math.toRadians(304), 11.73))
                .back(5)
                .strafeLeft(2)
                .turn(Math.toRadians(180))
                .forward(75)
                .strafeLeft(10)
                .build();
        TrajectorySequence trajSeq1c = drive.trajectorySequenceBuilder(trajSeq1b.end())
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(10, Math.toRadians(304), 11.73))
                .forward(12)
                .build();
        TrajectorySequence trajSeq1d = drive.trajectorySequenceBuilder(trajSeq1c.end())
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(7, Math.toRadians(304), 11.73))
                .back(3)
                .build();
        TrajectorySequence trajSeq1e = drive.trajectorySequenceBuilder(trajSeq1b.end())
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(45, Math.toRadians(304), 11.73))
                .waitSeconds(0.4)
                .build();
        TrajectorySequence trajSeq1f = drive.trajectorySequenceBuilder(trajSeq1d.end())
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(10, Math.toRadians(304), 11.73))
                .turn(Math.toRadians(180))
                .build();
        //2
        TrajectorySequence trajSeq2a = drive.trajectorySequenceBuilder(startPose)
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(32, Math.toRadians(304), 11.73))
                .strafeRight(6)
                .forward(30)
                .build();
        TrajectorySequence trajSeq2b = drive.trajectorySequenceBuilder(trajSeq2a.end())
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(32, Math.toRadians(304), 11.19))
                .back(2)
                .turn(Math.toRadians(-90))
                .forward(75)
                .build();
        TrajectorySequence trajSeq2c = drive.trajectorySequenceBuilder(trajSeq2b.end())
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(32, Math.toRadians(304), 11.73))
                .waitSeconds(0.04)
                .build();
        //3
        TrajectorySequence trajSeq3a = drive.trajectorySequenceBuilder(startPose)
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(32, Math.toRadians(304), 11.73))
                .strafeRight(10)
                .forward(24)
                .turn(Math.toRadians(90))
                .forward(24)
                .build();
        TrajectorySequence trajSeq3b = drive.trajectorySequenceBuilder(trajSeq3a.end())
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(32, Math.toRadians(304), 11.73))
                //.back(5)
                .forward(70)
                .strafeRight(6)
                .turn(Math.toRadians(180))
                .build();
        TrajectorySequence trajSeq3c = drive.trajectorySequenceBuilder(trajSeq3b.end())
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(32, Math.toRadians(304), 11.73))
                .waitSeconds(0.4)
                .build();

        switch(position){
            case "Left":
                drive.clawLeft.setPosition(0.07);
                drive.clawRight.setPosition(0.4);
                drive.followTrajectorySequence(trajSeq1a);
                drive.clawLeft.setPosition(0.2);
                drive.wrist.setPosition(0.34);
                drive.followTrajectorySequence(trajSeq1b);
                drive.slide.setPower(0.9);
                drive.slide.setTargetPosition(1500);
                drive.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                drive.followTrajectorySequence(trajSeq1e);
                while (opModeIsActive() && drive.slide.isBusy()){

                }
                drive.followTrajectorySequence(trajSeq1c);
                drive.clawRight.setPosition(0.27);
                drive.followTrajectorySequence(trajSeq1d);
                drive.slide.setPower(0.7);
                drive.slide.setTargetPosition(1);
                drive.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                drive.followTrajectorySequence(trajSeq1e);
                while (opModeIsActive() && drive.slide.isBusy()){

                }
                //reset stuff
                drive.clawLeft.setPosition(0.07);
                drive.clawRight.setPosition(0.38);
                drive.wrist.setPosition(0.169);
                //drive.followTrajectorySequence(trajSeq1f);
                drive.followTrajectorySequence(trajSeq1e);
                break;
            case "Center":
                drive.clawLeft.setPosition(0.39);
                drive.clawRight.setPosition(0.38);
                drive.followTrajectorySequence(trajSeq2a);
                drive.clawLeft.setPosition(0.56);
                drive.followTrajectorySequence(trajSeq2b);
                drive.slide.setPower(0.9);
                drive.slide.setTargetPosition(4800);
                drive.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                drive.followTrajectorySequence(trajSeq2c);
                while (opModeIsActive() && drive.slide.isBusy()){

                }
                drive.wrist.setPosition(0.475);
                drive.clawRight.setPosition(0.27);
                drive.slide.setPower(0.7);
                drive.slide.setTargetPosition(1);
                drive.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                drive.followTrajectorySequence(trajSeq2c);
                while (opModeIsActive() && drive.slide.isBusy()){

                }
                //reset stuff
                drive.clawLeft.setPosition(0.39);
                drive.clawRight.setPosition(0.38);
                drive.wrist.setPosition(0.2);

                drive.followTrajectorySequence(trajSeq2c);
                break;
            case "Right":
                drive.setMotorPowers(0.2, 0.2, 0.2, 0.2);
                drive.clawLeft.setPosition(0.39);
                drive.clawRight.setPosition(0.38);
                drive.followTrajectorySequence(trajSeq3a);
                drive.clawLeft.setPosition(0.56);
                drive.followTrajectorySequence(trajSeq3b);
                drive.slide.setPower(0.9);
                drive.slide.setTargetPosition(4800);
                drive.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                drive.followTrajectorySequence(trajSeq3c);
                while (opModeIsActive() && drive.slide.isBusy()){

                }
                drive.wrist.setPosition(0.475);
                drive.clawRight.setPosition(0.27);
                drive.slide.setPower(0.7);
                drive.slide.setTargetPosition(1);
                drive.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                drive.followTrajectorySequence(trajSeq3c);
                while (opModeIsActive() && drive.slide.isBusy()){

                }
                //reset stuff
                drive.clawLeft.setPosition(0.39);
                drive.clawRight.setPosition(0.38);
                drive.wrist.setPosition(0.2);

                drive.followTrajectorySequence(trajSeq3c);
                break;
        }

    }
}
