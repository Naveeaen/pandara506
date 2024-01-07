package org.firstinspires.ftc.teamcode.pandara506;

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
import org.firstinspires.ftc.teamcode.pandara506.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.pandara506.opencv.PipelineRedBack;
import org.firstinspires.ftc.teamcode.pandara506.opencv.PipelineRedFront;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

/*
 * This is a simple routine to test turning capabilities.
 */
@Config
@Autonomous(group = "auto")
public class automusRedFront extends LinearOpMode {

    OpenCvCamera webCam;
    public PipelineRedFront detector;
    public  String position = "Insert Here";
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        //vision cam
        //cameraqa
        int cameraMotionViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        detector = new PipelineRedFront();
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
                .splineTo(new Vector2d(36, 9), Math.toRadians(90))
                .build();
        Trajectory traj2a = drive.trajectoryBuilder(traj1a.end())
                .back(15)
                .build();
        Trajectory traj3a = drive.trajectoryBuilder(traj2a.end())
                .splineTo(new Vector2d(36, -28), Math.toRadians(-90))
                .build();
        Trajectory traj4a = drive.trajectoryBuilder(traj3a.end())
                .forward(8)
                .build();
        Trajectory traj5a = drive.trajectoryBuilder(traj4a.end())
                .back(8)
                .build();

        Trajectory traj1b = drive.trajectoryBuilder(new Pose2d())
                .forward(30)
                .build();
        Trajectory traj2b = drive.trajectoryBuilder(traj1b.end())
                .back(15)
                .build();
        Trajectory traj3b = drive.trajectoryBuilder(traj2b.end())
                .splineTo(new Vector2d(28, -28), Math.toRadians(-90))
                .build();
        Trajectory traj4b = drive.trajectoryBuilder(traj3b.end())
                .forward(8)
                .build();

        Trajectory traj1c = drive.trajectoryBuilder(new Pose2d())
                .splineTo(new Vector2d(23, -6), Math.toRadians(-20))
                .build();
        Trajectory traj2c = drive.trajectoryBuilder(traj1c.end())
                .back(10)
                .build();
        Trajectory traj3c = drive.trajectoryBuilder(traj2c.end())
                .splineTo(new Vector2d(21, -28), Math.toRadians(-90))
                .build();
        Trajectory traj4c = drive.trajectoryBuilder(traj3c.end())
                .forward(8)
                .build();

        drive.clawLeft.setPosition(0.07);
        drive.clawRight.setPosition(0.4);

        waitForStart();
        int cameraZone = 1;
        switch (cameraZone) {
            case 1:
                drive.followTrajectory(traj1a);
                drive.clawLeft.setPosition(0.2);
                drive.wrist.setPosition(0.34);
                drive.followTrajectory(traj2a);
                drive.followTrajectory(traj3a);
                drive.slide.setPower(0.9);
                drive.slide.setTargetPosition(1300);
                drive.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                while (opModeIsActive() && drive.slide.isBusy()) {
                }
                drive.followTrajectory(traj4a);
                drive.clawRight.setPosition(0.27);
                drive.followTrajectory(traj5a);
                drive.slide.setPower(0.7);
                drive.slide.setTargetPosition(1);
                drive.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                while (opModeIsActive() && drive.slide.isBusy()){
                }
                drive.clawLeft.setPosition(0.07);
                drive.clawRight.setPosition(0.38);
                drive.wrist.setPosition(0.169);
                break;
            case 2:
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
                break;
        }
    }
}