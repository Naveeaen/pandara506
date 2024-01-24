package org.firstinspires.ftc.teamcode.pandara506;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
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

@Autonomous(group = "drive")
public class autoTest extends LinearOpMode {
    oldHardware robot = oldHardware.getInstance();
    OpenCvCamera webCam;
    public PipelineBlueFront detector;
    private String position = "Insert Here";
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();


    public void runOpMode() {
        robot.init(hardwareMap);
        Hardware drive = new Hardware(hardwareMap);

        Pose2d startPose = new Pose2d(-36,  63.625, Math.toRadians(90));

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        int cameraMotionViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        detector = new PipelineBlueFront();
        webCam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "wc1"), cameraMotionViewId);
        webCam.openCameraDevice();
        FtcDashboard.getInstance().startCameraStream(webCam, 0);
        webCam.startStreaming(640,480, OpenCvCameraRotation.UPSIDE_DOWN);
        webCam.setPipeline(detector);
        while (!isStarted() && !isStopRequested()) {
            position = "Left";//detector.position;
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
        waitForStart();
        TrajectorySequence trajSeq1a = drive.trajectorySequenceBuilder(startPose)
                .strafeLeft(10)
                .lineTo(new Vector2d(-30, -37))
                .turn(Math.toRadians(88))
                .forward(5)
                .build();
        TrajectorySequence trajSeq1b = drive.trajectorySequenceBuilder(trajSeq1a.end())
                .waitSeconds(0.5)
                .back(5)
                .turn(Math.toRadians(178))
                .lineTo(new Vector2d(30, -37)) //49
                .strafeLeft(6)
                .build();
        TrajectorySequence trajSeq1c = drive.trajectorySequenceBuilder(trajSeq1b.end())
                .waitSeconds(0.4)
                .build();

        if ((position.equals("Left"))) {
            drive.clawLeft.setPosition(0.39);
            drive.clawRight.setPosition(0.38);
            drive.followTrajectorySequence(trajSeq1a);
            drive.clawLeft.setPosition(0.56);
            drive.followTrajectorySequence(trajSeq1b);
            drive.slide.setPower(0.9);
            drive.slide.setTargetPosition(1000);
            drive.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            drive.wrist.setPosition(0.475);
            if(drive.slide.getCurrentPosition() >= 950){
                drive.clawRight.setPosition(0.27);
            }
            drive.followTrajectorySequence(trajSeq1c);

            //reset stuff
            drive.clawLeft.setPosition(0.39);
            drive.clawRight.setPosition(0.38);
            drive.wrist.setPosition(0.2);
            drive.slide.setPower(0.7);
            drive.slide.setTargetPosition(1);
            drive.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            drive.followTrajectorySequence(trajSeq1c);
        }

        if ((position.equals("Right"))) {

        }
    }
}
