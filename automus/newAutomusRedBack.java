package org.firstinspires.ftc.teamcode.pandara506.automus;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.pandara506.camera.PipelineRedBack;
import org.firstinspires.ftc.teamcode.pandara506.roadrunner.DriveConstants;
import org.firstinspires.ftc.teamcode.pandara506.mainPrograms.Hardware;
import org.firstinspires.ftc.teamcode.pandara506.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Config
@Autonomous (group = "autoRed")
public class newAutomusRedBack extends LinearOpMode {
    OpenCvCamera webCam;
    PipelineRedBack detector;
    String position = "left";
    int timeout = 0;
    String cycle = "yes";
    String park = "left";
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    @Override
    public void runOpMode() throws InterruptedException {
        Hardware drive = new Hardware(hardwareMap);

        drive.closeL();
        drive.closeR();
        drive.wristD();

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
            telemetry.addData("rightRed", detector.leftRed);
            telemetry.update();

            dashboardTelemetry.addData("position", position);
            dashboardTelemetry.addData("leftRed", detector.leftRed);
            dashboardTelemetry.addData("centerRed", detector.centerRed);
            dashboardTelemetry.addData("rightRed", detector.leftRed);
            dashboardTelemetry.update();

            dashboardTelemetry.addData("position", position);
        }

        //left
        TrajectorySequence spikeL = drive.trajectorySequenceBuilder(new Pose2d(0, 0))
                .lineTo(new Vector2d(32.4, 4))
                .build();
        TrajectorySequence stackL1 = drive.trajectorySequenceBuilder(spikeL.end())
                .lineToLinearHeading(new Pose2d(31.7, -3, Math.toRadians(90)))
                .lineTo(new Vector2d(31, 14.5))
                .setVelConstraint(Hardware.getVelocityConstraint(0.1 * DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .forward(5.8)
                .build();
        Trajectory stackL1b = drive.trajectoryBuilder(stackL1.end())
                .back(5)
                .build();
        TrajectorySequence boardL1a = drive.trajectorySequenceBuilder(stackL1b.end())
                .lineToLinearHeading(new Pose2d(5, -10, Math.toRadians(-90)))
                .lineTo(new Vector2d(5, -72))
                .build();
        TrajectorySequence boardL1b = drive.trajectorySequenceBuilder(boardL1a.end())
                .lineTo(new Vector2d(34, -91))
                .setVelConstraint(Hardware.getVelocityConstraint(0.15 * DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .forward(5)
                .build();
        TrajectorySequence boardL1c = drive.trajectorySequenceBuilder(boardL1b.end())
                .setVelConstraint(Hardware.getVelocityConstraint(0.15 * DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .back(5)
                .build();
        TrajectorySequence boardL1d = drive.trajectorySequenceBuilder(boardL1c.end())
                .setVelConstraint(Hardware.getVelocityConstraint(0.15 * DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .strafeRight(6)
                .forward(5)
                .build();
        TrajectorySequence stackL2 = drive.trajectorySequenceBuilder(boardL1d.end())
                .back(5)
                .lineToLinearHeading(new Pose2d(6, -72, Math.toRadians(86)))
                .lineTo(new Vector2d(8, -10))
                .lineTo(new Vector2d(34, 12))
                .build();
        TrajectorySequence stackL2b = drive.trajectorySequenceBuilder(stackL2.end())
                .setVelConstraint(Hardware.getVelocityConstraint(0.15 * DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .forward(7.5)
                .build();
        Trajectory stackL2c = drive.trajectoryBuilder(stackL2b.end())
                .back(7)
                .build();
        TrajectorySequence boardL2a = drive.trajectorySequenceBuilder(stackL2c.end())
                .setVelConstraint(Hardware.getVelocityConstraint(0.8 * DriveConstants.MAX_VEL, 0.6*DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .lineToLinearHeading(new Pose2d(11, 5, Math.toRadians(-89)))
                .lineTo(new Vector2d(15, -72))
                .build();
        TrajectorySequence boardL2b = drive.trajectorySequenceBuilder(boardL2a.end())
                .setVelConstraint(Hardware.getVelocityConstraint(0.6 * DriveConstants.MAX_VEL, 0.6*DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .lineTo(new Vector2d(37, -90))
                .setVelConstraint(Hardware.getVelocityConstraint(0.15 * DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .forward(5)
                .build();
        TrajectorySequence fstL = drive.trajectorySequenceBuilder(boardL2b.end())
                .setVelConstraint(Hardware.getVelocityConstraint(0.15 * DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .lineTo(new Vector2d(31, -91))
                .build();
        TrajectorySequence parkL = drive.trajectorySequenceBuilder(fstL.end())
                .back(5)
                .strafeLeft(20)
                .build();

        //center
        TrajectorySequence spikeC = drive.trajectorySequenceBuilder(new Pose2d(0, 0))
                .lineTo(new Vector2d(32, -3))
                .build();
        TrajectorySequence stackC1 = drive.trajectorySequenceBuilder(spikeC.end())
                .back(8)
                .lineToLinearHeading(new Pose2d(30.5, 14.5, Math.toRadians(90)))
                .setVelConstraint(Hardware.getVelocityConstraint(0.1 * DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .forward(5.68)
                .build();
        TrajectorySequence stackC1b = drive.trajectorySequenceBuilder(stackC1.end())
                .back(5)
                .build();
        TrajectorySequence boardC1a = drive.trajectorySequenceBuilder(stackC1b.end())
                .lineToLinearHeading(new Pose2d(4, -10, Math.toRadians(-90)))
                .lineTo(new Vector2d(4, -72))
                .build();
        TrajectorySequence boardC1b = drive.trajectorySequenceBuilder(boardC1a.end())
                .lineTo(new Vector2d(26, -91))
                .setVelConstraint(Hardware.getVelocityConstraint(0.15 * DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .forward(5)
                .build();
        TrajectorySequence stackC2 = drive.trajectorySequenceBuilder(boardC1b.end())
                .back(5)
                .lineToLinearHeading(new Pose2d(4, -72, Math.toRadians(83)))
                .lineTo(new Vector2d(8, -10))
                .lineTo(new Vector2d(33, 14))
                .build();
        TrajectorySequence stackC2b = drive.trajectorySequenceBuilder(stackC2.end())
                .setVelConstraint(Hardware.getVelocityConstraint(0.15 * DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .forward(8)
                .build();
        TrajectorySequence stackC2c = drive.trajectorySequenceBuilder(stackC2b.end())
                .back(7)
                .build();
        TrajectorySequence boardC2a = drive.trajectorySequenceBuilder(stackC2c.end())
                .setVelConstraint(Hardware.getVelocityConstraint(0.6 * DriveConstants.MAX_VEL, 0.6*DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .lineToLinearHeading(new Pose2d(11, 5, Math.toRadians(-93)))
                .lineTo(new Vector2d(11, -72))
                .build();
        TrajectorySequence boardC2b = drive.trajectorySequenceBuilder(boardC2a.end())
                .setVelConstraint(Hardware.getVelocityConstraint(0.6 * DriveConstants.MAX_VEL, 0.6*DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .lineTo(new Vector2d(34, -90))
                .setVelConstraint(Hardware.getVelocityConstraint(0.15 * DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .forward(5)
                .build();
        TrajectorySequence fstC = drive.trajectorySequenceBuilder(boardC2b.end())
                .setVelConstraint(Hardware.getVelocityConstraint(0.15 * DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .lineTo(new Vector2d(28, -91))
                .build();
        TrajectorySequence parkC = drive.trajectorySequenceBuilder(fstC.end())
                .back(5)
                .strafeLeft(20)
                .build();

        //right
        TrajectorySequence spikeR = drive.trajectorySequenceBuilder(new Pose2d(0, 0))
                .lineToLinearHeading(new Pose2d(34, -8, Math.toRadians(-90)))
                .build();
        TrajectorySequence stackR1 = drive.trajectorySequenceBuilder(spikeR.end())
                .lineToLinearHeading(new Pose2d(31.7, 11, Math.toRadians(88.5)))
                .setVelConstraint(Hardware.getVelocityConstraint(0.1 * DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .forward(10)
                .build();
        Trajectory stackR1b = drive.trajectoryBuilder(stackR1.end())
                .back(7)
                .build();
        TrajectorySequence boardR1a = drive.trajectorySequenceBuilder(stackR1b.end())
                .lineToLinearHeading(new Pose2d(5, 0, Math.toRadians(-89)))
                .lineTo(new Vector2d(5, -72))
                .build();
        TrajectorySequence boardR1b = drive.trajectorySequenceBuilder(boardR1a.end())
                .lineTo(new Vector2d(27, -91))
                .setVelConstraint(Hardware.getVelocityConstraint(0.15 * DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .forward(5)
                .build();
        TrajectorySequence stackR2 = drive.trajectorySequenceBuilder(boardR1b.end())
                .back(10)
                .lineToLinearHeading(new Pose2d(10, -72, Math.toRadians(82)))
                .lineTo(new Vector2d(12, -10))
                .lineTo(new Vector2d(35, 13))
                .build();
        TrajectorySequence stackR2b = drive.trajectorySequenceBuilder(stackR2.end())
                .setVelConstraint(Hardware.getVelocityConstraint(0.15 * DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .forward(8.5)
                .build();
        Trajectory stackR2c = drive.trajectoryBuilder(stackR2b.end())
                .back(7)
                .build();
        TrajectorySequence boardR2a = drive.trajectorySequenceBuilder(stackR2c.end())
                .setVelConstraint(Hardware.getVelocityConstraint(0.8 * DriveConstants.MAX_VEL, 0.6*DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .lineToLinearHeading(new Pose2d(15, 5, Math.toRadians(-93)))
                .lineTo(new Vector2d(15, -72))
                .build();
        TrajectorySequence boardR2b = drive.trajectorySequenceBuilder(boardR2a.end())
                .setVelConstraint(Hardware.getVelocityConstraint(0.6 * DriveConstants.MAX_VEL, 0.6*DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .lineTo(new Vector2d(40, -90))
                .setVelConstraint(Hardware.getVelocityConstraint(0.15 * DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .forward(5)
                .build();
        TrajectorySequence fstR = drive.trajectorySequenceBuilder(boardR2b.end())
                .setVelConstraint(Hardware.getVelocityConstraint(0.15 * DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .lineTo(new Vector2d(34, -88))
                .build();
        TrajectorySequence parkR = drive.trajectorySequenceBuilder(fstR.end())
                .back(5)
                .strafeLeft(20)
                .build();
        waitForStart();

        sleep(timeout);
        switch(position){
            case("left"):
                drive.followTrajectorySequence(spikeL); // spike L
                drive.openL();
                sleep(100);
                drive.slidesTo(360);

                drive.followTrajectorySequence(stackL1); // stack L 1
                sleep(100);
                drive.closeL();
                sleep(200);
                drive.slidesTo(430, 0.3);
                sleep(300);

                drive.followTrajectory(stackL1b);
                drive.slidesTo(0, 0.3);
                drive.followTrajectorySequence(boardL1a); // board L 1
                drive.wristU();
                drive.slidesTo(1200);
                drive.followTrajectorySequence(boardL1b);
                drive.openR();
                drive.slidesTo(500, 0.5);
                drive.followTrajectorySequence(boardL1c);
                drive.slidesTo(1600);
                drive.followTrajectorySequence(boardL1d);
                drive.openL();
                drive.slidesTo(0, 0.25);
                sleep(100);
                drive.wristD();

                drive.followTrajectorySequence(stackL2); // stack L 2
                drive.slidesTo(170);
                drive.followTrajectorySequence(stackL2b);
                drive.closeR();
                drive.closeL();
                sleep(200);
                drive.slidesTo(230);

                drive.followTrajectory(stackL2c);
                drive.slidesTo(0);
                drive.followTrajectorySequence(boardL2a); // board L 2
                drive.wristU();
                drive.slidesTo(1500);
                drive.followTrajectorySequence(boardL2b);
                drive.openL();
                drive.openR();
                drive.followTrajectorySequence(fstL);
                drive.slidesTo(0);
                sleep(100);
                drive.wristD();

                drive.followTrajectorySequence(parkL); // park L
                break;

            case("center"):
                drive.followTrajectorySequence(spikeC); // spike C
                drive.openL();
                sleep(100);
                drive.slidesTo(351);

                drive.followTrajectorySequence(stackC1); // stack C 1
                sleep(100);
                drive.closeL();
                sleep(200);
                drive.slidesTo(430, 0.3);
                sleep(300);

                drive.followTrajectorySequence(stackC1b);
                drive.slidesTo(0, 0.3);
                drive.followTrajectorySequence(boardC1a); // board C 1
                drive.wristU();
                drive.slidesTo(1200);
                drive.followTrajectorySequence(boardC1b);
                drive.openL();
                drive.openR();
                drive.slidesTo(0, 0.5);
                sleep(100);
                drive.wristD();

                drive.followTrajectorySequence(stackC2); // stack C 2
                drive.slidesTo(170);
                drive.followTrajectorySequence(stackC2b);
                sleep(200);
                drive.closeR();
                drive.closeL();
                sleep(200);
                drive.slidesTo(220);

                drive.followTrajectorySequence(stackC2c);
                drive.slidesTo(0);
                drive.followTrajectorySequence(boardC2a); // board C 2
                drive.wristU();
                drive.slidesTo(1500);
                drive.followTrajectorySequence(boardC2b);
                drive.openL();
                drive.openR();
                drive.followTrajectorySequence(fstC);
                drive.slidesTo(0);
                sleep(100);
                drive.wristD();

                drive.followTrajectorySequence(parkC); // park C
                break;

            case("right"):
                drive.followTrajectorySequence(spikeR); // spike R
                drive.openL();
                sleep(100);
                drive.slidesTo(351);

                drive.followTrajectorySequence(stackR1); // stack R 1
                sleep(100);
                drive.closeL();
                sleep(200);
                drive.slidesTo(430, 0.3);
                sleep(300);

                drive.followTrajectory(stackR1b);
                drive.slidesTo(0, 0.3);
                drive.followTrajectorySequence(boardR1a); // board R 1
                drive.wristU();
                drive.slidesTo(1200);
                drive.followTrajectorySequence(boardR1b);
                drive.openL();
                drive.openR();
                drive.slidesTo(0, 0.2);
                sleep(100);
                drive.wristD();

                drive.followTrajectorySequence(stackR2); // stack R 2
                drive.slidesTo(170);
                drive.followTrajectorySequence(stackR2b);
                drive.closeR();
                drive.closeL();
                sleep(200);
                drive.slidesTo(220);

                drive.followTrajectory(stackR2c);
                drive.slidesTo(0);
                drive.followTrajectorySequence(boardR2a); // board R 2
                drive.wristU();
                drive.slidesTo(1500);
                drive.followTrajectorySequence(boardR2b);
                drive.openL();
                drive.openR();
                drive.followTrajectorySequence(fstR);
                drive.slidesTo(0);
                sleep(100);
                drive.wristD();

                drive.followTrajectorySequence(parkR); // park R
                break;
        }
    }
}
