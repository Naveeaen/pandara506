package org.firstinspires.ftc.teamcode.pandara506.automus;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.pandara506.camera.PipelineRedFront;
import org.firstinspires.ftc.teamcode.pandara506.roadrunner.DriveConstants;
import org.firstinspires.ftc.teamcode.pandara506.mainPrograms.Hardware;
import org.firstinspires.ftc.teamcode.pandara506.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.lang.invoke.VolatileCallSite;
import java.util.Vector;

@Config
@Autonomous(group = "autoRed")
public class newAutomusRedFront extends LinearOpMode {
    OpenCvCamera webCam;
    PipelineRedFront detector;
    String position = "left";
    int timeout = 0;
    String cycle = "no";
    String park = "left";
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();
    boolean Bpressed = false;
    boolean Ltriggered = false;
    boolean Rtriggered = false;
    boolean Cycswitch = true;

    @Override
    public void runOpMode() throws InterruptedException {
        Hardware drive = new Hardware(hardwareMap);

        drive.closeL();
        drive.closeR();
        drive.wristD();

        //cameraqa
        int cameraMotionViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
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
            telemetry.addData("Cycle", cycle);
            telemetry.addData("Timeout", timeout);
            telemetry.update();

            dashboardTelemetry.addData("position", position);
            dashboardTelemetry.addData("leftRed", detector.leftRed);
            dashboardTelemetry.addData("centerRed", detector.centerRed);
            dashboardTelemetry.addData("rightRed", detector.rightRed);
            dashboardTelemetry.addData("Cycle", cycle);
            dashboardTelemetry.addData("Timeout", timeout);
            dashboardTelemetry.update();

            dashboardTelemetry.addData("position", position);

            if (gamepad1.b && !Bpressed) {
                if (Cycswitch) {
                    cycle = "yes";
                    Cycswitch = false;
                } else {
                    cycle = "no";
                    Cycswitch = true;
                }
                Bpressed = true;
            } else if (!gamepad1.b) {
                Bpressed = false;
            }

            // timeout +
            if (gamepad1.right_trigger != 0 && !Rtriggered) {
                timeout += 400 * gamepad1.right_trigger;
                Rtriggered = true;
            } else if (gamepad1.right_trigger == 0) {
                Rtriggered = false;
            }
            //timeout -
            if (gamepad1.left_trigger != 0 && !Ltriggered) {
                timeout -= 400 * gamepad1.left_trigger;
                Ltriggered = true;
            } else if (gamepad1.left_trigger == 0) {
                Ltriggered = false;
            }

            if(timeout < 0){
                timeout = 0;
            }
        }

        //left trajectories
        TrajectorySequence spikeL = drive.trajectorySequenceBuilder(new Pose2d(0, 0))
                .lineToLinearHeading(new Pose2d(32, 8, Math.toRadians(90)))
                .build();
        TrajectorySequence boardL1 = drive.trajectorySequenceBuilder(spikeL.end())
                .lineToLinearHeading(new Pose2d(36, -32, Math.toRadians(-90)))
                .setVelConstraint(Hardware.getVelocityConstraint(0.15 * DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .forward(5)
                .build();
        TrajectorySequence stackL1;
        if(cycle.equals("yes")) {
            stackL1 = drive.trajectorySequenceBuilder(boardL1.end())
                    .back(5)
                    .lineToLinearHeading(new Pose2d(55, -20, Math.toRadians(90)))
                    .lineTo(new Vector2d(59.7, 74))
                    .build();
        }
        else{
            stackL1 = drive.trajectorySequenceBuilder(boardL1.end())
                    .back(5)
                    .strafeRight(-30)
                    .turn(Math.toRadians(180))
                    .back(5)
                    .build();
        }
        TrajectorySequence stackL1b = drive.trajectorySequenceBuilder(stackL1.end())
                .setVelConstraint(Hardware.getVelocityConstraint(0.1 * DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .lineTo(new Vector2d(60, 75))
                .build();
        TrajectorySequence boardL2a = drive.trajectorySequenceBuilder(stackL1b.end())
                .lineToLinearHeading(new Pose2d(60, 60, Math.toRadians(-90)))
                .build();
        TrajectorySequence boardL2b = drive.trajectorySequenceBuilder(boardL2a.end())
                .lineTo(new Vector2d(60, -20))
                .build();
        TrajectorySequence boardL2c = drive.trajectorySequenceBuilder(boardL2b.end())
                .lineTo(new Vector2d(29, -32))
                .setVelConstraint(Hardware.getVelocityConstraint(0.15 * DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .forward(5.2)
                .build();
        TrajectorySequence fstL1 = drive.trajectorySequenceBuilder(boardL2c.end())
                .setVelConstraint(Hardware.getVelocityConstraint(0.15 * DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .lineTo(new Vector2d(23, -35))
                .build();
        TrajectorySequence stackL2 = drive.trajectorySequenceBuilder(fstL1.end())
                .back(5)
                .lineToLinearHeading(new Pose2d(55, -20, Math.toRadians(90)))
                .lineTo(new Vector2d(63, 70))
                .build();
        TrajectorySequence stackL2b = drive.trajectorySequenceBuilder(stackL2.end())
                .setVelConstraint(Hardware.getVelocityConstraint(0.15 * DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .lineTo(new Vector2d(64.5, 78))
                .build();
        TrajectorySequence boardL3a = drive.trajectorySequenceBuilder(stackL2b.end())
                .lineToLinearHeading(new Pose2d(60, 60, Math.toRadians(-91)))
                .build();
        TrajectorySequence boardL3b = drive.trajectorySequenceBuilder(boardL3a.end())
                .lineTo(new Vector2d(60, -20))
                .build();
        TrajectorySequence boardL3c = drive.trajectorySequenceBuilder(boardL3b.end())
                .lineTo(new Vector2d(28, -32))
                .setVelConstraint(Hardware.getVelocityConstraint(0.15 * DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .forward(7)
                .build();
        TrajectorySequence fstL2 = drive.trajectorySequenceBuilder(boardL3c.end())
                .setVelConstraint(Hardware.getVelocityConstraint(0.15 * DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .lineTo(new Vector2d(34, -35.5))
                .build();
        TrajectorySequence parkL = drive.trajectorySequenceBuilder(fstL2.end())
                .back(2)
                .strafeLeft(20)
                .build();

        //center trajectories
        TrajectorySequence spikeC = drive.trajectorySequenceBuilder(new Pose2d(0, 0))
                .lineTo(new Vector2d(32, 8))
                .build();
        TrajectorySequence boardC1 = drive.trajectorySequenceBuilder(spikeC.end())
                .lineToLinearHeading(new Pose2d(31, -32, Math.toRadians(-90)))
                .setVelConstraint(Hardware.getVelocityConstraint(0.15 * DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .forward(5)
                .build();
        TrajectorySequence stackC1;
        if(cycle.equals("yes")) {
            stackC1 = drive.trajectorySequenceBuilder(boardC1.end())
                    .back(5)
                    .lineToLinearHeading(new Pose2d(60, -20, Math.toRadians(90)))
                    .lineTo(new Vector2d(59.7, 74))
                    .build();
        }
        else{
            stackC1 = drive.trajectorySequenceBuilder(boardC1.end())
                    .back(5)
                    .strafeRight(-29)
                    .turn(Math.toRadians(180))
                    .back(5)
                    .build();
        }
        TrajectorySequence stackC1b = drive.trajectorySequenceBuilder(stackC1.end())
                .setVelConstraint(Hardware.getVelocityConstraint(0.1 * DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .lineTo(new Vector2d(59.7, 79))
                .build();
        TrajectorySequence boardC2a = drive.trajectorySequenceBuilder(stackC1b.end())
                .lineTo(new Vector2d(55, 60))
                .build();
        TrajectorySequence boardC2b = drive.trajectorySequenceBuilder(boardC2a.end())
                .lineToLinearHeading(new Pose2d(55, -20, Math.toRadians(-93)))
                .build();
        TrajectorySequence boardC2c = drive.trajectorySequenceBuilder(boardC2b.end())
                .lineTo(new Vector2d(20, -31))
                .setVelConstraint(Hardware.getVelocityConstraint(0.15 * DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .forward(5.2)
                .build();
        TrajectorySequence fstC1 = drive.trajectorySequenceBuilder(boardC2c.end())
                .setVelConstraint(Hardware.getVelocityConstraint(0.15 * DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .lineTo(new Vector2d(26, -34))
                .build();
        TrajectorySequence stackC2 = drive.trajectorySequenceBuilder(fstC1.end())
                .back(5)
                .lineToLinearHeading(new Pose2d(55, -20, Math.toRadians(90)))
                .lineTo(new Vector2d(63, 72))
                .build();
        TrajectorySequence stackC2b = drive.trajectorySequenceBuilder(stackC2.end())
                .setVelConstraint(Hardware.getVelocityConstraint(0.15 * DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .lineTo(new Vector2d(63, 80))
                .build();
        TrajectorySequence boardC3a = drive.trajectorySequenceBuilder(stackC2b.end())
                .lineTo(new Vector2d(63, 60))
                .build();
        TrajectorySequence boardC3b = drive.trajectorySequenceBuilder(boardC3a.end())
                .lineToLinearHeading(new Pose2d(60, -20, Math.toRadians(-93)))
                .build();
        TrajectorySequence boardC3c = drive.trajectorySequenceBuilder(boardC3b.end())
                .lineTo(new Vector2d(28, -31))
                .setVelConstraint(Hardware.getVelocityConstraint(0.15 * DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .forward(6)
                .build();
        TrajectorySequence fstC2 = drive.trajectorySequenceBuilder(boardC3c.end())
                .setVelConstraint(Hardware.getVelocityConstraint(0.15 * DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .lineTo(new Vector2d(34, -34.5))
                .build();
        TrajectorySequence parkC = drive.trajectorySequenceBuilder(fstC2.end())
                .back(2)
                .strafeLeft(20)
                .build();

        //right trajectories
        TrajectorySequence spikeR = drive.trajectorySequenceBuilder(new Pose2d(0, 0))
                .lineToLinearHeading(new Pose2d(32, -15, Math.toRadians(90)))
                .build();
        TrajectorySequence boardR1 = drive.trajectorySequenceBuilder(spikeR.end())
                .lineToLinearHeading(new Pose2d(27, -32, Math.toRadians(-90)))
                .setVelConstraint(Hardware.getVelocityConstraint(0.15 * DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .forward(5)
                .build();
        TrajectorySequence stackR1;
        if(cycle.equals("yes")) {
            stackR1 = drive.trajectorySequenceBuilder(boardR1.end())
                    .back(5)
                    .lineToLinearHeading(new Pose2d(55, -20, Math.toRadians(90)))
                    .lineTo(new Vector2d(59.7, 74))
                    .build();
        }
        else{
            stackR1 = drive.trajectorySequenceBuilder(boardR1.end())
                    .back(5)
                    .strafeRight(-20)
                    .turn(Math.toRadians(180))
                    .back(5)
                    .build();
        }
        TrajectorySequence stackR1b = drive.trajectorySequenceBuilder(stackR1.end())
                .setVelConstraint(Hardware.getVelocityConstraint(0.1 * DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .lineTo(new Vector2d(67.5, 75))
                .build();
        TrajectorySequence boardR2a = drive.trajectorySequenceBuilder(stackR1b.end())
                .lineToLinearHeading(new Pose2d(65, 60, Math.toRadians(-90)))
                .build();
        TrajectorySequence boardR2b = drive.trajectorySequenceBuilder(boardR2a.end())
                .lineTo(new Vector2d(50, -20))
                .build();
        TrajectorySequence boardR2c = drive.trajectorySequenceBuilder(boardR2b.end())
                .lineTo(new Vector2d(31, -33))
                .setVelConstraint(Hardware.getVelocityConstraint(0.15 * DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .forward(5.5)
                .build();
        TrajectorySequence fstR1 = drive.trajectorySequenceBuilder(boardR2c.end())
                .setVelConstraint(Hardware.getVelocityConstraint(0.15 * DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .lineTo(new Vector2d(25, -35.1))
                .build();
        TrajectorySequence stackR2 = drive.trajectorySequenceBuilder(fstR1.end())
                .back(5)
                .lineToLinearHeading(new Pose2d(55, -20, Math.toRadians(84)))
                .lineTo(new Vector2d(63, 72))
                .build();
        TrajectorySequence stackR2b = drive.trajectorySequenceBuilder(stackR2.end())
                .setVelConstraint(Hardware.getVelocityConstraint(0.15 * DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .lineTo(new Vector2d(62.9, 78))
                .build();
        TrajectorySequence boardR3a = drive.trajectorySequenceBuilder(stackR2b.end())
                .lineToLinearHeading(new Pose2d(60, 60, Math.toRadians(-91)))
                .build();
        TrajectorySequence boardR3b = drive.trajectorySequenceBuilder(boardR3a.end())
                .lineTo(new Vector2d(60, -20))
                .build();
        TrajectorySequence boardR3c = drive.trajectorySequenceBuilder(boardR3b.end())
                .lineTo(new Vector2d(32, -33))
                .setVelConstraint(Hardware.getVelocityConstraint(0.15 * DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .forward(7.2)
                .build();
        TrajectorySequence fstR2 = drive.trajectorySequenceBuilder(boardR3b.end())
                .setVelConstraint(Hardware.getVelocityConstraint(0.15 * DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .lineTo(new Vector2d(38, -35.9))
                .build();
        TrajectorySequence parkR = drive.trajectorySequenceBuilder(fstR2.end())
                .back(2)
                .strafeLeft(20)
                .build();


        waitForStart();
        sleep(timeout);
        switch (position) {
            case ("left"):
                drive.followTrajectorySequence(spikeL); // spike mark L
                drive.openL();
                sleep(100);
                drive.wristU();
                drive.slidesTo(1200);

                drive.followTrajectorySequence(boardL1); // board L 1
                drive.openR();
                drive.slidesTo(0, 0.5);
                sleep(100);
                drive.wristD();

                drive.followTrajectorySequence(stackL1); // stack L 1
                if(cycle.equals("yes")) {
                    drive.slidesTo(250);
                    drive.followTrajectorySequence(stackL1b);
                    drive.closeL();
                    drive.closeR();
                    sleep(200);
                    drive.slidesTo(300);

                    drive.followTrajectorySequence(boardL2a); // board L 2
                    drive.slidesTo(0, 0.9);
                    drive.followTrajectorySequence(boardL2b);
                    drive.slidesTo(1500);
                    drive.wristU();
                    drive.followTrajectorySequence(boardL2c);
                    drive.openL();
                    drive.openR();
                    drive.followTrajectorySequence(fstL1);
                    drive.slidesTo(0, 0.5);
                    sleep(80);
                    drive.wristD();

                    drive.followTrajectorySequence(stackL2); // stack L 2
                    drive.slidesTo(130);
                    drive.followTrajectorySequence(stackL2b);
                    drive.closeL();
                    drive.closeR();
                    sleep(200);
                    drive.slidesTo(170);

                    drive.followTrajectorySequence(boardL3a); // board L 3
                    drive.slidesTo(0, 0.9);
                    drive.followTrajectorySequence(boardL3b);
                    drive.wristU();
                    drive.slidesTo(1900);
                    drive.followTrajectorySequence(boardL3c);
                    drive.openL();
                    drive.openR();
                    drive.followTrajectorySequence(fstL2);
                    drive.slidesTo(0);
                    sleep(100);
                    drive.wristD();

                    drive.followTrajectorySequence(parkL); // park L
                }
                break;

            case ("center"):
                drive.followTrajectorySequence(spikeC); // spike mark C
                drive.openL();
                sleep(100);
                drive.wristU();
                drive.slidesTo(1200);

                drive.followTrajectorySequence(boardC1); // board C 1
                drive.openR();
                drive.slidesTo(0, 0.5);
                sleep(100);
                drive.wristD();

                drive.followTrajectorySequence(stackC1); // stack C 1
                if(cycle.equals("yes")) {
                    drive.slidesTo(250);
                    drive.followTrajectorySequence(stackC1b);
                    drive.closeL();
                    drive.closeR();
                    sleep(200);
                    drive.slidesTo(300);

                    drive.followTrajectorySequence(boardC2a); // board C 2
                    drive.slidesTo(0, 0.9);
                    drive.followTrajectorySequence(boardC2b);
                    drive.slidesTo(1500);
                    drive.wristU();
                    drive.followTrajectorySequence(boardC2c);
                    drive.openL();
                    drive.openR();
                    drive.followTrajectorySequence(fstC1);
                    drive.slidesTo(0, 0.5);
                    sleep(80);
                    drive.wristD();

                    drive.followTrajectorySequence(stackC2); // stack C 2
                    drive.slidesTo(130);
                    drive.followTrajectorySequence(stackC2b);
                    drive.closeL();
                    drive.closeR();
                    sleep(200);
                    drive.slidesTo(170);

                    drive.followTrajectorySequence(boardC3a); // board C 3
                    drive.slidesTo(0, 0.9);
                    drive.followTrajectorySequence(boardC3b);
                    drive.wristU();
                    drive.slidesTo(1900);
                    drive.followTrajectorySequence(boardC3c);
                    drive.openL();
                    drive.openR();
                    drive.followTrajectorySequence(fstC2);
                    drive.slidesTo(0);
                    sleep(100);
                    drive.wristD();

                    drive.followTrajectorySequence(parkC); // park C
                }
                break;

            case ("right"):
                drive.followTrajectorySequence(spikeR); // spike mark R
                drive.openL();
                sleep(100);
                drive.wristU();
                drive.slidesTo(1200);

                drive.followTrajectorySequence(boardR1); // board R 1
                drive.openR();
                drive.slidesTo(0, 0.5);
                sleep(100);
                drive.wristD();

                drive.followTrajectorySequence(stackR1); // stack R 1
                if (cycle.equals("yes")) {
                    drive.slidesTo(250);
                    drive.followTrajectorySequence(stackR1b);
                    drive.closeL();
                    drive.closeR();
                    sleep(200);
                    drive.slidesTo(300);

                    drive.followTrajectorySequence(boardR2a); // board R 2
                    drive.slidesTo(0, 0.9);
                    drive.followTrajectorySequence(boardR2b);
                    drive.slidesTo(1500);
                    drive.wristU();
                    drive.followTrajectorySequence(boardR2c);
                    drive.openL();
                    drive.openR();
                    drive.followTrajectorySequence(fstR1);
                    drive.slidesTo(0, 0.5);
                    sleep(80);
                    drive.wristD();

                    drive.followTrajectorySequence(stackR2); // stack R 2
                    drive.slidesTo(130);
                    drive.followTrajectorySequence(stackR2b);
                    drive.closeL();
                    drive.closeR();
                    sleep(200);
                    drive.slidesTo(170);

                    drive.followTrajectorySequence(boardR3a); // board R 3
                    drive.slidesTo(0, 0.9);
                    drive.followTrajectorySequence(boardR3b);
                    drive.wristU();
                    drive.slidesTo(1900);
                    drive.followTrajectorySequence(boardR3c);
                    drive.openL();
                    drive.openR();
                    drive.followTrajectorySequence(fstR2);
                    drive.slidesTo(0);
                    sleep(100);
                    drive.wristD();

                    drive.followTrajectorySequence(parkR); // park R
                }
                break;
        }


    }
}