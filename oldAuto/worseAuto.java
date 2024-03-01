package org.firstinspires.ftc.teamcode.pandara506.oldAuto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.pandara506.mainPrograms.Hardware;
import org.firstinspires.ftc.teamcode.pandara506.camera.PipelineRedFront;
import org.firstinspires.ftc.teamcode.pandara506.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

/*
 * This is a simple routine to test turning capabilities.
 */
@Autonomous(group = "autoRed")
@Disabled
public class worseAuto extends LinearOpMode {

    OpenCvCamera webCam;
    public PipelineRedFront detector;
    public  String position = "Insert Here";
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    //april tag vars
    final double DESIRED_DISTANCE = 12.0; //  this is how close the camera should get to the target (inches)
    final double SPEED_GAIN  =  0.02  ;   //  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double STRAFE_GAIN =  0.015 ;   //  Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
    final double TURN_GAIN   =  0.01  ;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)
    final double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_STRAFE= 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN  = 0.3;
    private static final boolean USE_WEBCAM = true;  // Set true to use a webcam, or false for a phone camera
    private static final int DESIRED_TAG_ID = -1;     // Choose the tag you want to approach or set to -1 for ANY tag.
    private VisionPortal visionPortal;               // Used to manage the video source.
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    private AprilTagDetection desiredTag = null;

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
                .splineTo(new Vector2d(30, 9), Math.toRadians(90))
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
        Trajectory traj6a = drive.trajectoryBuilder(traj5a.end())
                .back(8)
                .build();

        Trajectory traj1b = drive.trajectoryBuilder(new Pose2d())
                .forward(30)
                .build();
        Trajectory traj2b = drive.trajectoryBuilder(traj1b.end())
                .back(15)
                .build();
        Trajectory traj3b = drive.trajectoryBuilder(traj2b.end())
                .splineTo(new Vector2d(28, -29), Math.toRadians(-90))
                .build();
        Trajectory traj4b = drive.trajectoryBuilder(traj3b.end())
                .forward(7)
                .build();
        Trajectory traj5b = drive.trajectoryBuilder(traj4a.end())
                .back(8)
                .build();
        Trajectory traj6b = drive.trajectoryBuilder(traj5a.end())
                .back(8)
                .build();

        Trajectory traj1c = drive.trajectoryBuilder(new Pose2d())
                .splineTo(new Vector2d(23, -6), Math.toRadians(-20))
                .build();
        Trajectory traj2c = drive.trajectoryBuilder(traj1c.end())
                .back(10)
                .build();
        Trajectory traj3c = drive.trajectoryBuilder(traj2c.end())
                .splineTo(new Vector2d(22, -28), Math.toRadians(-90))
                .build();
        Trajectory traj4c = drive.trajectoryBuilder(traj3c.end())
                .forward(6)
                .build();
        Trajectory traj5c = drive.trajectoryBuilder(traj4a.end())
                .back(8)
                .build();
        Trajectory traj6c = drive.trajectoryBuilder(traj5a.end())
                .back(8)
                .build();

        //Cycle trajectories
        Pose2d end = new Pose2d();
        Trajectory trajCycle1 = drive.trajectoryBuilder(end)
                .splineTo(new Vector2d (56, 20), Math.toRadians(90))
                .build();
        Trajectory trajCycle2 = drive.trajectoryBuilder(trajCycle1.end())
                .splineTo(new Vector2d(46, 78), Math.toRadians(96))
                .build();
        Trajectory trajCycle3 = drive.trajectoryBuilder(trajCycle2.end())
                .back(8)
                .build();
        TrajectorySequence trajCycle3b = drive.trajectorySequenceBuilder(trajCycle3.end())
                .strafeLeft(4)
                .forward(7.26)
                .build();
        TrajectorySequence trajCycle4 = drive.trajectorySequenceBuilder(trajCycle3b.end())
                .back(10)
                .splineTo(new Vector2d(53, -20), Math.toRadians(-90))
                .build();

        Vector2d end2 = new Vector2d(21, -28);
        Trajectory trajCycle5 = drive.trajectoryBuilder(trajCycle4.end())
                .splineTo(end2, Math.toRadians(-90))
                .build();
        Trajectory trajCycle6 = drive.trajectoryBuilder(trajCycle5.end())
                .forward(8)
                .build();
        Trajectory trajCycle7 = drive.trajectoryBuilder(trajCycle6.end())
                .back(8)
                .build();
        TrajectorySequence trajCycle8 = drive.trajectorySequenceBuilder(trajCycle7.end())
                .forward(1)
                .build();

        boolean targetFound     = false;    // Set to true when an AprilTag target is detected
        double  forward           = 0;        // Desired forward power/speed (-1 to +1)
        double  strafe          = 0;        // Desired strafe power/speed (-1 to +1)
        double  turn            = 0;        // Desired turning power/speed (-1 to +1)
        initAprilTag();

        waitForStart();
        switch (position) {
            case "Left":
                drive.followTrajectory(traj1a);
                drive.clawLeft.setPosition(leftOpenPos);
                drive.wrist.setPosition(0.34);
                drive.followTrajectory(traj2a);
                drive.followTrajectory(traj3a);
                drive.slide.setPower(0.9);
                drive.slide.setTargetPosition(1300);
                drive.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                while (opModeIsActive() && drive.slide.isBusy()) {
                }
                drive.followTrajectory(traj4a);
                drive.clawRight.setPosition(rightOpenPos);
                drive.followTrajectory(traj5a);
                drive.slide.setPower(0.7);
                drive.slide.setTargetPosition(320);
                drive.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                while (opModeIsActive() && drive.slide.isBusy()){
                }
                drive.wrist.setPosition(0.169);
                drive.followTrajectory(traj6a);
                end = traj6a.end();
                trajCycle1 = drive.trajectoryBuilder(end)
                        .splineTo(new Vector2d (53, 20), Math.toRadians(90))
                        .build();
                break;
            case "Center":
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
                drive.followTrajectory(traj5b);
                drive.slide.setPower(0.7);
                drive.slide.setTargetPosition(500);
                drive.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                while (opModeIsActive() && drive.slide.isBusy()){
                }
                drive.wrist.setPosition(0.169);
                drive.followTrajectory(traj6b);
                end = traj6b.end();
                trajCycle1 = drive.trajectoryBuilder(end)
                        .splineTo(new Vector2d (53, 20), Math.toRadians(90))
                        .build();
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
                drive.followTrajectory(traj5c);
                drive.slide.setPower(0.7);
                drive.slide.setTargetPosition(500);
                drive.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                while (opModeIsActive() && drive.slide.isBusy()){
                }
                drive.wrist.setPosition(0.169);
                drive.followTrajectory(traj6c);
                end = traj6c.end();
                trajCycle1 = drive.trajectoryBuilder(end)
                        .splineTo(new Vector2d (53, 20), Math.toRadians(90))
                        .build();
                break;
        }
        while(drive.isBusy()){
            //april tags
            targetFound = false;
            desiredTag = null;
            // Step through the list of detected tags and look for a matching tag
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                // Look to see if we have size info on this tag.
                if (detection.metadata != null) {
                    //  Check to see if we want to track towards this tag.
                    if ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID)) {
                        // Yes, we want to use this tag.
                        targetFound = true;
                        desiredTag = detection;
                        break;  // don't look any further.
                    } else {
                        // This tag is in the library, but we do not want to track it right now.
                        telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                    }
                } else {
                    // This tag is NOT in the library, so we don't have enough information to track to it.
                    telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
                }
            }

            // Tell the driver what we see, and what to do.
            if (targetFound) {
                telemetry.addData("\n>", "HOLD Left-Bumper to Drive to Target\n");
                telemetry.addData("Found", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
                telemetry.addData("Range", "%5.1f inches", desiredTag.ftcPose.range);
                telemetry.addData("Bearing", "%3.0f degrees", desiredTag.ftcPose.bearing);
                telemetry.addData("Yaw", "%3.0f degrees", desiredTag.ftcPose.yaw);
                // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
                double  rangeError      = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
                double  headingError    = desiredTag.ftcPose.bearing;
                double  yawError        = desiredTag.ftcPose.yaw;

                // Use the speed and turn "gains" to calculate how we want the robot to move.
                forward  = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                turn   = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN) ;
                strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);
            } else {
                telemetry.addData("\n>", "Drive using joysticks to find valid target\n");
            }
            telemetry.update();

            // Apply desired axes motions to the drivetrain.
            moveRobot(drive, forward, strafe, turn);
            sleep(10);
            break;
        }
        //Cycling
        end = drive.getPoseEstimate();
        trajCycle1 = drive.trajectoryBuilder(end)
                .splineTo(new Vector2d (53, 20), Math.toRadians(90))
                .build();
        drive.followTrajectory(trajCycle1);
        drive.followTrajectory(trajCycle2);
        drive.clawLeft.setPosition(leftClosePos);
        drive.slide.setPower(0.7);
        drive.slide.setTargetPosition(340);
        drive.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (opModeIsActive() && drive.slide.isBusy()){
        }
        drive.followTrajectory(trajCycle3);
        drive.slide.setPower(0.7);
        drive.slide.setTargetPosition(245);
        drive.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (opModeIsActive() && drive.slide.isBusy()){
        }
        drive.followTrajectorySequence(trajCycle3b);
        drive.clawRight.setPosition(rightClosePos);
        sleep(200);
        drive.wrist.setPosition(0.34);
        drive.followTrajectorySequence(trajCycle4);
        if (position.equals("Left")){
                end2 = new Vector2d(21, -28);
                trajCycle5 = drive.trajectoryBuilder(trajCycle4.end())
                        .splineTo(end2, Math.toRadians(-90))
                        .build();
                trajCycle6 = drive.trajectoryBuilder(trajCycle5.end())
                    .forward(8)
                    .build();
        }
        drive.followTrajectory(trajCycle5);
        //cycle slidesd up
        drive.slide.setPower(0.9);
        drive.slide.setTargetPosition(1300);
        drive.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (opModeIsActive() && drive.slide.isBusy()) {
        }
        //forward and drop
        drive.followTrajectory(trajCycle6);
        drive.clawLeft.setPosition(leftOpenPos);
        drive.clawRight.setPosition(rightOpenPos);
        //back and slide down
        drive.slide.setPower(0.7);
        drive.slide.setTargetPosition(1);
        drive.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (opModeIsActive() && drive.slide.isBusy()){
        }
        drive.wrist.setPosition(0.169);
        drive.followTrajectory(trajCycle7);
    }


    //April tag methods
    public void moveRobot(Hardware drive, double x, double y, double yaw) {
        // Calculate wheel powers.
        double leftFrontPower    =  x -y -yaw;
        double rightFrontPower   =  x +y +yaw;
        double leftBackPower     =  x +y -yaw;
        double rightBackPower    =  x -y +yaw;

        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        drive.setMotorPowers(leftFrontPower, leftBackPower, rightFrontPower, rightBackPower);
    }
    private void initAprilTag() {
        // Create the AprilTag processor by using a builder.
        aprilTag = new AprilTagProcessor.Builder().build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTag.setDecimation(2);

        // Create the vision portal by using a builder.
        if (USE_WEBCAM) {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "wc1"))
                    .addProcessor(aprilTag)
                    .build();
        } else {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(BuiltinCameraDirection.BACK)
                    .addProcessor(aprilTag)
                    .build();
        }
    }
    private void    setManualExposure(int exposureMS, int gain) {
        // Wait for the camera to be open, then use the controls

        if (visionPortal == null) {
            return;
        }

        // Make sure camera is streaming before we try to set the exposure controls
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        // Set camera controls unless we are stopping.
        if (!isStopRequested())
        {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);
        }
    }

}