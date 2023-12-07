package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

@TeleOp(name = "VisionTesting")
@Config
public class PipelineOne extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();
    Hardware robot = Hardware.getInstance();
    OpenCvCamera webCam;
    public PipelineRed detector;
    private String position = "AssumingUpperTray";

    public static int matLrowStart = 20;
    public static int matLrowEnd = 100;
    public static int matLcolStart = 225;

    public static int matLcolEnd = 300
            ;


    public static int matRrowStart = 270;
    public static int matRrowEnd = 350;
    public static int matRcolStart = 225;

    public static int matRcolEnd = 300;


    public static int matCrowStart = 530;
    public static int matCrowEnd = 610;
    public static int matCcolStart = 225;

    public static int matCcolEnd = 300;

    public void runOpMode() {
        robot.init(hardwareMap);
        //DASHBOARD TELEMETRY INSERT HERE
        telemetry.addData("Status", "Initailized");
        telemetry.update();

        waitForStart();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        detector = new PipelineRed();
        webCam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "wc1"), cameraMonitorViewId);
        webCam.openCameraDevice();
        FtcDashboard.getInstance().startCameraStream(webCam, 0);
        webCam.startStreaming(640, 480, OpenCvCameraRotation.UPSIDE_DOWN);
        webCam.setPipeline(detector);
        while (opModeIsActive()) {
            position = detector.position;
            telemetry.addData("position", position);
            telemetry.addData("Left Red", detector.leftRed);
            telemetry.addData("Right Red", detector.rightRed);
            telemetry.addData("Center Red", detector.centerRed);
            telemetry.update();

            dashboardTelemetry.addData("position", position);
            dashboardTelemetry.addData("Left Red", detector.leftRed);
            dashboardTelemetry.addData("Right Red", detector.rightRed);
            dashboardTelemetry.addData("Center Red", detector.centerRed);
            dashboardTelemetry.update();
        }

    }

    public class Pipeline extends OpenCvPipeline {

        private Mat workingMatrix = new Mat();
        public String position = "Insert Here";

        public double leftRed = 0;
        public double rightRed = 0;
        public double centerRed = 0;

        public Pipeline() {

        }

        public final Mat processFrame(Mat input) {

            input.copyTo(workingMatrix);

            if (workingMatrix.empty()) {
                return input;
            }

            Mat matL = workingMatrix.submat(matLcolStart, matLcolEnd, matLrowStart, matLrowEnd);
            Imgproc.rectangle(workingMatrix, new Rect(matLrowStart, matLcolStart, (matLrowEnd - matLrowStart), (matLcolEnd - matLcolStart)), new Scalar(0, 255, 0));



            Mat matR = workingMatrix.submat(matRcolStart, matRcolEnd, matRrowStart, matRrowEnd);
            Imgproc.rectangle(workingMatrix, new Rect(matRrowStart, matRcolStart, (matRrowEnd - matRrowStart), (matRcolEnd - matRcolStart)), new Scalar(0, 0, 0));


            Mat matC = workingMatrix.submat(matCcolStart, matCcolEnd, matCrowStart, matCrowEnd);
            Imgproc.rectangle(workingMatrix, new Rect(matCrowStart, matCcolStart, (matCrowEnd - matCrowStart), (matCcolEnd - matCcolStart)), new Scalar(0, 0, 255));

            leftRed = Core.sumElems(matL).val[0];
            leftRed /= matL.rows() * matL.cols();

            rightRed = Core.sumElems(matR).val[0];
            rightRed /= matR.rows() * matR.cols();

            centerRed = Core.sumElems(matC).val[0];
            centerRed /= matC.rows() * matC.cols();

            if ((leftRed > rightRed) && (leftRed > centerRed)) {
                position = "Left";
            } else if ((rightRed > leftRed) && (rightRed > centerRed)) {
                position = "Right";
            } else {
                position = "Center";
            }
            return workingMatrix;
        }

    }
}
