package org.firstinspires.ftc.teamcode.pandara506.camera;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
public class PipelineRedBack extends OpenCvPipeline {

    private Mat workingMatrix = new Mat();
    public String position = "Right";
    public static int matLrowStart = 60;
    public static int matLrowEnd = 110;
    public static int matLcolStart = 325;
    public static int matLcolEnd = 350
            ;


    public static int matCrowStart = 305;
    public static int matCrowEnd = 355;
    public static int matCcolStart = 325;
    public static int matCcolEnd = 350;


    public static int matRrowStart = 540;
    public static int matRrowEnd = 600; //610
    public static int matRcolStart = 325;

    public static int matRcolEnd = 350;


    public double leftRed = 0;
    public double rightRed = 0;
    public double centerRed = 0;

    public PipelineRedBack() {

    }

    public final Mat processFrame(Mat input) {

        input.copyTo(workingMatrix);

        if (workingMatrix.empty()) {
            return input;
        }

        Mat matR = workingMatrix.submat(matRcolStart, matRcolEnd, matRrowStart, matRrowEnd);
        Imgproc.rectangle(workingMatrix, new Rect(matRrowStart, matRcolStart, (matRrowEnd - matRrowStart), (matRcolEnd - matRcolStart)), new Scalar(0, 0, 0));


        Mat matC = workingMatrix.submat(matCcolStart, matCcolEnd, matCrowStart, matCrowEnd);
        Imgproc.rectangle(workingMatrix, new Rect(matCrowStart, matCcolStart, (matCrowEnd - matCrowStart), (matCcolEnd - matCcolStart)), new Scalar(0, 0, 255));

        rightRed = Core.sumElems(matR).val[0];
        rightRed -= Core.sumElems(matR).val[1];
        rightRed -= Core.sumElems(matR).val[2];
        rightRed /= matR.rows() * matR.cols();

        centerRed = Core.sumElems(matC).val[0];
        centerRed -= Core.sumElems(matC).val[1];
        centerRed -= Core.sumElems(matC).val[2];
        centerRed /= matC.rows() * matC.cols();

        if (centerRed > 0 && centerRed > rightRed) {
            position = "Center";
        }   else if (rightRed > 0 && rightRed > centerRed) {
            position = "Right";
        }   else {
            position = "Left";
        }
        return workingMatrix;
    }
}