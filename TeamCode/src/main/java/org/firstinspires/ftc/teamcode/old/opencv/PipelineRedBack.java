package org.firstinspires.ftc.teamcode.old.opencv;

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
        }   else if ((rightRed > leftRed) && (rightRed > centerRed)) {
            position = "Right";
        }   else {
            position = "Center";
        }
        return workingMatrix;
    }
}