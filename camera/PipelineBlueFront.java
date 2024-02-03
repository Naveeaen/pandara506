package org.firstinspires.ftc.teamcode.pandara506.camera;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
public class PipelineBlueFront extends OpenCvPipeline {

    private Mat workingMatrix = new Mat();
    public String position = "Right";

    public static int matCrowStart = 305;
    public static int matCrowEnd = 355;
    public static int matCcolStart = 250;
    public static int matCcolEnd = 275;

    public static int matRrowStart = 550;
    public static int matRrowEnd = 610; //610
    public static int matRcolStart = 260;
    public static int matRcolEnd = 285;


    public double leftBlue = 0;
    public double rightBlue = 0;
    public double centerBlue = 0;

    public PipelineBlueFront() {

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

        rightBlue = Core.sumElems(matR).val[2];
        rightBlue -= Core.sumElems(matR).val[0];
        rightBlue -= Core.sumElems(matR).val[1];
        rightBlue /= matR.rows() * matR.cols();

        centerBlue = Core.sumElems(matC).val[2];
        centerBlue -= Core.sumElems(matC).val[0];
        centerBlue -= Core.sumElems(matC).val[1];
        centerBlue /= matC.rows() * matC.cols();

        if (centerBlue > 0 && centerBlue > rightBlue) {
            position = "Center";
        } else if (rightBlue > 0 && rightBlue > centerBlue) {
            position = "Right";
        } else {
            position = "Left";
        }
        return workingMatrix;
    }
}