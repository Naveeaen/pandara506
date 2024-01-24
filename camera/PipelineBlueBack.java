package org.firstinspires.ftc.teamcode.pandara506.camera;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
public class PipelineBlueBack extends OpenCvPipeline {

    private Mat workingMatrix = new Mat();
    public String position = "Right";

    public static int matCrowStart = 150;
    public static int matCrowEnd = 200;
    public static int matCcolStart = 325;
    public static int matCcolEnd = 350;


    public static int matRrowStart = 370;
    public static int matRrowEnd = 425; //610
    public static int matRcolStart = 325;
    public static int matRcolEnd = 350;


    public double leftBlue = 0;
    public double rightBlue = 0;
    public double centerBlue = 0;

    public PipelineBlueBack() {

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