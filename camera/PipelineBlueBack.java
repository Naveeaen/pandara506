package org.firstinspires.ftc.teamcode.pandara506.camera;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
public class PipelineBlueBack extends OpenCvPipeline {

    private Mat workingMatrix = new Mat();
    public String position = "right";

    public static int matCrowStart = 270; // width from left
    public static int matCrowEnd = 310; // width to right
    public static int matCcolStart = 325; // height from top
    public static int matCcolEnd = 350; // height to bottom


    public static int matLrowStart = 18; // width
    public static int matLrowEnd = 58;
    public static int matLcolStart = 325; // height
    public static int matLcolEnd = 350;


    public double leftBlue = 0;
    public double centerBlue = 0;

    public PipelineBlueBack() {

    }

    public final Mat processFrame(Mat input) {

        input.copyTo(workingMatrix);

        if (workingMatrix.empty()) {
            return input;
        }

        Mat matR = workingMatrix.submat(matLcolStart, matLcolEnd, matLrowStart, matLrowEnd);
        Imgproc.rectangle(workingMatrix, new Rect(matLrowStart, matLcolStart, (matLrowEnd - matLrowStart), (matLcolEnd - matLcolStart)), new Scalar(0, 0, 0));

        Mat matC = workingMatrix.submat(matCcolStart, matCcolEnd, matCrowStart, matCrowEnd);
        Imgproc.rectangle(workingMatrix, new Rect(matCrowStart, matCcolStart, (matCrowEnd - matCrowStart), (matCcolEnd - matCcolStart)), new Scalar(0, 0, 255));

        leftBlue = Core.sumElems(matR).val[2];
        leftBlue -= Core.sumElems(matR).val[0];
        leftBlue -= Core.sumElems(matR).val[1];
        leftBlue /= matR.rows() * matR.cols();

        centerBlue = Core.sumElems(matC).val[2];
        centerBlue -= Core.sumElems(matC).val[0];
        centerBlue -= Core.sumElems(matC).val[1];
        centerBlue /= matC.rows() * matC.cols();

        if (centerBlue > 0 && centerBlue > leftBlue) {
            position = "center";
        } else if (leftBlue > 0 && leftBlue > centerBlue) {
            position = "left";
        } else {
            position = "right";
        }
        return workingMatrix;
    }
}