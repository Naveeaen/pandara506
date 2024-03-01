package org.firstinspires.ftc.teamcode.pandara506.camera;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
public class PipelineRedFront extends OpenCvPipeline {

    private Mat workingMatrix = new Mat();
    public String position = "right";

    public static int matCrowStart = 300; // width right
    public static int matCrowEnd = 340; // width left
    public static int matCcolStart = 135; // height bottom
    public static int matCcolEnd = 160; // height top


    public static int matRrowStart = 65; // width
    public static int matRrowEnd = 105;
    public static int matRcolStart = 135; // height
    public static int matRcolEnd = 160;


    public double leftRed = 0;
    public double rightRed = 0;
    public double centerRed = 0;

    public PipelineRedFront() {

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
            position = "center";
        }   else if (rightRed > 0 && rightRed > centerRed) {
            position = "right";
        }   else {
            position = "left";
        }
        return workingMatrix;
    }
}