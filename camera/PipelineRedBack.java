package org.firstinspires.ftc.teamcode.pandara506.camera;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
public class PipelineRedBack extends OpenCvPipeline {

    private Mat workingMatrix = new Mat();
    public String position = "right";

    public static int matCrowStart = 120; // width right
    public static int matCrowEnd = 160; // width left
    public static int matCcolStart = 140; // height bottom
    public static int matCcolEnd = 165; // height top


    public static int matLrowStart = 320; // width
    public static int matLrowEnd = 360;
    public static int matLcolStart = 130;

    public static int matLcolEnd = 155;


    public double rightRed = 0;
    public double leftRed = 0;
    public double centerRed = 0;

    public PipelineRedBack() {

    }

    public final Mat processFrame(Mat input) {

        input.copyTo(workingMatrix);

        if (workingMatrix.empty()) {
            return input;
        }

        Mat matL = workingMatrix.submat(matLcolStart, matLcolEnd, matLrowStart, matLrowEnd);
        Imgproc.rectangle(workingMatrix, new Rect(matLrowStart, matLcolStart, (matLrowEnd - matLrowStart), (matLcolEnd - matLcolStart)), new Scalar(0, 0, 0));


        Mat matC = workingMatrix.submat(matCcolStart, matCcolEnd, matCrowStart, matCrowEnd);
        Imgproc.rectangle(workingMatrix, new Rect(matCrowStart, matCcolStart, (matCrowEnd - matCrowStart), (matCcolEnd - matCcolStart)), new Scalar(0, 0, 255));

        leftRed = Core.sumElems(matL).val[0];
        leftRed -= Core.sumElems(matL).val[1];
        leftRed -= Core.sumElems(matL).val[2];
        leftRed /= matL.rows() * matL.cols();

        centerRed = Core.sumElems(matC).val[0];
        centerRed -= Core.sumElems(matC).val[1];
        centerRed -= Core.sumElems(matC).val[2];
        centerRed /= matC.rows() * matC.cols();

        if (centerRed > 0 && centerRed > leftRed) {
            position = "center";
        }   else if (leftRed > 0 && leftRed > centerRed) {
            position = "left";
        }   else {
            position = "right";
        }
        return workingMatrix;
    }
}