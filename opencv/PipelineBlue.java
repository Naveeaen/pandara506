package org.firstinspires.ftc.teamcode;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class PipelineBlue extends OpenCvPipeline {

    private Mat workingMatrix = new Mat();
    public String position = "Insert Here";
    public static int matLrowStart = 20;
    public static int matLrowEnd = 100;
    public static int matLcolStart = 225;

    public static int matLcolEnd = 250;
            ;


    public static int matCrowStart = 270;
    public static int matCrowEnd = 350;
    public static int matCcolStart = 225;

    public static int matCcolEnd = 300;


    public static int matRrowStart = 520;
    public static int matRrowEnd = 600;
    public static int matRcolStart = 225;

    public static int matRcolEnd = 300;



    public double leftBlue = 0;
    public double rightBlue = 0;
    public double centerBlue = 0;

    public PipelineBlue() {

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

                leftBlue = Core.sumElems(matL).val[2];
                leftBlue /= matL.rows() * matL.cols();


                rightBlue = Core.sumElems(matR).val[2];
                rightBlue /= matR.rows() * matR.cols();


                centerBlue = Core.sumElems(matC).val[2];
                centerBlue /= matC.rows() * matC.cols();

            if ((leftBlue > rightBlue) && (leftBlue > centerBlue)) {
                position = "Left";
        }   else if ((rightBlue > leftBlue) && (rightBlue > centerBlue)) {
                position = "Right";
        }   else {
                position = "Center";
        }
        return workingMatrix;
    }
}
