/*package org.firstinspires.ftc.teamcode.JavaProggramming;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

public class Pipeline {
    private Mat workingMatrix = new Mat();

    public String location = "Location 1";

    public double totalA = 0;
    public double totalB = 0;
    public double totalC = 0;
    public double total = 0;

    public static int matArowStart = 100;
    public static int matArowEnd = 400;
    public static int matAcolumnStart = 200;
    public static int matAcolumnEnd = 640;

    public Pipeline(){

    }

    public final Mat processFrame(Mat input){

        input.copyTo(workingMatrix);

        if(workingMatrix.empty()){
            return input;
        }
        Mat matA = workingMatrix.submat(matArowStart, matArowEnd, matAcolumnStart, matAcolumnEnd);

        Imgproc.rectangle(workingMatrix, new Rect(matAcolumnStart, matArowStart, (matArowEnd - matArowStart), (matAcolumnEnd - matArowEnd)), new Scalar(0, 255, 0));

        totalA = Core.sumElems(matA).val[0];
        totalA /= matA.rows * matA.cols;
        totalB = Core.sumElems(matA).val[1];
        totalB /= matA.rows * matA.cols;
        totalC = Core.sumElems(matA).val[2];
        totalC /= matA.rows * matA.cols;

        if(totalA > totalB && totalA > totalC){
            location = "Location 1";
        } else if(totalB > totalA && totalB >totalC) {
            location = "Location 2";
        } else{
            location = "Location 3";
        }

        return workingMatrix;
    }
}
*/