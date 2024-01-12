package org.firstinspires.ftc.teamcode.old;

public class Hardware {

    //wheels
   /* public DcMotorEx lf, lr, rf, rr;
    public DcMotorEx slide;
    public DcMotorEx hanger;
    public Servo clawRight;
    public Servo clawLeft;
    public Servo wrist;
    public Servo launchPadPivot;
    public Servo initiateLaunch;
    public IMU imu;
    //public RevColorSensorV3 color;

    public static double maxSpeed = 1;

    private static Hardware myInstance = null;

    public static Hardware getInstance() {
        if (myInstance == null) {
            myInstance = new Hardware();
        }
        return myInstance;
    }

    public void init(HardwareMap hwMap) {

        try{
            rf = hwMap.get(DcMotorEx.class, "cm0");
            rf.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            rf.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            rf.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            rf.setPower(0);
        } catch (Exception p_exception) {
            rf = null;
        }

        try{
            lf = hwMap.get(DcMotorEx.class, "em0");
            lf.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            lf.setDirection(DcMotorEx.Direction.REVERSE);
            lf.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            lf.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            lf.setPower(0);
        } catch (Exception p_exception) {
            lf = null;
        }

        try{
            rr = hwMap.get(DcMotorEx.class, "cm1");
            rr.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            rr.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            rr.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            rr.setPower(0);
        } catch (Exception p_exception) {
            rr = null;
        }

        try{
            lr = hwMap.get(DcMotorEx.class, "em1");
            lr.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            lr.setDirection(DcMotorEx.Direction.REVERSE);
            lr.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            lr.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            lr.setPower(0);
        } catch (Exception p_exception) {
            lr = null;
        }

        try{
            slide = hwMap.get(DcMotorEx.class, "em3");
            slide.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            slide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            slide.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            slide.setPower(0);
        } catch (Exception p_exception) {
            slide = null;
        }

        try{
            clawLeft = hwMap.get(Servo.class, "cs1");
        } catch (Exception p_exception) {
            clawLeft = null;
        }

        try{
            clawRight = hwMap.get(Servo.class, "cs2");
        } catch (Exception p_exception) {
            clawRight = null;
        }

        try{
            wrist = hwMap.get(Servo.class, "cs0");
        } catch (Exception p_exception) {
            wrist = null;
        }

        try {
            color = hwMap.get(RevColorSensorV3.class, "Color");
        } catch (Exception p_exception) {
            color = null;
        }


    }

    public void setPower(double  rightFront, double rightBack, double leftFront, double leftBack) {
        if (rf != null) {
            rf.setPower(Range.clip(rightFront, -maxSpeed, maxSpeed));
        }
        if (rr != null) {
            rr.setPower(Range.clip(rightBack, -maxSpeed, maxSpeed));
        }
        if (lf != null) {
            lf.setPower(Range.clip(leftFront, -maxSpeed, maxSpeed));
        }
        if (lr != null) {
            lr.setPower(Range.clip(leftBack, -maxSpeed, maxSpeed));
        }

    }*/


}