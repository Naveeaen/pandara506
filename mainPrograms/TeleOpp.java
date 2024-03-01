package org.firstinspires.ftc.teamcode.pandara506.mainPrograms;

import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "TeleOp")
public class TeleOpp extends LinearOpMode {
    public void runOpMode(){
        Hardware drive = new Hardware(hardwareMap);

        telemetry.addData("status: ", "helloooo");
        telemetry.update();
        double leftOpenPos = drive.clawLeftOpenPos;
        double leftClosePos = drive.clawLeftClosePos;
        double rightOpenPos = drive.clawRightOpenPos;
        double rightClosePos = drive.clawRightClosePos;
        double wristBoard = drive.wristUpPos;
        double wristGround = drive.wristDownPos;

        drive.clawLeft.setPosition(leftOpenPos);
        drive.clawRight.setPosition(rightOpenPos);

        /*if(!drive.touchSensor.getState()){
            drive.slide.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            drive.slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            drive.slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            drive.slide.setPower(0);
        }
        if(drive.touchSensor.getState()){
            drive.slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            drive.slide.setPower(-1);
        }*/

        waitForStart();

        //claw vars
        String clawMode = "";
        boolean leftOpen = true;
        boolean rightOpen = true;
        boolean bothOpen = true;

        //pressing buttons
        boolean pressingy = false;
        boolean pressing = false;
        boolean pressingtriggerR = false;
        boolean pressingtriggerL = false;
        boolean pressingBump = false;
        boolean pressingx = false;
        boolean pressingx2 = false;
        boolean pressingb = false;
        boolean pressinga = false;
        boolean right_y_up = false;
        boolean right_y_down = false;
        boolean pressingDpadD = false;
        boolean pressingDpadU = false;
        boolean pressingstickL = false;
        boolean pressingstickR = false;

        boolean launchLaunched = false;
        double launchPos = 0.3;
        double launchPosSpeed= 0;
        String mode = "slides";
        boolean pressingback = false;

        double wristPos = 0.54;
        int position = 0;
        double posW = drive.wristUpPos;
        String wristState = "down";
        boolean wristIsFree = false;

        //slow vars
        double speed = 1.0;


        while(opModeIsActive()) {
            if (gamepad1.x && !pressingx){
                drive.setWeightedDrivePower(
                        new Pose2d(0, 0, 0)
                );
                pressingx = true;
            } else if(!gamepad1.x){
                drive.setWeightedDrivePower(
                        new Pose2d(
                                -(speed * gamepad1.left_stick_y) * Math.max(Math.abs(1.2 - gamepad1.left_trigger), 0.2),
                                -(speed * gamepad1.left_stick_x) * Math.max(Math.abs(1.5 - gamepad1.left_trigger), 0.2),
                                -(speed * gamepad1.right_stick_x) * Math.max(Math.abs(1.4 - gamepad1.left_trigger), 0.2)
                        )
                );
                /*Pose2d poseEstimate = drive.getPoseEstimate();
                Vector2d input = new Vector2d(-gamepad1.left_stick_y, -gamepad1.left_stick_x).rotated(-poseEstimate.getHeading());

                drive.setWeightedDrivePower(new Pose2d(input.getX(), input.getY(), -gamepad1.right_stick_x));
                */pressingx = false;
            }
            drive.update();

            if(gamepad1.left_bumper)            {clawMode = "left";}
            else if(gamepad1.right_bumper)      {clawMode = "right";}
            else if(gamepad1.right_trigger > 0) {clawMode = "bump";}
            else {clawMode = "none";}

            telemetry.addData("Claw", clawMode);
                 if(leftOpen && !rightOpen)  telemetry.addData("Claw state", "Left open");
            else if(rightOpen) telemetry.addData("Claw state", "Right open");
            else if(bothOpen)  telemetry.addData("Claw state", "Both open");
            else               telemetry.addData("Claw state", "Both closed");

            //biancas claw code (much better (same concept as mine but looks nicer))
            if(clawMode.equals("left") && !pressing){
                if(!leftOpen){
                    bothOpen = false;
                    drive.clawLeft.setPosition(leftOpenPos);
                    leftOpen = true;
                } else{
                    bothOpen = true;
                    drive.clawLeft.setPosition(leftClosePos);
                    leftOpen = false;
                }
                pressing = true;
            } else if(!clawMode.equals("left")){
                pressing = false;
            }

            if(clawMode.equals("right") && !pressingb){
                if(!rightOpen){
                    bothOpen = false;
                    drive.clawRight.setPosition(rightOpenPos);
                    rightOpen = true;
                } else{
                    bothOpen = true;
                    drive.clawRight.setPosition(rightClosePos);
                    rightOpen = false;
                }
                pressingb = true;
            } else if(!clawMode.equals("right")){
                pressingb = false;
            }

            if(clawMode.equals("bump") && !pressingBump){
                if(!bothOpen){
                    if(!leftOpen || !rightOpen) {
                        leftOpen = true;
                        rightOpen = true;
                        drive.clawRight.setPosition(rightOpenPos);
                        drive.clawLeft.setPosition(leftOpenPos);
                        bothOpen = true;
                    } else {
                        leftOpen = false;
                        rightOpen = false;
                        drive.clawRight.setPosition(rightClosePos);
                        drive.clawLeft.setPosition(leftClosePos);
                        bothOpen = false;
                    }
                } else if(bothOpen){
                    if(leftOpen || rightOpen) {
                        leftOpen = false;
                        rightOpen = false;
                        drive.clawRight.setPosition(rightClosePos);
                        drive.clawLeft.setPosition(leftClosePos);
                        bothOpen = false;
                    } else{
                        leftOpen = true;
                        rightOpen = true;
                        drive.clawRight.setPosition(rightOpenPos);
                        drive.clawLeft.setPosition(leftOpenPos);
                        bothOpen = true;
                    }
                }
                pressingBump = true;
            } else if(!clawMode.equals("bump")){
                pressingBump = false;
            }




            //slide to hanger
            if(gamepad2.back && !pressingback){
                if(mode.equals("slides")){
                    mode = "hanger";

                } else if(mode.equals("hanger")){
                    mode = "slides";
                }
                pressingback = true;
            } else if(!gamepad2.back){
                pressingback = false;
            }
            telemetry.addData("(back toggle) Joystick controlling", mode);

            /*if(drive.touchSensor.getState() == true){
                //drive.slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                //drive.slide.setPower(0);
                telemetry.addData("Touch sensor", "untouched");
            } else{
                telemetry.addData("Touch sensor", "touched");
            }*/

            //slides
            if(mode.equals("slides")) {
                drive.slidesTo(position);

                if (drive.touchSensor.getState() == true) {
                    telemetry.addData("Touch sensor", "untouched");
                } else {
                    telemetry.addData("Touch sensor", "touched. slides at " + drive.slide.getCurrentPosition());
                }

                if (drive.slide.getCurrentPosition() > 3900 && gamepad2.left_stick_y < 0) {
                    position = 3200;
                } else if (drive.touchSensor.getState() == false && gamepad2.left_stick_y > 0) {
                    drive.slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    drive.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    position = 0;
                } else if (gamepad2.left_stick_y != 0) {
                    position += 50 * -gamepad2.left_stick_y;
                } else {
                    position = drive.slide.getCurrentPosition();
                }


                //speed multiplier
                if(drive.slide.getCurrentPosition() > 800 && gamepad1.left_trigger == 0){
                    speed = (double)800/drive.slide.getCurrentPosition();
                    telemetry.addData("Speed", speed);
                } else {
                    speed = 1.0;
                }
            }

            telemetry.update();

            //hanger
            if(mode.equals("hanger")){
                drive.slidesTo(10);

                //slide code copy pasted for hanger
                if (drive.hanger.getCurrentPosition() < 0 && gamepad2.left_stick_y > 0) {
                    drive.hanger.setPower(0);
                } else if (drive.hanger.getCurrentPosition() > 4500 && gamepad2.left_stick_y < 0) {
                    drive.hanger.setPower(0);
                } else {
                    drive.hanger.setPower(-gamepad2.left_stick_y);
                }

            }

            //wrist
            if (gamepad2.left_trigger > 0){
                drive.wrist.setPosition(1);
            } else if (gamepad2.right_trigger > 0){
                drive.wrist.setPosition(0);
            } else {
                drive.wrist.setPosition(posW);
            }

            if (drive.slide.getCurrentPosition() > 100 && wristIsFree) {
                wristIsFree = false;
                posW = wristBoard;
                wristState = "up";
            } else if(drive.slide.getCurrentPosition() < 100 && !wristIsFree){
                wristIsFree = true;
                posW = wristGround;
                wristState = "down";
            }

            if(gamepad2.a && !pressinga){
                if(wristState.equals("down")) {
                    posW = wristBoard;
                    wristState = "up";
                } else if(wristState.equals("up")) {
                    posW = wristGround;
                    wristState = "down";
                }
                pressinga = true;
            } else if(!gamepad2.a){
                pressinga = false;
            }

            //launch pos = 0.38
            //"launcher"
            if(gamepad2.b && !pressingx2){
                drive.launchPadPivot.setPosition(0.41);
                pressingx2 = true;
            } else if(!gamepad2.b){
                pressingx2= false;
            }
            drive.launchPadPivot.setPosition(0.41);


            //0.642
            //1
            //launch trigger
            if(gamepad2.y && !pressingy){
                if(!launchLaunched) {
                    drive.initiateLaunch.setPosition(0.5);
                    launchLaunched = true;
                }else{
                    drive.initiateLaunch.setPosition(1);
                    launchLaunched = false;
                }
                pressingy = true;
            } else if(!gamepad2.y){
                pressingy= false;
            }




            /*//forwards/backwards
            if(gamepad1.left_stick_y >= 0){
                robot.setPower(0.5, 0.5, 0.5, 0.5);
            } else{
                robot.setPower(0, 0, 0, 0);
            }
            if(gamepad1.left_stick_y <= 0){
                robot.setPower(-0.5, -0.5, -0.5, -0.5);
            } else{
                robot.setPower(0, 0, 0, 0);
            }

            //srafing
            if(gamepad1.left_stick_x >= 0){
                robot.setPower(-0.5, 0.5, 0.5, -0.5);
            } else{
                robot.setPower(0, 0, 0, 0);
            }
            if(gamepad1.left_stick_x <= 0){
                robot.setPower(0.5, -0.5, -0.5, 0.5);
            } else{
                robot.setPower(0, 0, 0, 0);
            }

            //turning
            if(gamepad1.right_stick_x >= 0){
                robot.setPower(0.5, -0.5, 0.5, -0.5);
            } else{
                robot.setPower(0, 0, 0, 0);
            }
            if(gamepad1.right_stick_x <= 0){
                robot.setPower(-0.5, 0.5, -0.5, 0.5);
            } else{
                robot.setPower(0, 0, 0, 0);
            }*/
        }
    }
}