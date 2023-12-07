package org.firstinspires.ftc.teamcode.pandara506;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.pandara506.drive.SampleMecanumDrive;

@TeleOp (name = "TeleOp")
public class TeleOpp extends LinearOpMode {
    int position = 0;
    public void runOpMode(){
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        telemetry.addData("status: ", "helloooo");
        telemetry.update();


        waitForStart();

        String clawMode = "";
        boolean leftOpen = true;
        boolean rightOpen = true;
        boolean bothOpen = true;
        double leftOpenPos = 0.56;
        double leftClosePos = 0.38;
        double rightOpenPos = 0.27;
        double rightClosePos = 0.39;

        boolean pressingy = false;
        boolean launchLaunched = false;
        boolean pressinga = false;
        boolean pressingtriggerR = false;
        boolean pressedtriggerR = false;
        boolean pressingtriggerL = false;
        boolean pressedtriggerL = false;
        boolean pressingBump = false;
        boolean pressedBump = false;
        boolean presseda = false;
        boolean pressedb = false;
        boolean pressingx = false;
        boolean pressingb = false;
        boolean pressedx = false;
        boolean right_y_up = false;
        boolean right_y_down = false;
        int spoolTime = 0;
        boolean pressingDpadD = false;
        boolean pressingDpadU = false;
        int position = 0;
        double wristPos = 0.54;
        double launchPos = 0.198;
        double launchPosSpeed= 0;
        String mode = "slide";


        while(opModeIsActive()) {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );

            drive.update();

            if(gamepad1.right_bumper) {clawMode = "right";}
            else if(gamepad1.left_bumper) {clawMode = "left";}
            else if(gamepad1.right_trigger > 0) {clawMode = "bump";}
            else {clawMode = "none";}

            telemetry.addData("Claw", clawMode);
            telemetry.addData("Left", leftOpen);
            telemetry.addData("Right", rightOpen);
            telemetry.addData("Both", bothOpen);
            telemetry.update();


            //biancas claw code (much better)
            if(clawMode.equals("left") && !pressingx){
                if(leftOpen){
                    bothOpen = false;
                    drive.clawLeft.setPosition(leftOpenPos);
                    leftOpen = false;
                } else{
                    bothOpen = true;
                    drive.clawLeft.setPosition(leftClosePos);
                    leftOpen = true;
                }
                pressingx = true;
            } else if(!clawMode.equals("left")){
                pressingx = false;
            }

            if(clawMode.equals("right") && !pressingb){
                if(rightOpen){
                    bothOpen = false;
                    drive.clawRight.setPosition(rightOpenPos);
                    rightOpen = false;
                } else{
                    bothOpen = true;
                    drive.clawRight.setPosition(rightClosePos);
                    rightOpen = true;
                }
                pressingb = true;
            } else if(!clawMode.equals("right")){
                pressingb = false;
            }

            if(clawMode.equals("bump") && !pressingBump){
                if(bothOpen){
                    leftOpen = false;
                    rightOpen = false;
                    drive.clawRight.setPosition(rightOpenPos);
                    drive.clawLeft.setPosition(leftOpenPos);
                    bothOpen = false;
                } else{
                    leftOpen = true;
                    rightOpen = true;
                    drive.clawRight.setPosition(rightClosePos);
                    drive.clawLeft.setPosition(leftClosePos);
                    bothOpen = true;
                }
                pressingBump = true;
            } else if(!clawMode.equals("bump")){
                pressingBump = false;
            }

            if(mode.equals("slide")) {
                //slides goin up
                if (drive.slide.getCurrentPosition() < 20 && gamepad2.left_stick_y > 0) {
                    drive.slide.setPower(0);
                } else if (drive.slide.getCurrentPosition() > 3900 && gamepad2.left_stick_y < 0) {
                    drive.slide.setPower(0);
                } else {
                    drive.slide.setPower(-gamepad2.left_stick_y);
                }
            }

            if(gamepad2.left_stick_button) mode = "hang";

            if(mode.equals("hang")){
                drive.slide.setPower(0.8);
                drive.slide.setTargetPosition(10);
                drive.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                //slide code copy pasted for hanger
                if (drive.hanger.getCurrentPosition() < 20 && gamepad2.left_stick_y < 0) {
                    drive.hanger.setPower(0);
                } else if (drive.hanger.getCurrentPosition() > 4650 && gamepad2.left_stick_y > 0) {
                    drive.hanger.setPower(0);
                } else {
                    drive.hanger.setPower(gamepad2.left_stick_y);
                }

            }

            //wrist
            if (drive.slide.getCurrentPosition() > 100) {
                drive.wrist.setPosition(0.488);
            } else {
                drive.wrist.setPosition(0.2);
            }

            //"launcher"
            drive.launchPadPivot.setPosition(launchPos);
            if(gamepad2.right_stick_y < 0 && !right_y_up && launchPos <= 0.39) {
                launchPos -= .01;
                right_y_up = true;
            } else {
                right_y_up = false;
            }

            if(gamepad2.right_stick_y > 0 && !right_y_down && launchPos >= 0.198) {
                launchPos += .01;
                right_y_down = true;
            } else {
                right_y_down = false;
            }

            //launch trigger
            if(gamepad2.y && !pressingy){
                if(!launchLaunched) {
                    drive.initiateLaunch.setPosition(1);
                    launchLaunched = true;
                }else{
                    drive.initiateLaunch.setPosition(0.623);
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