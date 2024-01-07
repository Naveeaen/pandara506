package org.firstinspires.ftc.teamcode.pandara506;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.pandara506.drive.SampleMecanumDrive;

@TeleOp(name = "TeleOp")
public class TeleOpp extends LinearOpMode {
    public void runOpMode(){
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        telemetry.addData("status: ", "helloooo");
        telemetry.update();


        waitForStart();

        //claw vars
        String clawMode = "";
        boolean leftOpen = true;
        boolean rightOpen = true;
        boolean bothOpen = true;
        double leftOpenPos = 0.2;
        double leftClosePos = 0.07;
        double rightOpenPos = 0.27;
        double rightClosePos = 0.39;

        //pressing buttons
        boolean pressingy = false;
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
        boolean pressingDpadD = false;
        boolean pressingDpadU = false;
        boolean pressingstickL = false;
        boolean pressingstickR = false;

        boolean launchLaunched = false;
        double launchPos = 0.198;
        double launchPosSpeed= 0;
        String mode = "slide";
        boolean pressingback = false;

        double wristPos = 0.54;
        int position = 0;

        //slow vars
        boolean slowDown = false;
        boolean pressingrighttriggerbutton = false;
        boolean pressingyd1 = false;


        while(opModeIsActive()) {
            if (gamepad1.x && !pressingx){
                drive.setWeightedDrivePower(
                        new Pose2d(0, 0, 0)
                );
                pressingx = true;
            } else if(!gamepad1.x){
                drive.setWeightedDrivePower(
                        new Pose2d(
                                -(Math.atan(4 * gamepad1.left_stick_y) / Math.atan(5)) * Math.max(Math.abs(1.38 - gamepad1.left_trigger), 0.2),
                                -(Math.atan(4 * gamepad1.left_stick_x) / Math.atan(5)) * Math.max(Math.abs(1.6 - gamepad1.left_trigger), 0.2),
                                -(Math.atan(4 * gamepad1.right_stick_x) / Math.atan(5)) * Math.max(Math.abs(1.4 - gamepad1.left_trigger), 0.2)
                        )
                );
                pressingx = false;
            }

            drive.update();

            if(gamepad1.left_bumper) {clawMode = "right";}
            else if(gamepad1.right_bumper) {clawMode = "left";}
            else if(gamepad1.right_trigger > 0) {clawMode = "bump";}
            else {clawMode = "none";}

            telemetry.addData("Claw", clawMode);
            if(leftOpen){
                telemetry.addData("Left claw is open", "");
            }
            else if(rightOpen){
                telemetry.addData("Right claw is open", "");
            }
            else if(bothOpen){
                telemetry.addData("Claw is open", "");
            } else{
                telemetry.addData("Claw is closed", "");
            }
            telemetry.update();


            //biancas claw code (much better (same concept as mine but looks nicer))
            if(clawMode.equals("right") && !pressinga){
                if(leftOpen){
                    bothOpen = false;
                    drive.clawLeft.setPosition(leftOpenPos);
                    leftOpen = false;
                } else{
                    bothOpen = true;
                    drive.clawLeft.setPosition(leftClosePos);
                    leftOpen = true;
                }
                pressinga = true;
            } else if(!clawMode.equals("right")){
                pressinga = false;
            }

            if(clawMode.equals("left") && !pressingb){
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
            } else if(!clawMode.equals("left")){
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




            //slide to hanger
            if(gamepad2.back && !pressingback){
                if(mode.equals("slide")){
                    mode = "hang";

                } else if(mode.equals("hang")){
                    mode = "slide";
                }
                pressingback = true;
            } else if(!gamepad2.back){
                pressingback = false;
            }
            telemetry.addData("slide mode", mode);
            telemetry.addData("pressingback", pressingback);
            telemetry.update();

            //slides
            if(mode.equals("slide")) {
                //slide restrictions
                if (drive.slide.getCurrentPosition() < 20 && gamepad2.left_stick_y > 0) {
                    drive.slide.setPower(0);
                } else if (drive.slide.getCurrentPosition() > 3900 && gamepad2.left_stick_y < 0) {
                    drive.slide.setPower(0);
                } else {
                    //drive controlled power
                    drive.slide.setPower(-gamepad2.left_stick_y);
                    /*if(gamepad2.left_stick_y < 0)
                        position += 1;
                    if(gamepad2.left_stick_y > 0)
                        position -= 1;
                    drive.slide.setPower(1);
                    drive.slide.setTargetPosition(position);
                    drive.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);*/
                }
            }


            //hanger
            if(mode.equals("hang")){
                drive.slide.setPower(0.8);
                drive.slide.setTargetPosition(10);
                drive.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                //slide code copy pasted for hanger
                if (drive.hanger.getCurrentPosition() < 10 && gamepad2.left_stick_y > 0) {
                    drive.hanger.setPower(0);
                } else if (drive.hanger.getCurrentPosition() > 4650 && gamepad2.left_stick_y < 0) {
                    drive.hanger.setPower(0);
                } else {
                    drive.hanger.setPower(-gamepad2.left_stick_y);
                }

            }

            //wrist
            if (drive.slide.getCurrentPosition() > 100) {
                drive.wrist.setPosition(0.34);
            } else {
                drive.wrist.setPosition(0.169);
            }



            //"launcher"
            drive.launchPadPivot.setPosition(launchPos);
            launchPos += launchPosSpeed;
            if(gamepad2.right_stick_y < 0 && launchPos > 0.39) {
                launchPosSpeed = 0;
            } else if(gamepad2.right_stick_y > 0 && launchPos < 0.198) {
                launchPosSpeed = 0;
            } else if(gamepad2.right_stick_y < 0){
                launchPosSpeed = .01;
            } else if(gamepad2.right_stick_y > 0){
                launchPosSpeed = -.01;
            } else if(gamepad2.right_stick_y == 0){
                launchPosSpeed = 0;
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