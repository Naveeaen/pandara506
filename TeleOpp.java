package org.firstinspires.ftc.teamcode.pandara506;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp (name = "TeleOp")
public class TeleOpp extends LinearOpMode {
    int position = 0;
    public void runOpMode(){
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        telemetry.addData("status: ", "helloooo");
        telemetry.update();

        waitForStart();

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
        boolean pressedx = false;
        int spoolTime = 0;
        boolean pressingDpadD = false;
        boolean pressingDpadU = false;
        int position = 0;
        double wristPos = 0.54;

        while(opModeIsActive()) {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );

            drive.update();

            //claw opening right
            if (gamepad1.right_trigger > 0.4 && !pressingtriggerR && !pressedtriggerR) {
                drive.clawRight.setPosition(0);
                pressingtriggerR = true;
            } else if (gamepad1.right_trigger < 0.4) {
                pressingtriggerR = false;
                pressedtriggerR = true;
            }
            //claw closing right
            if (gamepad1.right_trigger > 0.4 && !pressingtriggerR && pressedtriggerR) {
                drive.clawRight.setPosition(0.183);
                pressingtriggerR = true;
            } else if (gamepad1.right_trigger < 0.4) {
                pressingtriggerR = false;
                pressedtriggerR = false;
            }


            //claw opening left
            if (gamepad1.left_trigger > 0.4 && !pressingtriggerL && !pressedtriggerL) {
                drive.clawLeft.setPosition(0.503);
                pressingtriggerL = true;
            } else if (gamepad1.left_trigger < 0.4) {
                pressingtriggerL = false;
                pressedtriggerL = true;
            }
            //claw closing left
            if (gamepad1.left_trigger > 0.4 && !pressingtriggerL && pressedtriggerL) {
                drive.clawLeft.setPosition(0.407);
                pressingtriggerL = true;
            } else if (gamepad1.left_trigger < 0.4) {
                pressingtriggerL = false;
                pressedtriggerL = false;
            }


            //claw opening bump mode
            if (gamepad1.right_bumper && !pressingBump && !pressedBump) {
                drive.clawLeft.setPosition(0.503);
                drive.clawRight.setPosition(0);
                pressingBump= true;
            } else if (!gamepad1.right_bumper) {
                pressingBump = false;
                pressedBump = true;
            }
            //claw closing bump mode
            if (gamepad1.right_bumper && !pressingBump && pressedBump) {
                drive.clawLeft.setPosition(0.407);
                drive.clawRight.setPosition(0.183);
                pressingBump = true;
            } else if (gamepad1.right_bumper) {
                pressingBump = false;
                pressedBump = false;
            }

            //slides goin up
            if(drive.slide.getCurrentPosition() < 20 && gamepad2.left_stick_y < 0){
                drive.slide.setPower(0);
            } else if(drive.slide.getCurrentPosition() > 3900 && gamepad2.left_stick_y > 0){
                drive.slide.setPower(0);
            } else {
                drive.slide.setPower(gamepad2.left_stick_y);
            }

            //wrist
            if(drive.slide.getCurrentPosition() > 100){
                drive.wrist.setPosition(0.488);
            } else{
                drive.wrist.setPosition(0.234);
            }

            //"launcher"
            

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