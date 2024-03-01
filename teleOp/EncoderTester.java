package org.firstinspires.ftc.teamcode.pandara506.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.pandara506.mainPrograms.Hardware;

//@Disabled
@TeleOp(name = "EncoderTester")

public class EncoderTester extends LinearOpMode {

    public void runOpMode() {
        Hardware drive = new Hardware(hardwareMap);
        waitForStart();

        int position = 0;
        int actualPosition = 0;
        boolean pressinga = false;
        boolean pressingy = false;
        boolean pressingx = false;
        boolean pressingb = false;
        double speed = 0.5;

        while(opModeIsActive()) {
            telemetry.addData("position", position);
            telemetry.addData("actualPosition", actualPosition);
            telemetry.addData("speedInterval", speed);
            telemetry.update();

            actualPosition = drive.hanger.getCurrentPosition();
            drive.hanger.setPower(speed);
            drive.hanger.setTargetPosition(position);
            drive.hanger.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            /*if(gamepad1.x){
                speed += 0.1;
            }
            if(gamepad1.b){
                speed -= 0.1;
            }
            //3800
            if(gamepad1.a && !pressinga) {
                position += 50;
                pressinga = true;
            } else if (!gamepad1.a) {
                pressinga = false;
            }

            if(gamepad1.y && !pressingy) {
                position -= 50;
                pressingy = true;
            } else if (!gamepad1.y) {
                pressingy = false;
            }*/

            if(gamepad1.x && !pressingx){
                drive.setMotorPowers(1, 0, 0, 0);
            } else if(!gamepad1.x){
                pressingx = false;
                drive.setMotorPowers(0, 0, 0, 0);
            }
            if(gamepad1.b && !pressingb){
                drive.setMotorPowers(0, 1, 0, 0);
            } else if(!gamepad1.b){
                pressingb = false;
                drive.setMotorPowers(0, 0, 0, 0);
            }
            if(gamepad1.a && !pressinga) {
                drive.setMotorPowers(0, 0, 1, 0);
            } else if (!gamepad1.a) {
                pressinga = false;
                drive.setMotorPowers(0, 0, 0, 0);
            }

            if(gamepad1.y && !pressingy) {
                drive.setMotorPowers(0, 0, 0, 1);
            } else if (!gamepad1.y) {
                pressingy = false;
                drive.setMotorPowers(0, 0, 0, 0);
            }


        }
    }
}