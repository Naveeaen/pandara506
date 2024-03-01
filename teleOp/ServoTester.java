package org.firstinspires.ftc.teamcode.pandara506.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pandara506.mainPrograms.Hardware;

//@Disabled
@TeleOp(name = "ServoTester")

public class ServoTester extends LinearOpMode {

    public void runOpMode() {
        Hardware drive = new Hardware(hardwareMap);
        waitForStart();

        double position = 0.2;
        double actualPosition = 0;
        double position2 = 0;
        double actualPosition2 = 0;
        double position3 = 0;
        double actualPosition3 = 0;
        boolean pressingy = false;
        boolean pressinga = false;
        boolean pressingbumpl = false;
        boolean pressingbumpr = false;
        boolean pressingdleft = false;
        boolean pressingdright = false;

        while(opModeIsActive()) {
            telemetry.addData("position", position);
            telemetry.addData("actualposition", actualPosition);
            telemetry.addData("position2", position2);
            telemetry.addData("actualposition2", actualPosition2);
            telemetry.addData("position2", position3);
            telemetry.addData("actualposition2", actualPosition3);
            telemetry.update();

            //left open pos = 0.22
            //left close pos = 0.099999999999
            //right open pos = 0.24
            //right close pos = 0.39
            //wrist floor = 0.169
            //wrist angled = 0.34
            //launcher open = 0.198
            //launcher max = 0.39
            //trigger init = 0.623
            //trigger pull = 1

            //drive.launchPadPivot.setPosition(position);
            actualPosition = drive.wrist.getPosition(); //es0
            drive.wrist.setPosition(position);
            if(gamepad1.a && !pressinga) {
                position += .01;
                pressinga = true;
            } else if (!gamepad1.a) {
                pressinga = false;
            }
            if(gamepad1.y && !pressingy) {
                position -= .01;
                pressingy = true;
            } else if (!gamepad1.y) {
                pressingy = false;
            }


            drive.clawRight.setPosition(position2); // es2
            actualPosition2 = drive.clawRight.getPosition();
            if(gamepad1.left_bumper && !pressingbumpl) {
                position2 += .01;
                pressingbumpl = true;
            } else if (!gamepad1.left_bumper) {
                pressingbumpl = false;
            }
            if(gamepad1.right_bumper && !pressingbumpr) {
                position2 -= .01;
                pressingbumpr = true;
            } else if (!gamepad1.right_bumper) {
                pressingbumpr = false;
            }

            drive.clawLeft.setPosition(position3); // es1
            actualPosition3 = drive.clawLeft.getPosition();
            if(gamepad1.dpad_left && !pressingdleft) {
                position3 += .01;
                pressingdleft = true;
            } else if (!gamepad1.dpad_left) {
                pressingdleft = false;
            }
            if(gamepad1.dpad_right && !pressingdright) {
                position3 -= .01;
                pressingdright = true;
            } else if (!gamepad1.dpad_left) {
                pressingdright = false;
            }


        }
    }
}