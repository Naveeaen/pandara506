package org.firstinspires.ftc.teamcode.old;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.old.drive.SampleMecanumDrive;

@Disabled
@TeleOp(name = "ServoTester")

public class ServoTester extends LinearOpMode {

    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        waitForStart();

        double position = 0;
        double actualPosition = 0;
        double position2 = 0;
        double actualPosition2 = 0;
        boolean pressingy = false;
        boolean pressinga = false;
        boolean pressingx = false;
        boolean pressingb = false;

        while(opModeIsActive()) {
            telemetry.addData("position", position);
            telemetry.addData("actualposition", actualPosition);
            telemetry.addData("position2", position2);
            telemetry.addData("actualposition2", actualPosition2);
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

            drive.clawRight.setPosition(position);
            actualPosition = drive.clawRight.getPosition();
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


            drive.clawLeft.setPosition(position2);
            actualPosition2 = drive.clawLeft.getPosition();
            if(gamepad1.x && !pressingx) {
                position2 += .01;
                pressingx = true;
            } else if (!gamepad1.x) {
                pressingx = false;
            }
            if(gamepad1.b && !pressingb) {
                position2 -= .01;
                pressingb = true;
            } else if (!gamepad1.b) {
                pressingb = false;
            }


        }
    }
}