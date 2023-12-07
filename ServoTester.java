package org.firstinspires.ftc.teamcode.pandara506;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

//@Disabled
@TeleOp(name = "ServoTester")

public class ServoTester extends LinearOpMode {

    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        waitForStart();

        double position = 0;
        double actualPosition = 0;
        boolean pressingy = false;
        boolean pressinga = false;

        while(opModeIsActive()) {
            telemetry.addData("position", position);
            telemetry.addData("actualposition", actualPosition);
            telemetry.update();

            drive.launchPadPivot.setPosition(position);
            actualPosition = drive.launchPadPivot.getPosition();
            //left open pos = 0.503
            //left close pos = 0.407
            //right open pos = 0
            //right close pos = 0.183
            //wrist floor = 0.234
            //wrist angled = 0.488
            //launcher open = 0.198
            //launcher max = 0.39
            //trigger init = 0.623
            //trigger pull = 1
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


        }
    }
}