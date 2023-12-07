package org.firstinspires.ftc.teamcode.pandara506;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.pandara506.Hardware;

//@Disabled
@TeleOp(name = "EncoderTester")

public class EncoderTester extends LinearOpMode {

    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        waitForStart();

        int position = 0;
        int actualPosition = 0;
        boolean pressinga = false;
        boolean pressingy = false;
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

            if(gamepad1.x){
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
            }


        }
    }
}