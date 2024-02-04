package org.firstinspires.ftc.teamcode.pandara506.automus;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pandara506.roadrunner.Hardware;

@Autonomous(name = "bigAuto")
public class bigAutomus extends LinearOpMode {
    public void runOpMode() throws InterruptedException {
        Hardware drive = new Hardware(hardwareMap);
        automusBlueFront autoBF = new automusBlueFront();
        automusBlueBack autoBB = new automusBlueBack();
        automusRedFront autoRF = new automusRedFront();
        automusRedFrontGolden autoRFGF = new automusRedFrontGolden();
        automusRedBackJava autoRB = new automusRedBackJava();

        String color = "";
        String side = "";
        int timeout = 0;

        boolean Cswitch = false;
        boolean Sswitch = false;
        boolean Tswitch = false;

        boolean Rbumped = false;
        boolean Lbumped = false;
        boolean Rtriggered = false;

        while(opModeInInit()) {

            telemetry.addData("Color", color);
            telemetry.addData("Side", side);
            telemetry.addData("Timeout", timeout);
            telemetry.update();

            //color switch (lol)
            if (gamepad1.right_bumper && !Rbumped) {
                if (Cswitch) {
                    color = "Red";
                    Cswitch = false;
                } else {
                    color = "Blue";
                    Cswitch = true;
                }
                Rbumped = true;
            } else if (!gamepad1.right_bumper) {
                Rbumped = false;
            }

            // side switch
            if (gamepad1.left_bumper && !Lbumped) {
                if (Sswitch) {
                    side = "Front";
                    Sswitch = false;
                } else {
                    side = "Back";
                    Sswitch = true;
                }
                Lbumped = true;
            } else if (!gamepad1.left_bumper) {
                Lbumped = false;
            }

            // timeout select
            if (gamepad1.right_trigger != 0 && !Rtriggered) {
                if (Tswitch) {
                    timeout += 500 * gamepad1.right_trigger;
                    Tswitch = false;
                } else {
                    timeout -= 500 * gamepad1.right_trigger;
                    Tswitch = true;
                }
                Rtriggered = true;
            } else if (gamepad1.right_trigger == 0) {
                Rtriggered = false;
            }

            waitForStart();
            autoBF.timeout = timeout;
            autoBB.timeout = timeout;
            autoRF.timeout = timeout;
            autoRB.timeout = timeout;
            autoRFGF.timeout = timeout;

            if(color.equals("Red") && side.equals("Front")) autoRF.runOpMode();
            if(color.equals("Red") && side.equals("Back")) autoRF.runOpMode();
            if(color.equals("Blue") && side.equals("Front")) autoRF.runOpMode();
            if(color.equals("Blue") && side.equals("Back")) autoRF.runOpMode();
            if(side.equals("GoldenF")) autoRFGF.runOpMode();


        }
    }
}
