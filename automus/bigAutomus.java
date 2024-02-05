package org.firstinspires.ftc.teamcode.pandara506.automus;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pandara506.roadrunner.Hardware;

@Autonomous(name = "bigAuto")
public class bigAutomus extends LinearOpMode {
    Hardware drive = new Hardware(hardwareMap);
    automusBlueFront autoBF = new automusBlueFront();
    automusBlueBack autoBB = new automusBlueBack();
    automusRedFront autoRF = new automusRedFront();
    automusRedFrontGolden autoRFGF = new automusRedFrontGolden();
    automusRedBackJava autoRB = new automusRedBackJava();

    String color = "";
    String side = "";
    int timeout = 0;
    String cycle = "";
    String park = "";

    boolean Cswitch = false;
    boolean Sswitch = false;
    boolean Cycswitch = false;
    boolean Pswitch = false;

    boolean Rbumped = false;
    boolean Lbumped = false;
    boolean Rtriggered = false;
    boolean Ltriggered = false;
    boolean Bpressed = false;
    boolean Xpressed = false;

    public void runOpMode() throws InterruptedException {

        while(opModeInInit()) {

            telemetry.addData("Color", color);
            telemetry.addData("Side", side);
            telemetry.addData("Timeout", timeout);
            telemetry.addData("Cycling", cycle);
            telemetry.addData("Park", park);
            telemetry.update();

            //color switch (lol)
            if (gamepad1.right_bumper && !Rbumped) {
                if (Sswitch) {
                    side = "Front";
                    Sswitch = false;
                } else {
                    side = "Back";
                    Sswitch = true;
                }
                Rbumped = true;
            } else if (!gamepad1.right_bumper) {
                Rbumped = false;
            }

            // side switch
            if (gamepad1.left_bumper && !Lbumped) {
                if (Cswitch) {
                    color = "Red";
                    Cswitch = false;
                } else {
                    color = "Blue";
                    Cswitch = true;
                }
                Lbumped = true;
            } else if (!gamepad1.left_bumper) {
                Lbumped = false;
            }

            // timeout +
            if (gamepad1.right_trigger != 0 && !Rtriggered) {
                timeout += 100 * gamepad1.right_trigger;
                Rtriggered = true;
            } else if (gamepad1.right_trigger == 0) {
                Rtriggered = false;
            }
            //timeout -
            if (gamepad1.left_trigger != 0 && !Ltriggered) {
                timeout -= 100 * gamepad1.left_trigger;
                Ltriggered = true;
            } else if (gamepad1.left_trigger == 0) {
                Ltriggered = false;
            }

            // cycle switch
            if (gamepad1.b && !Bpressed) {
                if (Cycswitch) {
                    cycle = "yes";
                    Cycswitch = false;
                } else {
                    cycle = "no";
                    Cycswitch = true;
                }
                Bpressed = true;
            } else if (!gamepad1.b) {
                Bpressed = false;
            }

            // side switch
            if (gamepad1.x && !Xpressed) {
                if (Pswitch) {
                    park = "Red";
                    Pswitch = false;
                } else {
                    park = "Blue";
                    Pswitch = true;
                }
                Xpressed = true;
            } else if (!gamepad1.x) {
                Xpressed = false;
            }

            autoBF.timeout = timeout;
            autoBB.timeout = timeout;
            autoRF.timeout = timeout;
            autoRB.timeout = timeout;
            autoRFGF.timeout = timeout;

            autoBF.cycle = cycle;
            autoBB.cycle = cycle;
            autoRF.cycle = cycle;
            autoRB.cycle = cycle;
            autoRFGF.cycle = cycle;

            autoBF.park = park;
            autoBB.park = park;
            autoRF.park = park;
            autoRB.park = park;
            autoRFGF.park = park;


            if(color.equals("Red") && side.equals("Front")) autoRF.runOpMode();
            if(color.equals("Red") && side.equals("Back")) autoRF.runOpMode();
            if(color.equals("Blue") && side.equals("Front")) autoRF.runOpMode();
            if(color.equals("Blue") && side.equals("Back")) autoRF.runOpMode();
            if(side.equals("GoldenF")) autoRFGF.runOpMode();


        }
    }
}
