package org.firstinspires.ftc.teamcode.pandara506.mainPrograms;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pandara506.automus.newAutomusBlueFront;
import org.firstinspires.ftc.teamcode.pandara506.automus.newAutomusRedBack;
import org.firstinspires.ftc.teamcode.pandara506.automus.newAutomusRedFront;
import org.firstinspires.ftc.teamcode.pandara506.mainPrograms.Hardware;
import org.firstinspires.ftc.teamcode.pandara506.automus.newAutomusBlueBack;
import org.firstinspires.ftc.teamcode.pandara506.automus.newAutomusBlueFront;


@Autonomous(name = "bigAutomus")
@Disabled
public class bigAutomus extends LinearOpMode {
    public String color = "";
    public String side = "";
    public int timeout = 0;
    public String cycle = "";
    public String park = "";

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

    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    public void runOpMode(){
        Hardware drive = new Hardware(hardwareMap);

        while(!isStarted() && !isStopRequested()) {

            telemetry.addData("Color", color);
            telemetry.addData("Side", side);
            telemetry.addData("Timeout", timeout);
            telemetry.addData("Cycling", cycle);
            telemetry.addData("Park", park);
            telemetry.update();

            // side switch
            if (gamepad1.right_bumper && !Rbumped) {
                if (Sswitch) {
                    side = "front";
                    Sswitch = false;
                } else {
                    side = "back";
                    Sswitch = true;
                }
                Rbumped = true;
            } else if (!gamepad1.right_bumper) {
                Rbumped = false;
            }

            // color switch (lol)
            if (gamepad1.left_bumper && !Lbumped) {
                if (Cswitch) {
                    color = "red";
                    Cswitch = false;
                } else {
                    color = "blue";
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

            // park switch
            if (gamepad1.x && !Xpressed) {
                if (Pswitch) {
                    park = "left";
                    Pswitch = false;
                } else {
                    park = "right";
                    Pswitch = true;
                }
                Xpressed = true;
            } else if (!gamepad1.x) {
                Xpressed = false;
            }



        }

        waitForStart();

        newAutomusBlueFront autoBF = new newAutomusBlueFront();
        newAutomusBlueBack autoBB = new newAutomusBlueBack();
        newAutomusRedFront autoRF = new newAutomusRedFront();
        newAutomusRedBack autoRB = new newAutomusRedBack();

        //if(color.equals("red" ) && side.equals("front")) autoRF.runOpMode();
        //if(color.equals("red" ) && side.equals("back" )) autoRB.runOpMode();
        //if(color.equals("blue") && side.equals("front")) autoBF.runOpMode();
        //if(color.equals("blue") && side.equals("back" )) autoBB.runOpMode();
    }
}
