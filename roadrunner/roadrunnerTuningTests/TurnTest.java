package org.firstinspires.ftc.teamcode.pandara506.roadrunner.roadrunnerTuningTests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pandara506.roadrunner.Hardware;

/*
 * This is a simple routine to test turning capabilities.
 */
@Config
@Disabled
@Autonomous(group = "drive")
public class TurnTest extends LinearOpMode {
    public static double ANGLE = 180; // deg

    @Override
    public void runOpMode() throws InterruptedException {
        Hardware drive = new Hardware(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;
        drive.turn(Math.toRadians(ANGLE));
        /*for(int i = 0; i < 10; i ++) {

            sleep(1000);
        }*/
    }
}
