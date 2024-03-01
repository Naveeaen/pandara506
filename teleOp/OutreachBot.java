package org.firstinspires.ftc.teamcode.pandara506.teleOp;

import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Disabled
@TeleOp(name = "OutreachBot")
public class OutreachBot extends LinearOpMode {
    public void runOpMode() {
        OutreachHardware drive = new OutreachHardware(hardwareMap);

        telemetry.addData("status: ", "helloooo");
        telemetry.update();

        boolean pressingx = false;
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.x && !pressingx) {
                drive.setMotorPowers(0, 0, 0, 0);
                pressingx = true;
            } else if (!gamepad1.x) {
                drive.setDrivePower(
                        new Pose2d(
                                (4 * gamepad1.left_stick_y / 5) * Math.max(Math.abs(1.38 - gamepad1.left_trigger), 0.2),
                                (4 * gamepad1.left_stick_x / 5) * Math.max(Math.abs(1.6 - gamepad1.left_trigger), 0.2),
                                (4 * gamepad1.right_stick_x / 5) * Math.max(Math.abs(1.4 - gamepad1.left_trigger), 0.2)
                        )
                );
                /*Pose2d poseEstimate = drive.getPoseEstimate();
                Vector2d input = new Vector2d(-gamepad1.left_stick_y, -gamepad1.left_stick_x).rotated(-poseEstimate.getHeading());

                drive.setDrivePower(new Pose2d(input.getX(), input.getY(), -gamepad1.right_stick_x));
                */
                pressingx = false;
            }
        }
    }
}