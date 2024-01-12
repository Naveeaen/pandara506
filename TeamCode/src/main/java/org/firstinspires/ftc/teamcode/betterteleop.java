package org.firstinspires.ftc.teamcode.pandara506;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit;
import org.firstinspires.ftc.teamcode.pandara506.MecanumCommand;
import org.firstinspires.ftc.teamcode.pandara506.drive.SampleMecanumDrive;

//@Disabled
@Config
@TeleOp(name = "betterTeleOp")
public class betterteleop extends CommandOpMode {
    SampleMecanumDrive robot;

    @Override
    public void initialize() {
        // Initialize the robot hardware
        robot = new SampleMecanumDrive(hardwareMap);

        // Set up telemetry on both Driver Station and Dashboard
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Set up gamepad controls
        GamepadEx gamepad1Ex = new GamepadEx(gamepad1);

        // Create a MecanumCommand for driving the robot
        MecanumCommand driveCommand = new MecanumCommand(robot.drive, gamepad1Ex::getLeftY, gamepad1Ex::getLeftX, gamepad1Ex::getRightX);

        gamepad1Ex.readButtons();
        schedule(driveCommand.alongWith(new RunCommand(() -> {
        })));
    }
}