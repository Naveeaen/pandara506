package org.firstinspires.ftc.teamcode.a_opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.b_commands.MecanumRRCommand;

//@Disabled
@Config
@TeleOp(name = "MainTeleOp", group = ".Drive")
public class MainTeleOp extends CommandOpMode {
    Robot robot;

    @Override
    public void initialize() {
        // Initialize the robot hardware
        robot = new Robot(hardwareMap);

        GamepadEx gamepad1Ex = new GamepadEx(gamepad1);

        // Create a MecanumCommand for driving the robot
        MecanumRRCommand driveCommand = new MecanumRRCommand(robot.drive,
                gamepad1Ex::getLeftY,
                gamepad1Ex::getLeftX,
                gamepad1Ex::getRightX);

        // Register and schedule commands
        gamepad1Ex.readButtons();
        schedule(driveCommand);
    }
}
