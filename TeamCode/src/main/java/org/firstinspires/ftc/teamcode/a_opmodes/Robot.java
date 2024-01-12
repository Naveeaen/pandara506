package org.firstinspires.ftc.teamcode.a_opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.c_subsystems.MecanumRRSubsystem;
import org.firstinspires.ftc.teamcode.d_roadrunner.drive.MecanumDrive;

/**
 * Represents the robot's hardware components and subsystems.
 */
@Config
public class Robot {

    // Hardware components
    public MotorEx slide, hanger;
    public ServoEx clawLeft, clawRight;
    public ServoEx wrist, launchPadPivot, initiateLaunch;

    // Subsystems
    public MecanumRRSubsystem drive;

    /**
     * Constructor for the Robot class.
     *
     * @param hardwareMap The hardware map for accessing robot components.
     */
    public Robot(HardwareMap hardwareMap) {
        this(hardwareMap, false);
    }

    /**
     * Overloaded constructor for the Robot class.
     *
     * @param hardwareMap The hardware map for accessing robot components.
     * @param isAuto      A flag indicating if the robot is in autonomous mode.
     */
    public Robot(HardwareMap hardwareMap, boolean isAuto) {
        configureRobot(hardwareMap, isAuto);
    }

    /**
     * Configures the robot's hardware components and subsystems.
     *
     * @param hardwareMap The hardware map for accessing robot components.
     * @param isAuto      A flag indicating if the robot is in autonomous mode.
     */
    private void configureRobot(HardwareMap hardwareMap, boolean isAuto) {
        // Initialize motors and servos
        slide = new MotorEx(hardwareMap, "em2", MotorEx.GoBILDA.RPM_312);
        slide.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);
        slide.setRunMode(MotorEx.RunMode.VelocityControl);
        slide.resetEncoder();

        hanger = new MotorEx(hardwareMap, "cm2", MotorEx.GoBILDA.RPM_312);
        hanger.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);
        hanger.setRunMode(MotorEx.RunMode.VelocityControl);
        hanger.resetEncoder();

        clawLeft = new SimpleServo(hardwareMap, "cs1",-180, 180, AngleUnit.DEGREES);
        clawRight = new SimpleServo(hardwareMap, "cs2",-180, 180, AngleUnit.DEGREES);
        wrist = new SimpleServo(hardwareMap, "cs0",-180, 180, AngleUnit.DEGREES);
        launchPadPivot = new SimpleServo(hardwareMap, "es0",-180, 180, AngleUnit.DEGREES);
        initiateLaunch = new SimpleServo(hardwareMap, "es1",-180, 180, AngleUnit.DEGREES);

        // Initialize subsystems
        drive = new MecanumRRSubsystem(new MecanumDrive(hardwareMap), true);

        if (isAuto) {
        // Configure auto-specific components
        }
    }

}
