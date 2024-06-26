package org.firstinspires.ftc.teamcode.pandara506.mainPrograms;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.kinematics.MecanumKinematics;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

//import org.firstinspires.ftc.teamcode.pandara506.MecanumSubsystem;
//import org.firstinspires.ftc.teamcode.pandara506.MecanumSubsystem;
import org.firstinspires.ftc.teamcode.pandara506.roadrunner.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.pandara506.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.pandara506.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.pandara506.trajectorysequence.TrajectorySequenceRunner;
import org.firstinspires.ftc.teamcode.pandara506.util.LynxModuleUtil;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/*
 * Simple mecanum drive hardware implementation for REV hardware.
 */
@Config
public class Hardware extends MecanumDrive {
    /*public static PIDCoefficients TRANSLATIONAL_PID = new PIDCoefficients(6, 0.5, 0.005);
    public static PIDCoefficients HEADING_PID = new PIDCoefficients(8.5,0.005, 0.005);*/
    public static PIDCoefficients HEADING_PID = new PIDCoefficients(14, 0.1, 0.01);
    public static PIDCoefficients TRANSLATIONAL_PID = new PIDCoefficients(5.5, 0.04, 0.009);

    public static double LATERAL_MULTIPLIER = 1.7604875;

    public static double VX_WEIGHT = 1;
    public static double VY_WEIGHT = 1;
    public static double OMEGA_WEIGHT = 1;

    private TrajectorySequenceRunner trajectorySequenceRunner;

    private static final TrajectoryVelocityConstraint VEL_CONSTRAINT = getVelocityConstraint(org.firstinspires.ftc.teamcode.pandara506.roadrunner.DriveConstants.MAX_VEL, org.firstinspires.ftc.teamcode.pandara506.roadrunner.DriveConstants.MAX_ANG_VEL, org.firstinspires.ftc.teamcode.pandara506.roadrunner.DriveConstants.TRACK_WIDTH);
    private static final TrajectoryAccelerationConstraint ACCEL_CONSTRAINT = getAccelerationConstraint(org.firstinspires.ftc.teamcode.pandara506.roadrunner.DriveConstants.MAX_ACCEL);

    private TrajectoryFollower follower;

    //wheels
    public DcMotorEx leftFront, leftRear, rightRear, rightFront;
    private List<DcMotorEx> motors;

    //hardware
    public DcMotorEx slide;
    public DcMotorEx hanger;
    public Servo clawLeft;
    public double clawLeftOpenPos = 0.22;
    public double clawLeftClosePos = 0.05;
    public Servo clawRight;
    public double clawRightOpenPos = 0.24;
    public double clawRightClosePos = 0.45;
    public double wristDownPos = 0.27;
    public double wristUpPos = 0.49;
    public int slideP1Pos = 360;
    public int slideP3Pos = 170;

    public Servo wrist;
    public Servo launchPadPivot;
    public Servo initiateLaunch;
    public DigitalChannel touchSensor;
    private IMU imu;
    private VoltageSensor batteryVoltageSensor;

    private List<Integer> lastEncPositions = new ArrayList<>();
    private List<Integer> lastEncVels = new ArrayList<>();
    //public MecanumSubsystem drive;

    public Hardware(HardwareMap hardwareMap) {
        super(org.firstinspires.ftc.teamcode.pandara506.roadrunner.DriveConstants.kV, org.firstinspires.ftc.teamcode.pandara506.roadrunner.DriveConstants.kA, org.firstinspires.ftc.teamcode.pandara506.roadrunner.DriveConstants.kStatic, org.firstinspires.ftc.teamcode.pandara506.roadrunner.DriveConstants.TRACK_WIDTH, org.firstinspires.ftc.teamcode.pandara506.roadrunner.DriveConstants.TRACK_WIDTH, LATERAL_MULTIPLIER);

        follower = new HolonomicPIDVAFollower(TRANSLATIONAL_PID, TRANSLATIONAL_PID, HEADING_PID,
                new Pose2d(0.5, 0.5, Math.toRadians(5.0)), 0.07);

        LynxModuleUtil.ensureMinimumFirmwareVersion(hardwareMap);

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // TODO: adjust the names of the following hardware devices to match your configuration
        leftFront = hardwareMap.get(DcMotorEx.class, "cm2");
        leftRear = hardwareMap.get(DcMotorEx.class, "cm0");
        rightRear = hardwareMap.get(DcMotorEx.class, "em0");
        rightFront = hardwareMap.get(DcMotorEx.class, "em3");

        motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront);

        //drive = new MecanumSubsystem(new SampleMecanumDrive(hardwareMap), true);

        for (DcMotorEx motor : motors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
        }

        if (org.firstinspires.ftc.teamcode.pandara506.roadrunner.DriveConstants.RUN_USING_ENCODER) {
            setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if (org.firstinspires.ftc.teamcode.pandara506.roadrunner.DriveConstants.RUN_USING_ENCODER && org.firstinspires.ftc.teamcode.pandara506.roadrunner.DriveConstants.MOTOR_VELO_PID != null) {
            setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, org.firstinspires.ftc.teamcode.pandara506.roadrunner.DriveConstants.MOTOR_VELO_PID);
        }

        // TODO: reverse any motors using DcMotor.setDirection()


        List<Integer> lastTrackingEncPositions = new ArrayList<>();
        List<Integer> lastTrackingEncVels = new ArrayList<>();
        leftRear.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setDirection(DcMotor.Direction.REVERSE);

        // TODO: if desired, use setLocalizer() to change the localization method
        setLocalizer(new StandardTrackingWheelLocalizer(hardwareMap, lastTrackingEncPositions, lastTrackingEncVels));

        trajectorySequenceRunner = new TrajectorySequenceRunner(
                follower, HEADING_PID, batteryVoltageSensor,
                lastEncPositions, lastEncVels, lastTrackingEncPositions, lastTrackingEncVels
        );


        //other hardware things
        try{
            slide = hardwareMap.get(DcMotorEx.class, "cm1");
            slide.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            slide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            slide.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            slide.setPower(0);
        } catch (Exception p_exception) {
            slide = null;
        }
        try{
            hanger = hardwareMap.get(DcMotorEx.class, "em1");
            hanger.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            hanger.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            hanger.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            hanger.setPower(0);
        } catch (Exception p_exception) {
            hanger = null;
        }

        try{
            clawLeft = hardwareMap.get(Servo.class, "es0");
        } catch (Exception p_exception) {
            clawLeft = null;
        }

        try{
            clawRight = hardwareMap.get(Servo.class, "es2");
        } catch (Exception p_exception) {
            clawRight = null;
        }

        try{
            wrist = hardwareMap.get(Servo.class, "es1");
        } catch (Exception p_exception) {
            wrist = null;
        }
        try{
            launchPadPivot = hardwareMap.get(Servo.class, "es5");
        } catch (Exception p_exception) {
            launchPadPivot = null;
        }
        try{
            initiateLaunch = hardwareMap.get(Servo.class, "es4");
        } catch (Exception p_exception) {
            initiateLaunch = null;
        }

        try{
            touchSensor = hardwareMap.get(DigitalChannel.class, "cd0");
            touchSensor.setMode(DigitalChannel.Mode.INPUT);
        } catch (Exception p_exception){
            touchSensor = null;
        }

    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose) {
        return new TrajectoryBuilder(startPose, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, boolean reversed) {
        return new TrajectoryBuilder(startPose, reversed, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, double startHeading) {
        return new TrajectoryBuilder(startPose, startHeading, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    public TrajectorySequenceBuilder trajectorySequenceBuilder(Pose2d startPose) {
        return new TrajectorySequenceBuilder(
                startPose,
                VEL_CONSTRAINT, ACCEL_CONSTRAINT,
                org.firstinspires.ftc.teamcode.pandara506.roadrunner.DriveConstants.MAX_ANG_VEL, org.firstinspires.ftc.teamcode.pandara506.roadrunner.DriveConstants.MAX_ANG_ACCEL
        );
    }

    public void turnAsync(double angle) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(
                trajectorySequenceBuilder(getPoseEstimate())
                        .turn(angle)
                        .build()
        );
    }

    public void turn(double angle) {
        turnAsync(angle);
        waitForIdle();
    }

    public void followTrajectoryAsync(Trajectory trajectory) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(
                trajectorySequenceBuilder(trajectory.start())
                        .addTrajectory(trajectory)
                        .build()
        );
    }
    public void followTrajectory(Trajectory trajectory) {
        followTrajectoryAsync(trajectory);
        waitForIdle();
    }

    public void followTrajectorySequenceAsync(TrajectorySequence trajectorySequence) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(trajectorySequence);
    }

    public void followTrajectorySequence(TrajectorySequence trajectorySequence) {
        followTrajectorySequenceAsync(trajectorySequence);
        waitForIdle();
    }

    public Pose2d getLastError() {
        return trajectorySequenceRunner.getLastPoseError();
    }

    public void update() {
        updatePoseEstimate();
        DriveSignal signal = trajectorySequenceRunner.update(getPoseEstimate(), getPoseVelocity());
        if (signal != null) setDriveSignal(signal);
    }

    public void waitForIdle() {
        while (!Thread.currentThread().isInterrupted() && isBusy())
            update();

    }

    public boolean isBusy() {
        return trajectorySequenceRunner.isBusy();
    }

    public void setMode(DcMotor.RunMode runMode) {
        for (DcMotorEx motor : motors) {
            motor.setMode(runMode);
        }
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(zeroPowerBehavior);
        }
    }

    public void setPIDFCoefficients(DcMotor.RunMode runMode, PIDFCoefficients coefficients) {
        PIDFCoefficients compensatedCoefficients = new PIDFCoefficients(
                coefficients.p, coefficients.i, coefficients.d,
                coefficients.f * 12 / batteryVoltageSensor.getVoltage()
        );

        for (DcMotorEx motor : motors) {
            motor.setPIDFCoefficients(runMode, compensatedCoefficients);
        }
    }

    public void setWeightedDrivePower(Pose2d drivePower) {
        Pose2d vel = drivePower;

        if (Math.abs(drivePower.getX()) + Math.abs(drivePower.getY())
                + Math.abs(drivePower.getHeading()) > 1) {
            // re-normalize the powers according to the weights
            double denom = VX_WEIGHT * Math.abs(drivePower.getX())
                    + VY_WEIGHT * Math.abs(drivePower.getY())
                    + OMEGA_WEIGHT * Math.abs(drivePower.getHeading());

            vel = new Pose2d(
                    VX_WEIGHT * drivePower.getX(),
                    VY_WEIGHT * drivePower.getY(),
                    OMEGA_WEIGHT * drivePower.getHeading()
            ).div(denom);
        }

        setDrivePower(vel);
    }
    @Override
    public void setDrivePower(Pose2d drivePower){
        List<Double> powers = MecanumKinematics.robotToWheelVelocities(
                drivePower,
                1.0,
                1.0,
                1.0
        );
        setMotorPowers(powers.get(0), powers.get(1), powers.get(2), powers.get(3));
    }


    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        lastEncPositions.clear();

        List<Double> wheelPositions = new ArrayList<>();
        for (DcMotorEx motor : motors) {
            int position = motor.getCurrentPosition();
            lastEncPositions.add(position);
            wheelPositions.add(org.firstinspires.ftc.teamcode.pandara506.roadrunner.DriveConstants.encoderTicksToInches(position));
        }
        return wheelPositions;
    }

    @Override
    public List<Double> getWheelVelocities() {
        lastEncVels.clear();

        List<Double> wheelVelocities = new ArrayList<>();
        for (DcMotorEx motor : motors) {
            int vel = (int) motor.getVelocity();
            lastEncVels.add(vel);
            wheelVelocities.add(org.firstinspires.ftc.teamcode.pandara506.roadrunner.DriveConstants.encoderTicksToInches(vel));
        }
        return wheelVelocities;
    }

    @Override
    public void setMotorPowers(double v, double v1, double v2, double v3) {
        leftFront.setPower(v);
        leftRear.setPower(v1);
        rightRear.setPower(v2);
        rightFront.setPower(v3);
    }

    @Override
    public double getRawExternalHeading() {
        return 0.0;
    }

    @Override
    public Double getExternalHeadingVelocity() {
        return 0.0;
    }

    public static TrajectoryVelocityConstraint getVelocityConstraint(double maxVel, double maxAngularVel, double trackWidth) {
        return new MinVelocityConstraint(Arrays.asList(
                new AngularVelocityConstraint(maxAngularVel),
                new MecanumVelocityConstraint(maxVel, trackWidth)
        ));
    }

    public static TrajectoryAccelerationConstraint getAccelerationConstraint(double maxAccel) {
        return new ProfileAccelerationConstraint(maxAccel);
    }

    public void closeR() {clawRight.setPosition(clawRightClosePos);}
    public void  openR() {clawRight.setPosition(clawRightOpenPos );}
    public void closeL() {clawLeft .setPosition(clawLeftClosePos );}
    public void  openL() {clawLeft .setPosition(clawLeftOpenPos  );}
    public void wristU() {wrist    .setPosition(wristUpPos       );}
    public void wristD() {wrist    .setPosition(wristDownPos     );}
    public void slidesTo(int slidePos, double power){
        slide.setTargetPosition(slidePos);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide.setPower(power);
    }
    public void slidesTo(int slidePos){
        slidesTo(slidePos, 1.0);
    }
}
