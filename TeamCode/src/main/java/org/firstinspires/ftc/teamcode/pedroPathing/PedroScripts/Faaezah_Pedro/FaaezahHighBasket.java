package org.firstinspires.ftc.teamcode.pedroPathing.PedroScripts.Faaezah_Pedro;

import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;



public class FaaezahHighBasket {
    /* Declare OpMode members. */
    private LinearOpMode myOpMode = null;   // gain access to methods in the calling OpMode.

    // Define Motor and Servo objects
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor armMotor = null;
    private Servo leftHand = null;
    private Servo rightHand = null;

    // Define Drive constants
    public static final double MID_SERVO = 0.5;
    public static final double HAND_SPEED = 0.02;  // sets rate to move servo
    public static final double ARM_UP_POWER = 0.45;
    public static final double ARM_DOWN_POWER = -0.45;

    // Define path builder
    public static PathBuilder builder = new PathBuilder();

    // Define all path chains
    public static PathChain line1 = builder
            .addPath(
                    new BezierLine(
                            new Point(12.464, 105.496, Point.CARTESIAN),
                            new Point(12.464, 132.204, Point.CARTESIAN)
                    )
            )
            .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
            .build();

    public static PathChain line2 = builder
            .addPath(
                    new BezierLine(
                            new Point(12.464, 132.204, Point.CARTESIAN),
                            new Point(40.507, 121.966, Point.CARTESIAN)
                    )
            )
            .setTangentHeadingInterpolation()
            .build();

    public static PathChain line3 = builder
            .addPath(
                    new BezierLine(
                            new Point(40.507, 121.966, Point.CARTESIAN),
                            new Point(12.464, 132.204, Point.CARTESIAN)
                    )
            )
            .setTangentHeadingInterpolation()
            .build();

    public static PathChain line4 = builder
            .addPath(
                    new BezierLine(
                            new Point(12.464, 132.204, Point.CARTESIAN),
                            new Point(40.507, 131.536, Point.CARTESIAN)
                    )
            )
            .setTangentHeadingInterpolation()
            .build();

    public static PathChain line5 = builder
            .addPath(
                    new BezierLine(
                            new Point(40.507, 131.536, Point.CARTESIAN),
                            new Point(12.241, 132.427, Point.CARTESIAN)
                    )
            )
            .setTangentHeadingInterpolation()
            .build();

    public static PathChain line6 = builder
            .addPath(
                    new BezierLine(
                            new Point(12.241, 132.427, Point.CARTESIAN),
                            new Point(39.839, 142.442, Point.CARTESIAN)
                    )
            )
            .setTangentHeadingInterpolation()
            .build();

    public static PathChain line7 = builder
            .addPath(
                    new BezierLine(
                            new Point(39.839, 142.442, Point.CARTESIAN),
                            new Point(11.796, 132.649, Point.CARTESIAN)
                    )
            )
            .setTangentHeadingInterpolation()
            .build();

    public static PathChain line8 = builder
            .addPath(
                    new BezierLine(
                            new Point(11.796, 132.649, Point.CARTESIAN),
                            new Point(61.873, 96.148, Point.CARTESIAN)
                    )
            )
            .setTangentHeadingInterpolation()
            .build();

    // Constructor
    public FaaezahHighBasket(LinearOpMode opmode) {
        myOpMode = opmode;
    }

    /**
     * Initialize all the robot's hardware.
     */
    public void init() {
        // Define and Initialize Motors
        leftDrive = myOpMode.hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = myOpMode.hardwareMap.get(DcMotor.class, "right_drive");
        armMotor = myOpMode.hardwareMap.get(DcMotor.class, "arm");

        // Set motor directions
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);

        // Define and initialize servos
        leftHand = myOpMode.hardwareMap.get(Servo.class, "left_hand");
        rightHand = myOpMode.hardwareMap.get(Servo.class, "right_hand");
        leftHand.setPosition(MID_SERVO);
        rightHand.setPosition(MID_SERVO);

        myOpMode.telemetry.addData(">", "Hardware Initialized");
        myOpMode.telemetry.update();
    }

    /**
     * Drive method combining drive and turn
     */
    public void driveRobot(double Drive, double Turn) {
        double left = Drive + Turn;
        double right = Drive - Turn;

        double max = Math.max(Math.abs(left), Math.abs(right));
        if (max > 1.0) {
            left /= max;
            right /= max;
        }

        setDrivePower(left, right);
    }

    /**
     * Set individual drive powers
     */
    public void setDrivePower(double leftWheel, double rightWheel) {
        leftDrive.setPower(leftWheel);
        rightDrive.setPower(rightWheel);
    }

    /**
     * Set arm power
     */
    public void setArmPower(double power) {
        armMotor.setPower(power);
    }

    /**
     * Move arm up for duration
     */
    public void armUpWithDuration(long duration) {
        setArmPower(ARM_UP_POWER);
        myOpMode.sleep(duration);
        setArmPower(0);
    }

    /**
     * Move arm down for duration
     */
    public void armDownWithDuration(long duration) {
        setArmPower(ARM_DOWN_POWER);
        myOpMode.sleep(duration);
        setArmPower(0);
    }

    /**
     * Set hand positions
     */
    public void setHandPositions(double offset) {
        offset = Range.clip(offset, -0.5, 0.5);
        leftHand.setPosition(MID_SERVO + offset);
        rightHand.setPosition(MID_SERVO - offset);
    }
    /**
     * Execute full path sequence with arm movements
     */
    public void executeFullPathSequence() {
        try {
            // Initial setup
            leftHand.setPosition(MID_SERVO);
            rightHand.setPosition(MID_SERVO);

            // Follow path sequence 1
            driveRobot(0.5, 0);  // Drive forward
            myOpMode.sleep(1500);
            driveRobot(0, 0);    // Stop

            // Move arm and hands for pickup
            leftHand.setPosition(MID_SERVO - 0.5);  // Open hands
            rightHand.setPosition(MID_SERVO + 0.5);
            myOpMode.sleep(500);

            setArmPower(ARM_DOWN_POWER);  // Lower arm
            myOpMode.sleep(1000);
            setArmPower(0);

            leftHand.setPosition(MID_SERVO + 0.5);  // Close hands to grab
            rightHand.setPosition(MID_SERVO - 0.5);
            myOpMode.sleep(500);

            setArmPower(ARM_UP_POWER);    // Raise arm with pixel
            myOpMode.sleep(1000);
            setArmPower(0);

            // Drive to scoring position
            driveRobot(-0.3, 0.2);  // Adjust direction as needed
            myOpMode.sleep(1000);
            driveRobot(0, 0);

            // Score the pixel
            setArmPower(ARM_UP_POWER);
            myOpMode.sleep(500);
            setArmPower(0);

            leftHand.setPosition(MID_SERVO - 0.5);  // Release pixel
            rightHand.setPosition(MID_SERVO + 0.5);
            myOpMode.sleep(500);

            // Return to rest position
            setArmPower(ARM_DOWN_POWER);
            myOpMode.sleep(500);
            setArmPower(0);

            // Reset hand positions
            leftHand.setPosition(MID_SERVO);
            rightHand.setPosition(MID_SERVO);

            myOpMode.telemetry.addData(">", "Path Sequence Complete");
            myOpMode.telemetry.update();

        } catch (Exception e) {
            myOpMode.telemetry.addData("Error", "Path execution failed: " + e.getMessage());
            myOpMode.telemetry.update();
            // Safety stop
            setDrivePower(0, 0);
            setArmPower(0);
        }
    }

}







