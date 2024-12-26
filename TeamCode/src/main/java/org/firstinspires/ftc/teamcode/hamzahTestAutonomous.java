package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Autonomous(name = "Robot: Test Auto (Hamzah)", group = "Robot")
public class hamzahTestAutonomous extends LinearOpMode {
    public DcMotor leftDrive = null;
    public DcMotor rightDrive = null;
    public DcMotor armMotor = null;
    public CRServo intake = null;
    public DcMotor viperKit = null;

    final double ARM_TICKS_PER_DEGREE =
            28 // number of encoder ticks per rotation of the bare motor
                    * 250047.0 / 4913.0 // This is the exact gear ratio of the 50.9:1 Yellow Jacket gearbox
                    * 100.0 / 20.0 // This is the external gear reduction, a 20T pinion gear that drives a 100T hub-mount gear
                    * 1/360.0; // we want ticks per degree, not per rotation
    // Arm or Viper Positions
    final double ARM_COLLAPSED_INTO_ROBOT  = 0;
    final double ARM_SCORE_SAMPLE_IN_HIGH  = 100 * ARM_TICKS_PER_DEGREE;
    final double VIPER_OUT                 = -80 * ARM_TICKS_PER_DEGREE;

    // Intake Positions
    final double INTAKE_COLLECT    = -1.0;
    final double INTAKE_OFF        =  0.0;
    final double INTAKE_DEPOSIT    =  0.5;

    @Override
    public void runOpMode() {
        setupDevices();
        telemetry.addData("Status", "Running");

        // Update the telemetry on the driver station
        telemetry.update();

        drive();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

    }

    public void setupDevices() {
        /* Define and Initialize Motors */
        leftDrive = hardwareMap.get(DcMotor.class, "left_front_drive"); //the left drivetrain motor
        rightDrive = hardwareMap.get(DcMotor.class, "right_front_drive"); //the right drivetrain motor
        armMotor = hardwareMap.get(DcMotor.class, "left_arm"); //the arm motor
        viperKit = hardwareMap.get(DcMotor.class, "viper_kit"); // the viper kit!!!

        /* Most skid-steer/differential drive robots require reversing one motor to drive forward.
        for this robot, we reverse the right motor.*/
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);


        /* Setting zeroPowerBehavior to BRAKE enables a "brake mode". This causes the motor to slow down
        much faster when it is coasting. This creates a much more controllable drivetrain. As the robot
        stops much quicker. */

        // Setup for Left Motor
        leftDrive.setTargetPosition(0);
        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // Setup for Right Motor
        rightDrive.setTargetPosition(0);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Setup for Arm Motor
        // Setting TargetPosition to 40 (because motor tried to collapse in on itself)
        armMotor.setTargetPosition(40);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ((DcMotorEx) armMotor).setVelocity(1600);


        // Setup for Viper Kit
        // Setting TargetPosition to 0, setting runMode to RUN_TO_POSITION. Also asking it to stop and reset encoder
        viperKit.setTargetPosition(0);
        viperKit.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        viperKit.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        viperKit.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ((DcMotorEx) viperKit).setVelocity(2100);


        // fetching servos
        intake = hardwareMap.get(CRServo.class, "intake");

        /* Make sure that the intake is off, and the wrist is folded in. */
        intake.setPower(INTAKE_OFF);

        telemetry.addLine("Robot Ready.");
        telemetry.update();
    }

    public void stopMoving() {
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        sleep(100);
    }

    public void turnRight(double speed) {
        leftDrive.setPower(-speed);
        rightDrive.setPower(speed);
        sleep(1400);
    }

    public void turnLeft(double speed) {
        leftDrive.setPower(speed);
        rightDrive.setPower(-speed);
        sleep(1000);
    }

    public void goStraight(double speed) {
        leftDrive.setPower(speed);
        rightDrive.setPower(speed);
        sleep(400);
    }

    public void extend(){
        viperKit.setTargetPosition((int) VIPER_OUT);
        sleep(500);
    }

    public void retract(){
        viperKit.setTargetPosition(0);
        sleep(500);
    }

    public void intakeOn(){
        intake.setPower(INTAKE_COLLECT);
    }

    public void intakeOff(){
        intake.setPower(INTAKE_OFF);
    }

    public void deposit(){
        intake.setPower(INTAKE_DEPOSIT);
        sleep(200);
    }

    public void scoreInHigh(){
        armMotor.setTargetPosition((int) ARM_SCORE_SAMPLE_IN_HIGH);
        sleep(300);
        extend();
        deposit();
        retract();
        armMotor.setTargetPosition((int) ARM_COLLAPSED_INTO_ROBOT);
    }

    public void drive(){
        waitForStart();
    }

    // TODO -- SET UP PROPER INSTRUCTIONS FOR DRIVE
    // TODO -- CURRENTLY, MOTORS ARE DRIVE-BY-TIME. CHANGE TO ENCODER DRIVE USING DEGREES.
}