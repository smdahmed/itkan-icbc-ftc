package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Faaezah Autonomous")
public class FaaezahAutonomous extends OpMode {
    // Motors
    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftBack;
    private DcMotor rightBack;

    // State machine for autonomous
    private int state = 0;
    private ElapsedTime runtime;

    // Constants for movement
    private final double DRIVE_SPEED = 0.15;
    private final double TURN_SPEED = 0.1;

    @Override
    public void init() {
        // Initialize motors
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");

        // Set motor directions
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.FORWARD);

        // Initialize timer
        runtime = new ElapsedTime();
    }

    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {
        switch (state) {
            case 0: // Drive forward for 1 second
                if (runtime.seconds() < 1.0) {
                    driveForward(DRIVE_SPEED);
                } else {
                    stopRobot();
                    state = 1;
                    runtime.reset();
                }
                break;

            case 1: // Turn right for 0.5 seconds
                if (runtime.seconds() < 0.5) {
                    turnRight(TURN_SPEED);
                } else {
                    stopRobot();
                    state = 2;
                    runtime.reset();
                }
                break;

            case 2: // Drive forward again for 1 second
                if (runtime.seconds() < 1.0) {
                    driveForward(DRIVE_SPEED);
                } else {
                    stopRobot();
                    state = 3;
                }
                break;

            case 3: // End autonomous
                stopRobot();
                break;
        }

        // Update telemetry
        telemetry.addData("State", state);
        telemetry.addData("Runtime", runtime.seconds());
        telemetry.update();
    }

    // Helper methods for movement
    private void driveForward(double power) {
        leftFront.setPower(power);
        rightFront.setPower(power);
        leftBack.setPower(power);
        rightBack.setPower(power);
    }

    private void turnRight(double power) {
        leftFront.setPower(power);
        rightFront.setPower(-power);
        leftBack.setPower(power);
        rightBack.setPower(-power);
    }

    private void stopRobot() {
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
    }
}
