package org.firstinspires.ftc.teamcode.DeprecatedEncoder;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@Disabled
@Autonomous(name = "Faaezah Encoder", group = "Robot")
public class FaaezahEncoder extends LinearOpMode {
    private DcMotorEx backRight;
    private DcMotorEx backLeft;

    private DcMotorEx arm;

    private CRServo robotSampleServo;
    public DcMotorEx viperKit;

    private final ElapsedTime runtime = new ElapsedTime();

    //Ticks is motor rate * the ratio provided by Gobilda.
    private final double ticks = 537.7;
    //Circumference is based on the wheel size.
    private final double circumference = 301.59;
    final double ARM_TICKS_PER_DEGREE =
            28 // number of encoder ticks per rotation of the bare motor
                    * 250047.0 / 4913.0 // This is the exact gear ratio of the 50.9:1 Yellow Jacket gearbox
                    * 100.0 / 20.0 // This is the external gear reduction, a 20T pinion gear that drives a 100T hub-mount gear
                    * 1 / 360.0; // we want ticks per degree, not per rotation

    final double ARM_SCORE_SAMPLE_IN_HIGH = 100 * ARM_TICKS_PER_DEGREE;

    private int target;

    @Override
    public void runOpMode() {
        initializeDevices();
        setupEncoders();
        waitForStart();

        turnRight((int) 34.5);
        goStraight(100);
        turnLeft((int) 34.5);
        goBackwards(100);

        telemetry.addData("Status", "Ended");
    }

    private void goStraight(int distance) {
        // Go straight
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        target = (int) ((distance * 10 / circumference) * ticks);
        backRight.setTargetPosition(target);
        backLeft.setTargetPosition(target);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setVelocity(400);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setVelocity(400);
        while (isNotInPosition(backLeft) && isNotInPosition(backRight)) {
            telemetry.addData("Path", "Going straight: Current position: %s Target Position:%s",
                    backRight.getCurrentPosition(), backRight.getTargetPosition());
            telemetry.update();
        }
        // Reset encoders
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    private void goBackwards(int distance) {
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        target = (int) ((distance * 10 / circumference) * ticks);
        backRight.setTargetPosition(-target);
        backLeft.setTargetPosition(-target);
        backRight.setVelocity(-400);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setVelocity(-400);
        while (backRight.isBusy() && backLeft.isBusy()) {
            telemetry.addData("Path", "Going straight: Current position: %s Target Position:%s",
                    backRight.getCurrentPosition(), backRight.getTargetPosition());
            telemetry.update();
        }
        backRight.setVelocity(0);
        backRight.setVelocity(0);

        // Reset encoders
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    private void turnRight(int distance) {
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //For some reason the simulator does not accepted casted values.
        target = (int) ((distance * 10 / circumference) * ticks);
        backRight.setTargetPosition(-target);
        backLeft.setTargetPosition(target);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setVelocity(-500);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setVelocity(500);
        telemetry.update();
        while (isNotInPosition(backRight) && isNotInPosition(backLeft)) {
            telemetry.addData("Path", "Turning right: Current position: %s Target Position:%s"
                    , backRight.getCurrentPosition(), backRight.getTargetPosition());
            telemetry.update();
        }
        backRight.setVelocity(0);
        backLeft.setVelocity(0);

        // Reset encoders
        backRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    private void turnLeft(int distance) {
        //For some reason the simulator does not accepted casted values.
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        target = (int) ((distance * 10 / circumference) * ticks);
        backRight.setTargetPosition(target);
        backLeft.setTargetPosition(-target);
        backRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        backLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backRight.setVelocity(500);
        backLeft.setVelocity(-500);
        telemetry.update();
        while (isNotInPosition(backRight) && isNotInPosition(backLeft)) {
            telemetry.addData("Turning left", "Leg: Current position: %s Target Position:%s"
                    , backLeft.getCurrentPosition(), backLeft.getTargetPosition());
            telemetry.update();
            backRight.setVelocity(0);
            backLeft.setVelocity(0);
        }

        // Reset encoders
        backRight.setVelocity(0);
        backLeft.setVelocity(0);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

    private void moveArmDown(int distance) {
        //For some reason the simulator does not accepted casted values.
        target = (int) ((distance * 10 / circumference) * ticks);

        arm.setTargetPosition(target);
        arm.setPower(0.5);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (isNotInPosition(arm)) {
            telemetry.addData("Path", "Leg %s: Arm movement: Current position: %s Target Position:%s"
                    , runtime.seconds(), target);
            telemetry.update();
        }

        // Reset encoders
        arm.setPower(0);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    private void moveArmUp() {
        arm.setTargetPosition((int) ARM_SCORE_SAMPLE_IN_HIGH);
        arm.setPower(0.5);
        arm.setVelocity(1600);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (arm.isBusy()) {
            telemetry.addData("Path", "Leg %s: Arm movement: Current position: %s Target Position:%s"
                    , runtime.seconds(), ARM_SCORE_SAMPLE_IN_HIGH);
            telemetry.update();
        }
        // Reset encoders
        arm.setPower(0);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }


    private void dropSample() {
        robotSampleServo.setPower(-1);
    }

    private boolean isNotInPosition(DcMotor motor) {
        return motor.getCurrentPosition() < motor.getTargetPosition();
    }

    private void initializeDevices() {
        backRight = hardwareMap.get(DcMotorEx.class, "right_front_drive");
        backLeft = hardwareMap.get(DcMotorEx.class, "left_front_drive");
        arm = hardwareMap.get(DcMotorEx.class, "left_arm");
        robotSampleServo = hardwareMap.get(CRServo.class, "intake");
        viperKit = hardwareMap.get(DcMotorEx.class, "viper_kit");
        viperKit.setTargetPosition(0);
        viperKit.setPower(0.5);

        backRight.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    private void setupEncoders() {
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Reset the motor encoder
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Reset the motor encoder
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
}
