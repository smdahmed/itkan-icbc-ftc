package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Encoder Test", group = "Robot")
public class EncoderTest extends LinearOpMode {
    private DcMotor backRight;
    private DcMotor backLeft;

    private DcMotor arm;

    private CRServo robotSampleServo;
    public DcMotor viperKit;

    private final ElapsedTime runtime = new ElapsedTime();

    //Ticks is motor rate * the ratio provided by Gobilda.
    private final double ticks = 537.7;
    //Circumference is based on the wheel size.
    private final double circumference = 301.59;
    private int target;

    @Override
    public void runOpMode() {
        initializeDevices();
        setupEncoders();
        waitForStart();

        goStraight(20);

        //Commented out while we test.
//        turnRight(600);
        //moveArmDown(12, 3);
//        pickupSample();
//        goStraight(250);
//        turnRight(1650);
//        goStraight(1750);
//        turnLeft(450);

        telemetry.addData("Status", "Ended");
    }

    private void initializeDevices() {
        backRight = hardwareMap.get(DcMotor.class,  "right_front_drive");
        backLeft = hardwareMap.get(DcMotor.class, "left_front_drive");
        arm = hardwareMap.get(DcMotor.class, "left_arm");
        robotSampleServo = hardwareMap.get(CRServo.class, "intake");
        viperKit = hardwareMap.get(DcMotor.class, "viper_kit");
        viperKit.setTargetPosition(0);
        viperKit.setPower(0.5);

        backRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    private void setupEncoders() {
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Reset the motor encoder
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Reset the motor encoder
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    private void goStraight(int distance) {
        // Go straight
        target = (int) ((distance * 10 / circumference) * ticks);
        backRight.setTargetPosition(target);
        backLeft.setTargetPosition(target);
        backRight.setPower(1);
        backLeft.setPower(1);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (backRight.isBusy() && backLeft.isBusy()) {
            telemetry.addData("Path", "Going straight: Current position: %s Target Position:%s",
                    backRight.getCurrentPosition(), backRight.getTargetPosition());
            telemetry.update();
        }
        // Reset encoders
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    private void turnRight(int distance) {
        //For some reason the simulator does not accepted casted values.
        target = (int) ((distance * 10 / circumference) * ticks);

        backRight.setTargetPosition(target);
        backLeft.setTargetPosition(target);
        backRight.setPower(-1);
        backLeft.setPower(1);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        telemetry.update();
        while (isNotInPosition(backRight) && isNotInPosition(backLeft)) {
            telemetry.addData("Path", "Turning right: Current position: %s Target Position:%s"
                    , backRight.getCurrentPosition(), backRight.getTargetPosition());
            telemetry.update();
        }
        // Reset encoders
        backRight.setPower(0);
        backLeft.setPower(0);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    private void turnLeft(int distance) {
        //For some reason the simulator does not accepted casted values.
        target = (int) ((distance * 10 / circumference) * ticks);

        backRight.setTargetPosition(target);
        backLeft.setTargetPosition(target);
        backRight.setPower(1);
        backLeft.setPower(-1);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        telemetry.update();
        while (isNotInPosition(backRight) && isNotInPosition(backLeft)) {
            telemetry.addData("Turning left", "Leg %s: Current position: %s Target Position:%s"
                    , backLeft.getCurrentPosition(), backLeft.getTargetPosition());
            telemetry.update();
        }
        // Reset encoders
        backRight.setPower(0);
        backLeft.setPower(0);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    private void moveArmDown(int distance) {
        //For some reason the simulator does not accepted casted values.
        target = (int) ((distance * 10 / circumference) * ticks);

        arm.setTargetPosition(target);
        arm.setPower(0.5);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (runtime.seconds() < distance) {
            telemetry.addData("Path", "Leg %s: Arm movement: Current position: %s Target Position:%s"
                    , runtime.seconds(), target);
            telemetry.update();
        }

        // Reset encoders
        arm.setPower(0);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    private void moveArmUp(int distance) {
        //For some reason the simulator does not accepted casted values.
        target = (int) ((distance * 10 / circumference) * ticks);

        arm.setTargetPosition(target);
        arm.setPower(0.5);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (runtime.seconds() < distance) {
            telemetry.addData("Path", "Leg %s: Arm movement: Current position: %s Target Position:%s"
                    , runtime.seconds(), target);
            telemetry.update();
        }

        // Reset encoders
        arm.setPower(0);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    private void pickupSample() {

        robotSampleServo.setPower(1);

    }

    private void dropSample() {
        robotSampleServo.setPower(-1);
    }

    private void dropSampleInHighBasket() {
        moveArmUp(20);
        viperKit.setTargetPosition(1);
        viperKit.setPower(1);
        while (viperKit.isBusy()) {
            telemetry.addData("Extending viper Arm", "Leg %s: Current position: %s Target Position:%s"
                    , viperKit.getCurrentPosition(), viperKit.getTargetPosition());
            dropSample();
            viperKit.setTargetPosition(0);
            viperKit.setPower(-1);
            while (viperKit.isBusy()) {
                telemetry.addData("Extending viper Arm", "Leg %s: Current position: %s Target Position:%s"
                        , viperKit.getCurrentPosition(), viperKit.getTargetPosition());
            }
        }
        moveArmDown(20);

    }

    private boolean isNotInPosition(DcMotor motor) {
        return motor.getCurrentPosition() < motor.getTargetPosition();
    }
}