package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class EncoderInterface {
    EncoderTest encoderTest;

    EncoderInterface(EncoderTest encoderTest) {
        this.encoderTest = encoderTest;
    }

    void extendViperKit(int viperPosition) {
        encoderTest.viperKit.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // Set the target position to the position the driver asked for
        encoderTest.viperKit.setTargetPosition((int) (viperPosition * encoderTest.ARM_TICKS_PER_DEGREE));
        // Set the velocity of the motor and use setMode to run
        encoderTest.viperKit.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        encoderTest.viperKit.setPower(1);
        while (encoderTest.viperKit.isBusy()) {
            encoderTest.telemetry.addData("Value", "Current Position: %s", encoderTest.viperKit.getCurrentPosition());
        }
    }

    void goStraight(int distance) {
        int target = (int) ((distance * 10 / encoderTest.circumference) * encoderTest.ticks);
        encoderTest.backRight.setTargetPosition(target);
        encoderTest.backLeft.setTargetPosition(target);
        encoderTest.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        encoderTest.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        encoderTest.backRight.setPower(0.5);
        encoderTest.backLeft.setPower(0.5);
        while (encoderTest.backRight.isBusy() && encoderTest.backLeft.isBusy()) {
            encoderTest.telemetry.addData("Path", "Going straight: Current position: %s Target Position:%s",
                    encoderTest.backRight.getCurrentPosition(), encoderTest.backRight.getTargetPosition());
            encoderTest.telemetry.update();
        }
        // Reset encoders
        encoderTest.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoderTest.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoderTest.backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        encoderTest.backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    void goBackwards(int distance) {
        encoderTest.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoderTest.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int target = (int) ((distance * 10 / encoderTest.circumference) * encoderTest.ticks);
        encoderTest.backRight.setTargetPosition(-target);
        encoderTest.backLeft.setTargetPosition(-target);
        encoderTest.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        encoderTest.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        encoderTest.backRight.setPower(-1);
        encoderTest.backLeft.setPower(-1);
        while (encoderTest.backRight.isBusy() && encoderTest.backLeft.isBusy()) {
            encoderTest.telemetry.addData("Path", "Going backwards: Current position: %s Target Position:%s",
                    encoderTest.backRight.getCurrentPosition(), encoderTest.backRight.getTargetPosition());
            encoderTest.telemetry.update();
        }
        // Reset encoders
        encoderTest.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoderTest.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoderTest.backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        encoderTest.backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    void turnRight(int distance) {
        int target = (int) ((distance * 10 / encoderTest.circumference) * encoderTest.ticks);

        encoderTest.backRight.setTargetPosition(target);
        encoderTest.backLeft.setTargetPosition(target);
        encoderTest.backRight.setPower(1);
        encoderTest.backLeft.setPower(-1);
        encoderTest.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        encoderTest.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        encoderTest.telemetry.update();
        while (isNotInPosition(encoderTest.backRight) && isNotInPosition(encoderTest.backLeft)) {
            encoderTest.telemetry.addData("Path", "Turning right: Current position: %s Target Position:%s"
                    , encoderTest.backRight.getCurrentPosition(), encoderTest.backRight.getTargetPosition());
            encoderTest.telemetry.update();
        }
        // Reset encoders
        encoderTest.backRight.setPower(0);
        encoderTest.backLeft.setPower(0);
        encoderTest.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoderTest.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoderTest.backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        encoderTest.backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    void turnLeft(int distance) {
        //For some reason the simulator does not accepted casted values.
        int target = (int) ((distance * 10 / encoderTest.circumference) * encoderTest.ticks);

        encoderTest.backRight.setTargetPosition(target);
        encoderTest.backLeft.setTargetPosition(target);
        encoderTest.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        encoderTest.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        encoderTest.backRight.setPower(1);
        encoderTest.backLeft.setPower(-1);
        encoderTest.telemetry.update();
        while (isNotInPosition(encoderTest.backRight) && isNotInPosition(encoderTest.backLeft)) {
            encoderTest.telemetry.addData("Turning left", "Leg %s: Current position: %s Target Position:%s"
                    , encoderTest.backLeft.getCurrentPosition(), encoderTest.backLeft.getTargetPosition());
            encoderTest.telemetry.update();
        }
        // Reset encoders
        encoderTest.backRight.setPower(0);
        encoderTest.backLeft.setPower(0);
        encoderTest.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoderTest.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

    void moveArmDown(int distance) {
        //For some reason the simulator does not accepted casted values.
        int target = (int) ((distance * 10 / encoderTest.circumference) * encoderTest.ticks);

        encoderTest.arm.setTargetPosition(-target);
        encoderTest.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        encoderTest.arm.setPower(0.5);
        while (encoderTest.arm.isBusy()) {
            encoderTest.telemetry.addData("Path",
                    "Arm movement down: Current position: %s Target Position:%s"
                    , encoderTest.arm.getCurrentPosition(), encoderTest.arm.getTargetPosition());
        }
        // Reset encoders
        encoderTest.arm.setPower(0);
        encoderTest.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    void moveArmUp() {
        encoderTest.arm.setTargetPosition((int) encoderTest.ARM_SCORE_SAMPLE_IN_HIGH);
        encoderTest.arm.setPower(0.5);
        encoderTest.arm.setPower(1);
        encoderTest.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (encoderTest.arm.isBusy()) {
            encoderTest.telemetry.addData("Path",
                    "Arm movement up: Current position: %s Target Position:%s"
                    , encoderTest.arm.getCurrentPosition(), encoderTest.arm.getTargetPosition());
            encoderTest.telemetry.update();
        }
        // Reset encoders
        encoderTest.arm.setPower(0);
        encoderTest.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    void pickupSample() {
        encoderTest.robotSampleServo.setPower(1);

    }

    void dropSample() {
        encoderTest.robotSampleServo.setPower(-1);
    }

    boolean isNotInPosition(DcMotor motor) {
        return motor.getCurrentPosition() < motor.getTargetPosition();
    }

    void initializeDevices() {
        encoderTest.backRight = encoderTest.hardwareMap.get(DcMotor.class, "right_front_drive");
        encoderTest.backLeft = encoderTest.hardwareMap.get(DcMotor.class, "left_front_drive");
        encoderTest.arm = encoderTest.hardwareMap.get(DcMotor.class, "left_arm");
        encoderTest.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoderTest.arm.setTargetPosition(15);
        encoderTest.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        encoderTest.arm.setPower(1);
        encoderTest.robotSampleServo = encoderTest.hardwareMap.get(CRServo.class, "intake");
        encoderTest.viperKit = encoderTest.hardwareMap.get(DcMotor.class, "viper_kit");
        encoderTest.viperKit.setTargetPosition(0);
        encoderTest.viperKit.setPower(0.5);

        encoderTest.backRight.setDirection(DcMotor.Direction.REVERSE);
        encoderTest.backLeft.setDirection(DcMotor.Direction.FORWARD);
        encoderTest.telemetry.addData("Status", "Initialized");
        encoderTest.telemetry.update();
    }

    void setupEncoders() {
        encoderTest.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Reset the motor encoder
        encoderTest.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Reset the motor encoder
        encoderTest.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoderTest.backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        encoderTest.backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        encoderTest.viperKit.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        encoderTest.arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }



    //Below is Faaezah's code that we will test out before adding it to the new Autonomous.
//    private void goStraight(int distance) {
//        // Go straight
//        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        target = (int) ((distance * 10 / circumference) * ticks);
//        backRight.setTargetPosition(target);
//        backLeft.setTargetPosition(target);
//        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        backRight.setVelocity(400);
//        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        backLeft.setVelocity(400);
//        while (isNotInPosition(backLeft) && isNotInPosition(backRight)) {
//            telemetry.addData("Path", "Going straight: Current position: %s Target Position:%s",
//                    backRight.getCurrentPosition(), backRight.getTargetPosition());
//            telemetry.update();
//        }
//        // Reset encoders
//        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//    }
//
//    private void goBackwards(int distance) {
//        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        target = (int) ((distance * 10 / circumference) * ticks);
//        backRight.setTargetPosition(-target);
//        backLeft.setTargetPosition(-target);
//        backRight.setVelocity(-400);
//        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        backLeft.setVelocity(-400);
//        while (backRight.isBusy() && backLeft.isBusy()) {
//            telemetry.addData("Path", "Going straight: Current position: %s Target Position:%s",
//                    backRight.getCurrentPosition(), backRight.getTargetPosition());
//            telemetry.update();
//        }
//        backRight.setVelocity(0);
//        backRight.setVelocity(0);
//
//        // Reset encoders
//        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//    }
//
//    private void turnRight(int distance) {
//        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        //For some reason the simulator does not accepted casted values.
//        target = (int) ((distance * 10 / circumference) * ticks);
//        backRight.setTargetPosition(-target);
//        backLeft.setTargetPosition(target);
//        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        backRight.setVelocity(-500);
//        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        backLeft.setVelocity(500);
//        telemetry.update();
//        while (isNotInPosition(backRight) && isNotInPosition(backLeft)) {
//            telemetry.addData("Path", "Turning right: Current position: %s Target Position:%s"
//                    , backRight.getCurrentPosition(), backRight.getTargetPosition());
//            telemetry.update();
//        }
//        backRight.setVelocity(0);
//        backLeft.setVelocity(0);
//
//        // Reset encoders
//        backRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//    }
//
//    private void turnLeft(int distance) {
//        //For some reason the simulator does not accepted casted values.
//        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        target = (int) ((distance * 10 / circumference) * ticks);
//        backRight.setTargetPosition(target);
//        backLeft.setTargetPosition(-target);
//        backRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
//
//        backLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
//        backRight.setVelocity(500);
//        backLeft.setVelocity(-500);
//        telemetry.update();
//        while (isNotInPosition(backRight) && isNotInPosition(backLeft)) {
//            telemetry.addData("Turning left", "Leg: Current position: %s Target Position:%s"
//                    , backLeft.getCurrentPosition(), backLeft.getTargetPosition());
//            telemetry.update();
//            backRight.setVelocity(0);
//            backLeft.setVelocity(0);
//        }
//
//        // Reset encoders
//        backRight.setVelocity(0);
//        backLeft.setVelocity(0);
//        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//    }
//
//    private void moveArmDown(int distance) {
//        //For some reason the simulator does not accepted casted values.
//        target = (int) ((distance * 10 / circumference) * ticks);
//
//        arm.setTargetPosition(target);
//        arm.setPower(0.5);
//        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        while (isNotInPosition(arm)) {
//            telemetry.addData("Path", "Leg %s: Arm movement: Current position: %s Target Position:%s"
//                    , runtime.seconds(), target);
//            telemetry.update();
//        }
//
//        // Reset encoders
//        arm.setPower(0);
//        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//    }
//
//    private void moveArmUp() {
//        arm.setTargetPosition((int) ARM_SCORE_SAMPLE_IN_HIGH);
//        arm.setPower(0.5);
//        ((DcMotorEx) arm).setVelocity(1600);
//        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        while (arm.isBusy()) {
//            telemetry.addData("Path", "Leg %s: Arm movement: Current position: %s Target Position:%s"
//                    , runtime.seconds(), ARM_SCORE_SAMPLE_IN_HIGH);
//            telemetry.update();
//        }
//        // Reset encoders
//        arm.setPower(0);
//        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//    }
//
//    private void pickupSample() {
//        robotSampleServo.setPower(1);
//
//    }
//
//    private void dropSample() {
//        robotSampleServo.setPower(-1);
//    }
//
//    private void dropSampleInHighBasket() {
//        moveArmUp();
//        viperKit.setTargetPosition(1);
//        viperKit.setPower(1);
//        while (viperKit.isBusy()) {
//            telemetry.addData("Extending viper Arm", "Leg %s: Current position: %s Target Position:%s"
//                    , viperKit.getCurrentPosition(), viperKit.getTargetPosition());
//            dropSample();
//            viperKit.setTargetPosition(0);
//            viperKit.setPower(-1);
//            while (viperKit.isBusy()) {
//                telemetry.addData("Extending viper Arm", "Leg %s: Current position: %s Target Position:%s"
//                        , viperKit.getCurrentPosition(), viperKit.getTargetPosition());
//            }
//        }
//        moveArmDown(20);
//
//    }
//
//    private boolean isNotInPosition(DcMotor motor) {
//        return motor.getCurrentPosition() < motor.getTargetPosition();
//    }
//
//    private void initializeDevices() {
//        backRight = hardwareMap.get(DcMotorEx.class, "right_front_drive");
//        backLeft = hardwareMap.get(DcMotorEx.class, "left_front_drive");
//        arm = hardwareMap.get(DcMotorEx.class, "left_arm");
//        robotSampleServo = hardwareMap.get(CRServo.class, "intake");
//        viperKit = hardwareMap.get(DcMotorEx.class, "viper_kit");
//        viperKit.setTargetPosition(0);
//        viperKit.setPower(0.5);
//
//        backRight.setDirection(DcMotor.Direction.REVERSE);
//        backLeft.setDirection(DcMotor.Direction.FORWARD);
//        telemetry.addData("Status", "Initialized");
//        telemetry.update();
//    }
//
//    private void setupEncoders() {
//        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Reset the motor encoder
//        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Reset the motor encoder
//        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//    }
}
