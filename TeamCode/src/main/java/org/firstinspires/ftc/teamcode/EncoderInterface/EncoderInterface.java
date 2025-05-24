package org.firstinspires.ftc.teamcode.EncoderInterface;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.HangSampleAndPark;

public class EncoderInterface {
    HangSampleAndPark hangSampleAndPark;
    final double ARM_TICKS_PER_DEGREE =
            28 // number of encoder ticks per rotation of the bare motor
                    * 250047.0 / 4913.0 // This is the exact gear ratio of the 50.9:1 Yellow Jacket gearbox
                    * 100.0 / 20.0 // This is the external gear reduction, a 20T pinion gear that drives a 100T hub-mount gear
                    * 1/360.0; // we want ticks per degree, not per rotation

    public EncoderInterface(HangSampleAndPark hangSampleAndPark) {
        this.hangSampleAndPark = hangSampleAndPark;
    }

    public void extendViperKit(int viperPosition) {
        hangSampleAndPark.viperKit.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // Set the target position to the position the driver asked for
        hangSampleAndPark.viperKit.setTargetPosition((int) (viperPosition * ARM_TICKS_PER_DEGREE));
        // Set the velocity of the motor and use setMode to run
        hangSampleAndPark.viperKit.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hangSampleAndPark.viperKit.setPower(1);
        while (hangSampleAndPark.viperKit.isBusy()) {
            hangSampleAndPark.telemetry.addData("Value", "Current Position: %s", hangSampleAndPark.viperKit.getCurrentPosition());
        }
    }

    public void goStraight(int distance) {
        int target = (int) ((distance * 10 / hangSampleAndPark.circumference) * hangSampleAndPark.ticks);
        hangSampleAndPark.backRight.setTargetPosition(target);
        hangSampleAndPark.backLeft.setTargetPosition(target);
        hangSampleAndPark.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hangSampleAndPark.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hangSampleAndPark.backRight.setPower(0.5);
        hangSampleAndPark.backLeft.setPower(0.5);
        while (hangSampleAndPark.backRight.isBusy() && hangSampleAndPark.backLeft.isBusy()) {
            hangSampleAndPark.telemetry.addData("Path", "Going straight: Current position: %s Target Position:%s",
                    hangSampleAndPark.backRight.getCurrentPosition(), hangSampleAndPark.backRight.getTargetPosition());
            hangSampleAndPark.telemetry.update();
        }
        // Reset encoders
        hangSampleAndPark.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hangSampleAndPark.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hangSampleAndPark.backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hangSampleAndPark.backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void goBackwards(int distance) {
        hangSampleAndPark.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hangSampleAndPark.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int target = (int) ((distance * 10 / hangSampleAndPark.circumference) * hangSampleAndPark.ticks);
        hangSampleAndPark.backRight.setTargetPosition(target);
        hangSampleAndPark.backLeft.setTargetPosition(target);
        hangSampleAndPark.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hangSampleAndPark.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hangSampleAndPark.backRight.setPower(1);
        hangSampleAndPark.backLeft.setPower(1);
        while (isNotInPosition(hangSampleAndPark.backRight) && isNotInPosition(hangSampleAndPark.backLeft)) {
            hangSampleAndPark.telemetry.addData("Path", "Going backwards: Current position: %s Target Position:%s",
                    hangSampleAndPark.backRight.getCurrentPosition(), hangSampleAndPark.backRight.getTargetPosition());
            hangSampleAndPark.telemetry.update();
        }
        // Reset encoders
        hangSampleAndPark.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hangSampleAndPark.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hangSampleAndPark.backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hangSampleAndPark.backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void turnRight(int distance) {
        int target = (int) ((distance * 10 / hangSampleAndPark.circumference) * hangSampleAndPark.ticks);
        hangSampleAndPark.backRight.setTargetPosition(-target);
        hangSampleAndPark.backLeft.setTargetPosition(target);
        hangSampleAndPark.backRight.setPower(1);
        hangSampleAndPark.backLeft.setPower(-1);
        hangSampleAndPark.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hangSampleAndPark.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hangSampleAndPark.telemetry.update();
        while (-hangSampleAndPark.backRight.getCurrentPosition() < -hangSampleAndPark.backRight.getTargetPosition() && isNotInPosition(hangSampleAndPark.backLeft)) {
            hangSampleAndPark.telemetry.addData("Path", "Turning right: Current position: %s Target Position:%s"
                    , hangSampleAndPark.backRight.getCurrentPosition(), hangSampleAndPark.backRight.getTargetPosition());
            hangSampleAndPark.telemetry.update();
        }
        // Reset encoders
        hangSampleAndPark.backRight.setPower(0);
        hangSampleAndPark.backLeft.setPower(0);
        hangSampleAndPark.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hangSampleAndPark.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hangSampleAndPark.backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hangSampleAndPark.backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void turnLeft(int distance) {
        //For some reason the simulator does not accepted casted values.
        int target = (int) ((distance * 10 / hangSampleAndPark.circumference) * hangSampleAndPark.ticks);

        hangSampleAndPark.backRight.setTargetPosition(target);
        hangSampleAndPark.backLeft.setTargetPosition(-target);
        hangSampleAndPark.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hangSampleAndPark.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hangSampleAndPark.backRight.setPower(1);
        hangSampleAndPark.backLeft.setPower(-1);
        hangSampleAndPark.telemetry.update();
        while (isNotInPosition(hangSampleAndPark.backRight) && isNotNegativeInPosition(hangSampleAndPark.backLeft)) {
            hangSampleAndPark.telemetry.addData("Turning left", "Leg: Current position: %s Target Position:%s"
                    , hangSampleAndPark.backLeft.getCurrentPosition(), hangSampleAndPark.backLeft.getTargetPosition());
            hangSampleAndPark.telemetry.update();
        }
        // Reset encoders
        hangSampleAndPark.backRight.setPower(0);
        hangSampleAndPark.backLeft.setPower(0);
        hangSampleAndPark.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hangSampleAndPark.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

    public void moveArmDown(int distance) {
        //For some reason the simulator does not accepted casted values.
        int target = (int) ((distance * 10 / hangSampleAndPark.circumference) * hangSampleAndPark.ticks);

        hangSampleAndPark.arm.setTargetPosition(-target);
        hangSampleAndPark.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hangSampleAndPark.arm.setPower(0.5);
        while (hangSampleAndPark.arm.isBusy()) {
            hangSampleAndPark.telemetry.addData("Path",
                    "Arm movement down: Current position: %s Target Position:%s"
                    , hangSampleAndPark.arm.getCurrentPosition(), hangSampleAndPark.arm.getTargetPosition());
        }
        // Reset encoders
        hangSampleAndPark.arm.setPower(0);
        hangSampleAndPark.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void moveArmUp() {
        hangSampleAndPark.arm.setTargetPosition((int) HangSampleAndPark.ARM_SCORE_SAMPLE_IN_HIGH);
        hangSampleAndPark.arm.setPower(0.5);
        hangSampleAndPark.arm.setPower(1);
        hangSampleAndPark.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (hangSampleAndPark.arm.isBusy()) {
            hangSampleAndPark.telemetry.addData("Path",
                    "Arm movement up: Current position: %s Target Position:%s"
                    , hangSampleAndPark.arm.getCurrentPosition(), hangSampleAndPark.arm.getTargetPosition());
            hangSampleAndPark.telemetry.update();
        }
        // Reset encoders
        hangSampleAndPark.arm.setPower(0);
        hangSampleAndPark.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    boolean isNotInPosition(DcMotor motor) {
        return motor.getCurrentPosition() < motor.getTargetPosition();
    }

    boolean isNotNegativeInPosition(DcMotor motor) {
        return -motor.getCurrentPosition() < -motor.getTargetPosition();
    }

    public void initializeDevices() {
        hangSampleAndPark.backRight = hangSampleAndPark.hardwareMap.get(DcMotorEx.class, "right_front_drive");
        hangSampleAndPark.backLeft = hangSampleAndPark.hardwareMap.get(DcMotorEx.class, "left_front_drive");
        hangSampleAndPark.arm = hangSampleAndPark.hardwareMap.get(DcMotorEx.class, "left_arm");
        hangSampleAndPark.viperKit = hangSampleAndPark.hardwareMap.get(DcMotorEx.class, "viper_kit");
        hangSampleAndPark.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hangSampleAndPark.arm.setTargetPosition(15);
        hangSampleAndPark.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hangSampleAndPark.arm.setPower(1);
        hangSampleAndPark.robotSampleServo = hangSampleAndPark.hardwareMap.get(CRServo.class, "claw");

        hangSampleAndPark.backRight.setDirection(DcMotor.Direction.REVERSE);
        hangSampleAndPark.backLeft.setDirection(DcMotor.Direction.FORWARD);
        hangSampleAndPark.telemetry.addData("Status", "Initialized");
        hangSampleAndPark.telemetry.update();
    }

    public void setupEncoders() {
        hangSampleAndPark.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Reset the motor encoder
        hangSampleAndPark.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Reset the motor encoder
        hangSampleAndPark.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hangSampleAndPark.backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hangSampleAndPark.backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hangSampleAndPark.arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }


    //Below is Faaezah's code that we will test out before adding it to the new Autonomous.
    void goStraightVelocity(int distance) {
        // Go straight
        hangSampleAndPark.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hangSampleAndPark.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int target = (int) ((distance * 10 / hangSampleAndPark.circumference) * hangSampleAndPark.ticks);
        hangSampleAndPark.backRight.setTargetPosition(target);
        hangSampleAndPark.backLeft.setTargetPosition(target);
        hangSampleAndPark.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hangSampleAndPark.backRight.setVelocity(1000);
        hangSampleAndPark.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hangSampleAndPark.backLeft.setVelocity(1000);
        while (isNotInPosition(hangSampleAndPark.backLeft) && isNotInPosition(hangSampleAndPark.backRight)) {
            hangSampleAndPark.telemetry.addData("Path", "Going straightv: Current position: %s Target Position:%s",
                    hangSampleAndPark.backRight.getCurrentPosition(), hangSampleAndPark.backRight.getTargetPosition());
            hangSampleAndPark.telemetry.update();
        }
        // Reset encoders
        hangSampleAndPark.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hangSampleAndPark.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hangSampleAndPark.backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hangSampleAndPark.backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    //
    void goBackwardsVelocity(int distance) {
        hangSampleAndPark.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hangSampleAndPark.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int target = (int) ((distance * 10 / hangSampleAndPark.circumference) * hangSampleAndPark.ticks);
        hangSampleAndPark.backRight.setTargetPosition(-target);
        hangSampleAndPark.backLeft.setTargetPosition(-target);
        hangSampleAndPark.backRight.setVelocity(-400);
        hangSampleAndPark.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hangSampleAndPark.backLeft.setVelocity(-400);
        while (hangSampleAndPark.backRight.isBusy() && hangSampleAndPark.backLeft.isBusy()) {
            hangSampleAndPark.telemetry.addData("Path", "Going straight: Current position: %s Target Position:%s",
                    hangSampleAndPark.backRight.getCurrentPosition(), hangSampleAndPark.backRight.getTargetPosition());
            hangSampleAndPark.telemetry.update();
        }
        hangSampleAndPark.backRight.setVelocity(0);
        hangSampleAndPark.backRight.setVelocity(0);

        // Reset encoders
        hangSampleAndPark.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hangSampleAndPark.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hangSampleAndPark.backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hangSampleAndPark.backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    //
    void turnRightVelocity(int distance) {
        hangSampleAndPark.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hangSampleAndPark.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //For some reason the simulator does not accepted casted values.
        int target = (int) ((distance * 10 / hangSampleAndPark.circumference) * hangSampleAndPark.ticks);
        hangSampleAndPark.backRight.setTargetPosition(-target);
        hangSampleAndPark.backLeft.setTargetPosition(target);
        hangSampleAndPark.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hangSampleAndPark.backRight.setVelocity(-500);
        hangSampleAndPark.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hangSampleAndPark.backLeft.setVelocity(500);
        hangSampleAndPark.telemetry.update();
        while (-hangSampleAndPark.backRight.getCurrentPosition() < -hangSampleAndPark.backRight.getTargetPosition() && isNotInPosition(hangSampleAndPark.backLeft)) {
            hangSampleAndPark.telemetry.addData("Path", "Turning right: Current position: %s Target Position:%s"
                    , hangSampleAndPark.backRight.getCurrentPosition(), hangSampleAndPark.backRight.getTargetPosition());
            hangSampleAndPark.telemetry.update();
        }
        hangSampleAndPark.backRight.setVelocity(0);
        hangSampleAndPark.backLeft.setVelocity(0);

        // Reset encoders
        hangSampleAndPark.backRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        hangSampleAndPark.backLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        hangSampleAndPark.backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hangSampleAndPark.backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    void turnLeftVelocity(int distance) {
        //For some reason the simulator does not accepted casted values.
        hangSampleAndPark.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hangSampleAndPark.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int target = (int) ((distance * 10 / hangSampleAndPark.circumference) * hangSampleAndPark.ticks);
        hangSampleAndPark.backRight.setTargetPosition(target);
        hangSampleAndPark.backLeft.setTargetPosition(-target);
        hangSampleAndPark.backRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        hangSampleAndPark.backLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        hangSampleAndPark.backRight.setVelocity(500);
        hangSampleAndPark.backLeft.setVelocity(-500);
        hangSampleAndPark.telemetry.update();
        while (isNotInPosition(hangSampleAndPark.backRight) && isNotInPosition(hangSampleAndPark.backLeft)) {
            hangSampleAndPark.telemetry.addData("Turning left", "Leg: Current position: %s Target Position:%s"
                    , hangSampleAndPark.backLeft.getCurrentPosition(), hangSampleAndPark.backLeft.getTargetPosition());
            hangSampleAndPark.telemetry.update();
            hangSampleAndPark.backRight.setVelocity(0);
            hangSampleAndPark.backLeft.setVelocity(0);
        }

        // Reset encoders
        hangSampleAndPark.backRight.setVelocity(0);
        hangSampleAndPark.backLeft.setVelocity(0);
        hangSampleAndPark.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hangSampleAndPark.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}
