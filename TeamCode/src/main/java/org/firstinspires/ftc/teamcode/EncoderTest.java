package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "PickupSample", group = "Robot")
public class EncoderTest extends LinearOpMode {
    public DcMotor backRight;
    public DcMotor backLeft;

    public DcMotor arm;

    public CRServo robotSampleServo;
    public DcMotor viperKit;

    private final ElapsedTime runtime = new ElapsedTime();

    //Ticks is motor rate * the ratio provided by Gobilda.
    public final double ticks = 537.7;
    //Circumference is based on the wheel size.
    public final double circumference = 301.59;
    public final double ARM_TICKS_PER_DEGREE =
            28 // number of encoder ticks per rotation of the bare motor
                    * 250047.0 / 4913.0 // This is the exact gear ratio of the 50.9:1 Yellow Jacket gearbox
                    * 100.0 / 20.0 // This is the external gear reduction, a 20T pinion gear that drives a 100T hub-mount gear
                    * 1 / 360.0; // we want ticks per degree, not per rotation

    final double ARM_SCORE_SAMPLE_IN_HIGH = 100 * ARM_TICKS_PER_DEGREE;

    @Override
    public void runOpMode() {
        EncoderInterface encoderInterface = new EncoderInterface(this);
        initializeDevices();
        setupEncoders();
        waitForStart();

        //Start with sample in basket
        moveArmUp();
        goStraight(38);
        robotSampleServo.setPower(-1);
        extendViperKit(-55);
        moveArmDown(50);
        goBackwards(25);
        extendViperKit(55);
        robotSampleServo.setPower(0);

        telemetry.addData("Status", "Ended");
    }

    private void extendViperKit(int viperPosition){
        viperKit.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // Set the target position to the position the driver asked for
        viperKit.setTargetPosition((int) (viperPosition * ARM_TICKS_PER_DEGREE));
        // Set the velocity of the motor and use setMode to run
        viperKit.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        viperKit.setPower(1);
        while(viperKit.isBusy()){
            telemetry.addData("Value", "Current Position: %s", viperKit.getCurrentPosition());
        }
    }
    private void goStraight(int distance) {
        // Go straight
        int target = (int) ((distance * 10 / circumference) * ticks);
        backRight.setTargetPosition(target);
        backLeft.setTargetPosition(target);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setPower(0.5);
        backLeft.setPower(0.5);
        while (backRight.isBusy() && backLeft.isBusy()) {
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
        // Go straight
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int target = (int) ((distance * 10 / circumference) * ticks);
        backRight.setTargetPosition(-target);
        backLeft.setTargetPosition(-target);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setPower(-1);
        backLeft.setPower(-1);
        while (backRight.isBusy() && backLeft.isBusy()) {
            telemetry.addData("Path", "Going backwards: Current position: %s Target Position:%s",
                    backRight.getCurrentPosition(), backRight.getTargetPosition());
            telemetry.update();
        }
        // Reset encoders
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    private void turnRight(int distance) {
        //For some reason the simulator does not accepted casted values.
        int target = (int) ((distance * 10 / circumference) * ticks);

        backRight.setTargetPosition(target);
        backLeft.setTargetPosition(target);
        backRight.setPower(1);
        backLeft.setPower(-1);
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
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    private void turnLeft(int distance) {
        //For some reason the simulator does not accepted casted values.
        int target = (int) ((distance * 10 / circumference) * ticks);

        backRight.setTargetPosition(target);
        backLeft.setTargetPosition(target);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setPower(1);
        backLeft.setPower(-1);
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
        int target = (int) ((distance * 10 / circumference) * ticks);

        arm.setTargetPosition(-target);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(0.5);
        while (arm.isBusy()) {
            telemetry.addData("Path",
                    "Arm movement down: Current position: %s Target Position:%s"
                    , arm.getCurrentPosition(), arm.getTargetPosition());
        }
        // Reset encoders
        arm.setPower(0);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    private void moveArmUp() {
        arm.setTargetPosition((int) ARM_SCORE_SAMPLE_IN_HIGH);
        arm.setPower(0.5);
        arm.setPower(1);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (arm.isBusy()) {
            telemetry.addData("Path",
                    "Arm movement up: Current position: %s Target Position:%s"
                    , arm.getCurrentPosition(), arm.getTargetPosition());
            telemetry.update();
        }
        // Reset encoders
        arm.setPower(0);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    private boolean isNotInPosition(DcMotor motor) {
        return motor.getCurrentPosition() < motor.getTargetPosition();
    }

    private void initializeDevices() {
        backRight = hardwareMap.get(DcMotor.class, "right_front_drive");
        backLeft = hardwareMap.get(DcMotor.class, "left_front_drive");
        arm = hardwareMap.get(DcMotor.class, "left_arm");
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setTargetPosition(15);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(1);
        robotSampleServo = hardwareMap.get(CRServo.class, "intake");
        viperKit = hardwareMap.get(DcMotor.class, "viper_kit");
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
        viperKit.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
}