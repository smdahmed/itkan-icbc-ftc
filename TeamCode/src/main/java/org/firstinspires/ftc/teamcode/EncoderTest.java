package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class EncoderTest extends LinearOpMode {
    private final boolean USE_SIMULATOR = true;

    /* Declare OpMode members. */
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;

    private DcMotor backRight = null;
    private DcMotor backLeft = null;

    private DcMotor m8;
    private DcMotor m5;
    private DcMotor m6releasesample;

    private Servo m8Robot;
    private CRServo m6releasesampleRobot;

    private final ElapsedTime runtime = new ElapsedTime();

    private final String RIGHT_MOTOR_ROBOT = "right_front_drive";
    private final String LEFT_MOTOR_ROBOT = "left_front_drive";
    private final String ARM_MOTOR_ROBOT = "left_arm";
    private final String WRIST_ROBOT = "wrist";
    private final String INTAKE_ROBOT = "intake";

    private final String RIGHT_MOTOR_SIM = "backRight";
    private final String LEFT_MOTOR_SIM = "backLeft";
    private final String ARM_MOTOR_SIM = "m5";
    private final String WRIST_SIM = "m8";
    private final String INTAKE_SIM = "m6-release sample";

    //Ticks is motor rate * the ratio provided by gobilda.
    private final double ticks = 537.7;
    //Circumference is based on the wheel size.
    private final double circumference = 301.59;
    private int target;

    @Override
    public void runOpMode() {
        initializeDevices();
        setupEncoders();
        waitForStart();

        goStraight(50, 1);
        turnRight(44, 2);
        moveArmDown(10, 4);
        pickupSample();
        goStraight(5, 1);

        telemetry.addData("Status", "Ended");
    }

    private void initializeDevices() {
        backRight = hardwareMap.get(DcMotor.class, USE_SIMULATOR ? RIGHT_MOTOR_SIM : RIGHT_MOTOR_ROBOT);
        backLeft = hardwareMap.get(DcMotor.class, USE_SIMULATOR ? LEFT_MOTOR_SIM : LEFT_MOTOR_ROBOT);
        m5 = hardwareMap.get(DcMotor.class, USE_SIMULATOR ? ARM_MOTOR_SIM : ARM_MOTOR_ROBOT);

        if (USE_SIMULATOR) {
            m8 = hardwareMap.get(DcMotor.class, USE_SIMULATOR ? WRIST_SIM : WRIST_ROBOT);
            m5 = hardwareMap.get(DcMotor.class, USE_SIMULATOR ? ARM_MOTOR_SIM : ARM_MOTOR_ROBOT);
            m6releasesample = hardwareMap.get(DcMotor.class, USE_SIMULATOR ? INTAKE_SIM : INTAKE_ROBOT);
            leftDrive = hardwareMap.get(DcMotor.class, "frontLeft");
            rightDrive = hardwareMap.get(DcMotor.class, "frontRight");
            leftDrive.setDirection(DcMotorSimple.Direction.FORWARD);
            rightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
            moveWrist(10,0);
        } else {
//            m8Robot = hardwareMap.get(Servo.class, USE_SIMULATOR ? WRIST_SIM : WRIST_ROBOT);
            m6releasesampleRobot = hardwareMap.get(CRServo.class, USE_SIMULATOR ? INTAKE_SIM : INTAKE_ROBOT);
        }

        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    private void setupEncoders() {
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m8.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m5.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m6releasesample.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        m5.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Reset the motor encoder
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Reset the motor encoder
        m8.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m5.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m6releasesample.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    private void goStraight(int distance, int leg) {
        // Go straight
        //For some reason the simulator does not accepted casted values.
        if (!USE_SIMULATOR) {
            target = (int) ((distance * 10 / circumference) * ticks);
        } else {
            //Comment this out if using robot.
//            target = (distance * 100 / circumference) * (ticks) + backRight.getCurrentPosition();
        }
        backRight.setTargetPosition(target);
        backLeft.setTargetPosition(target);
        backRight.setPower(1);
        backLeft.setPower(1);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (backRight.getCurrentPosition() < backRight.getTargetPosition() && backLeft.getCurrentPosition() < backLeft.getTargetPosition()) {
            telemetry.addData("Path", "Leg: %s Going straight: Current position: %s Target Position:%s",
                    leg, backRight.getCurrentPosition(), backRight.getTargetPosition());
            telemetry.update();
        }
        // Reset encoders
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    private void turnRight(int distance, int leg) {
        //For some reason the simulator does not accepted casted values.
        if (!USE_SIMULATOR) {
            target = (int) ((distance * 10 / circumference) * ticks);
        } else {
            //Comment this out if using robot.
//            target = (distance * 100 / circumference) * (ticks) + backRight.getCurrentPosition();
        }
        backRight.setTargetPosition(target);
        backLeft.setTargetPosition(target);
        backRight.setPower(1);
        backLeft.setPower(-1);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        telemetry.addData("Path1", "Leg 2: Current position: %s Target Position:%s"
                , backRight.getCurrentPosition(), backRight.getTargetPosition());
        telemetry.update();
        while (backRight.getCurrentPosition() < backRight.getTargetPosition() && backLeft.getCurrentPosition() < backLeft.getTargetPosition()) {
            telemetry.addData("Path", "Leg %s: Current position: %s Target Position:%s", leg
                    , backRight.getCurrentPosition(), backRight.getTargetPosition());
            telemetry.update();
        }
        // Reset encoders
        backRight.setPower(0);
        backLeft.setPower(0);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    private void moveArmDown(int distance, int leg) {
        //For some reason the simulator does not accepted casted values.
        if (!USE_SIMULATOR) {
            target = (int) ((distance * 10 / circumference) * ticks);
        } else {
            //Comment this out if using robot.
//            target = (distance * 100 / circumference) * (ticks) + m8.getCurrentPosition();
        }
        m8.setTargetPosition(target);
        m8.setPower(1);
        m8.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (runtime.seconds() < distance) {
            telemetry.addData("Path", "Leg %s: Arm movement: Current position: %s Target Position:%s"
                    , leg, runtime.seconds(), target);
            telemetry.update();
        }

        // Reset encoders
        m8.setPower(0);
        m8.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    private void moveArmUp(int distance, int leg) {
        //For some reason the simulator does not accepted casted values.
        if (!USE_SIMULATOR) {
            target = (int) ((distance * 10 / circumference) * ticks);
        } else {
            //Comment this out if using robot.
//            target = (distance * 100 / circumference) * (ticks) + m8.getCurrentPosition();
        }
        m8.setTargetPosition(target);
        m8.setPower(1);
        m8.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (runtime.seconds() < distance) {
            telemetry.addData("Path", "Leg %s: Arm movement: Current position: %s Target Position:%s"
                    , leg, runtime.seconds(), target);
            telemetry.update();
        }

        // Reset encoders
        m8.setPower(0);
        m8.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    private void moveWrist(int distance, int leg) {
        //For some reason the simulator does not accepted casted values.
        if (!USE_SIMULATOR) {
            target = (int) ((distance * 10 / circumference) * ticks);
        } else {
            //Comment this out if using robot.
//            target = (distance / circumference) * (ticks) + m8.getCurrentPosition();
        }
        m5.setTargetPosition(target);
        m5.setPower(1);
        m5.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (runtime.seconds() < distance) {
            telemetry.addData("Path", "Leg %s: Wrist movement: Current position: %s Target Position:%s"
                    , leg, m5.getCurrentPosition(), m5.getTargetPosition());
            telemetry.update();
        }
        m5.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m5.setPower(0);

        // Reset encoders
        m5.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    private void pickupSample() {
        if(USE_SIMULATOR){
            m6releasesample.setPower(1);
        }
        else{
            m6releasesampleRobot.setPower(1);
        }
    }

    private void dropSample() {
        if(USE_SIMULATOR){
            m6releasesample.setPower(-1);
        }
        else{
            m6releasesampleRobot.setPower(-1);
        }
    }
}
