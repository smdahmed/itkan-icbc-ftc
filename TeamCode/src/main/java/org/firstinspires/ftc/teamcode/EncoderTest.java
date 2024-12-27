package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

public class EncoderTest extends LinearOpMode {
    private final boolean USE_SIMULATOR = true;

    private DcMotor backRight;
    private DcMotor backLeft;

    private DcMotor m8;
    private DcMotor arm;
    private DcMotor simSampleServo;

    private CRServo robotSampleServo;
    public DcMotor  viperKit;

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

        goStraight(200);
        turnRight(600);
        //moveArmDown(12, 3);
        pickupSample();
        goStraight(250);
        turnRight(1650);
        goStraight(1750);
        turnLeft(450);

        telemetry.addData("Status", "Ended");
    }

    private void initializeDevices() {
       final String RIGHT_MOTOR_ROBOT = "right_front_drive";
       final String LEFT_MOTOR_ROBOT = "left_front_drive";
       final String ARM_MOTOR_ROBOT = "left_arm";
       final String WRIST_ROBOT = "wrist";
       final String INTAKE_ROBOT = "intake";
       final String VIPER_KIT = "viper_kit";

       final String RIGHT_MOTOR_SIM = "backRight";
       final String LEFT_MOTOR_SIM = "backLeft";
       final String ARM_MOTOR_SIM = "m5";
       final String WRIST_SIM = "m8";
       final String INTAKE_SIM = "m6-release sample";

        backRight = hardwareMap.get(DcMotor.class, USE_SIMULATOR ? RIGHT_MOTOR_SIM : RIGHT_MOTOR_ROBOT);
        backLeft = hardwareMap.get(DcMotor.class, USE_SIMULATOR ? LEFT_MOTOR_SIM : LEFT_MOTOR_ROBOT);
        arm = hardwareMap.get(DcMotor.class, USE_SIMULATOR ? ARM_MOTOR_SIM : ARM_MOTOR_ROBOT);

        if (USE_SIMULATOR) {
            m8 = hardwareMap.get(DcMotor.class, USE_SIMULATOR ? WRIST_SIM : WRIST_ROBOT);
            arm = hardwareMap.get(DcMotor.class, USE_SIMULATOR ? ARM_MOTOR_SIM : ARM_MOTOR_ROBOT);
            simSampleServo = hardwareMap.get(DcMotor.class,  INTAKE_SIM);
            /* Declare OpMode members. */
            DcMotor leftDrive = hardwareMap.get(DcMotor.class, "frontLeft");
            DcMotor rightDrive = hardwareMap.get(DcMotor.class, "frontRight");
            leftDrive.setDirection(DcMotorSimple.Direction.FORWARD);
            rightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        } else {
            robotSampleServo = hardwareMap.get(CRServo.class, INTAKE_ROBOT);
            viperKit = hardwareMap.get(DcMotor.class, VIPER_KIT);
            viperKit.setTargetPosition(0);
        }

        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    private void setupEncoders() {
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Reset the motor encoder
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Reset the motor encoder
        m8.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        m8.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    private void goStraight(int distance) {
        // Go straight
        //For some reason the simulator does not accepted casted values.
        if (!USE_SIMULATOR) {
            target = (int) ((distance * 10 / circumference) * ticks);
        }
//        else {
            //Comment this out if using robot.
//            target = (distance * 10 / circumference) * (ticks) + backRight.getCurrentPosition();
//        }
        backRight.setTargetPosition(target);
        backLeft.setTargetPosition(target);
        backRight.setPower(1);
        backLeft.setPower(1);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (backRight.getCurrentPosition() < backRight.getTargetPosition() && backLeft.getCurrentPosition() < backLeft.getTargetPosition()) {
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
        if (!USE_SIMULATOR) {
            target = (int) ((distance * 10 / circumference) * ticks);
        }
//        else {
            //Comment this out if using robot.
//            target = (distance * 10 / circumference) * (ticks) + backRight.getCurrentPosition();
//        }
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
    }

    private void turnLeft(int distance) {
        //For some reason the simulator does not accepted casted values.
        if (!USE_SIMULATOR) {
            target = (int) ((distance * 10 / circumference) * ticks);
        }
//        else {
            //Comment this out if using robot.
//            target = (distance * 10 / circumference) * (ticks) + backLeft.getCurrentPosition();
//        }
        backRight.setTargetPosition(target);
        backLeft.setTargetPosition(target);
        backRight.setPower(-1);
        backLeft.setPower(1);
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
        if (!USE_SIMULATOR) {
            target = (int) ((distance * 10 / circumference) * ticks);
        }
//        else {
            //Comment this out if using robot.
//            target = (distance * 10 / circumference) * (ticks) + m8.getCurrentPosition();
//        }
        m8.setTargetPosition(target);
        m8.setPower(0.5);
        m8.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (runtime.seconds() < distance) {
            telemetry.addData("Path", "Leg %s: Arm movement: Current position: %s Target Position:%s"
                    , runtime.seconds(), target);
            telemetry.update();
        }

        // Reset encoders
        m8.setPower(0);
        m8.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    private void moveArmUp(int distance) {
        //For some reason the simulator does not accepted casted values.
        if (!USE_SIMULATOR) {
            target = (int) ((distance * 10 / circumference) * ticks);
        }
//        else {
            //Comment this out if using robot.
//            target = (distance * 10 / circumference) * (ticks) + m8.getCurrentPosition();
//        }
        m8.setTargetPosition(target);
        m8.setPower(0.5);
        m8.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (runtime.seconds() < distance) {
            telemetry.addData("Path", "Leg %s: Arm movement: Current position: %s Target Position:%s"
                    , runtime.seconds(), target);
            telemetry.update();
        }

        // Reset encoders
        m8.setPower(0);
        m8.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    private void pickupSample() {
        if(USE_SIMULATOR){
            simSampleServo.setPower(1);
        }
        else{
            robotSampleServo.setPower(1);
        }
    }

    private void dropSample() {
        if(USE_SIMULATOR){
            simSampleServo.setPower(-1);
        }
        else{
            robotSampleServo.setPower(-1);
        }
    }

    private void dropSampleInHighBasket(){
        moveArmUp(20);
        if(!USE_SIMULATOR){
            viperKit.setTargetPosition(1);
            viperKit.setPower(1);
            while(viperKit.isBusy()){
                telemetry.addData("Extending viper Arm", "Leg %s: Current position: %s Target Position:%s"
                        , viperKit.getCurrentPosition(), viperKit.getTargetPosition());
            }
            dropSample();
            viperKit.setTargetPosition(0);
            viperKit.setPower(-1);
            while(viperKit.isBusy()){
                telemetry.addData("Extending viper Arm", "Leg %s: Current position: %s Target Position:%s"
                        , viperKit.getCurrentPosition(), viperKit.getTargetPosition());
            }
        }
        moveArmDown(20);

    }

    private boolean isNotInPosition(DcMotor motor){
        return motor.getCurrentPosition() < motor.getTargetPosition();
    }
}