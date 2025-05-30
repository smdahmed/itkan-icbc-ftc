package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="FTC Starter Kit TeleOp (Pedro Pathing)", group="Robot")
public class tele extends LinearOpMode {

    private DcMotorEx leftDrive, rightDrive, backLeft, backRight, frontLeft, frontRight, armMotor, viperKit;
    private CRServo intake;

    private double forward, rotate, left, right, maxPower;
    private double armPosition, viperPosition;

    // Encoder constants for precise movement
    private final double ARM_TICKS_PER_DEGREE = 28 * 250047.0 / 4913.0 * 100.0 / 20.0 * 1/360.0;
    private final double VIPER_TICKS_PER_DEGREE = 28 * 13.7 * 1/360.0;

    // Arm positions
    private final double ARM_COLLAPSED = 0;
    private final double ARM_COLLECT = 20 * ARM_TICKS_PER_DEGREE;
    private final double ARM_GET_SAMPLE = 30 * ARM_TICKS_PER_DEGREE;
    private final double ARM_SCORE_SAMPLE_IN_HIGH = 100 * ARM_TICKS_PER_DEGREE;
    private final double ARM_ATTACH_HOOK = 140 * ARM_TICKS_PER_DEGREE;
    private final double ARM_WINCH_ROBOT = 15 * ARM_TICKS_PER_DEGREE;

    // Viper kit positions
    private final double VIPER_OUT = -4 * VIPER_TICKS_PER_DEGREE;
    private final double VIPER_INIT = 5 * VIPER_TICKS_PER_DEGREE;

    // Intake settings
    private final double INTAKE_COLLECT = -1.0;
    private final double INTAKE_OFF = 0.0;
    private final double INTAKE_DEPOSIT = 0.5;

    @Override
    public void runOpMode() {
        // Initialize hardware
        backLeft = hardwareMap.get(DcMotorEx.class, "left_back_drive");
        backRight = hardwareMap.get(DcMotorEx.class, "right_back_drive");
        frontLeft = hardwareMap.get(DcMotorEx.class, "left_front_drive");
        frontRight = hardwareMap.get(DcMotorEx.class, "right_front_drive");
        leftDrive = hardwareMap.get(DcMotorEx.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotorEx.class, "right_drive");
        armMotor = hardwareMap.get(DcMotorEx.class, "left_arm");
        viperKit = hardwareMap.get(DcMotorEx.class, "viper_kit");
        intake = hardwareMap.get(CRServo.class, "intake");

        configureMotor(backLeft);
        configureMotor(backRight);
        configureMotor(frontLeft);
        configureMotor(frontRight);
        configureMotor(leftDrive);
        configureMotor(rightDrive);
        configureMotor(armMotor);
        configureMotor(viperKit);

        waitForStart();

        while (opModeIsActive()) {
            // Drive control
            forward = -gamepad1.left_stick_y;
            rotate = gamepad1.right_stick_x;

            left = forward + rotate;
            right = forward - rotate;

            // Normalize power values
            maxPower = Math.max(Math.abs(left), Math.abs(right));
            if (maxPower > 1.0) {
                left /= maxPower;
                right /= maxPower;
            }

            leftDrive.setPower(left);
            rightDrive.setPower(right);

            // Intake control (Fixing mismapped buttons)
            if (gamepad2.a) intake.setPower(INTAKE_COLLECT);
            else if (gamepad2.x) intake.setPower(INTAKE_OFF);
            else if (gamepad2.b) intake.setPower(INTAKE_DEPOSIT);

            // Arm movement logic (Fixed button inconsistencies)
            if (gamepad2.right_bumper) armPosition = ARM_COLLECT;
            else if (gamepad2.left_bumper) armPosition = ARM_GET_SAMPLE;
            else if (gamepad2.y) armPosition = ARM_SCORE_SAMPLE_IN_HIGH;
            else if (gamepad2.dpad_left) {
                armPosition = ARM_COLLAPSED;
                intake.setPower(INTAKE_OFF);
                viperPosition = -VIPER_INIT;
            }
            else if (gamepad2.dpad_up) armPosition = ARM_ATTACH_HOOK;
            else if (gamepad2.dpad_down) armPosition = ARM_WINCH_ROBOT;

            // Viper Kit control (Fix encoder reset timing)
            if (gamepad2.left_trigger > 0.0) {
                viperPosition = VIPER_OUT;
                if (armPosition == ARM_COLLAPSED) {
                    viperPosition = 0;
                    viperKit.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                }
            }
            else if (gamepad2.right_trigger > 0.0) viperPosition = 0;
            else if (gamepad2.dpad_right) {
                viperPosition = -VIPER_INIT;
                viperKit.setPower(0.5);
                viperKit.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }

            // Special functions
            if (gamepad1.guide) autoHang();
            else if (gamepad2.left_stick_button) {
                inSub();
                intake.setPower(INTAKE_COLLECT);
                sleep(500);
                armMotor.setTargetPosition((int) ARM_COLLECT);
            }
            else if (gamepad2.right_stick_button) outSub();

            // Apply movement commands
            armMotor.setTargetPosition((int) armPosition);
            ((DcMotorEx) armMotor).setVelocity(1600);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            viperKit.setTargetPosition((int) viperPosition);
            ((DcMotorEx) viperKit).setVelocity(1600);
            viperKit.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Debug telemetry (Ensures proper debugging feedback)
            telemetry.addData("Arm Target", armMotor.getTargetPosition());
            telemetry.addData("Arm Encoder", armMotor.getCurrentPosition());
            telemetry.addData("Viper Kit Target", viperKit.getTargetPosition());
            telemetry.addData("Viper Kit Encoder", viperKit.getCurrentPosition());
            telemetry.update();
        }
    }

    private void configureMotor(DcMotorEx motor) {
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void autoHang() {
        armMotor.setTargetPosition((int) ARM_ATTACH_HOOK);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        intake.setPower(INTAKE_OFF);
        sleep(1500);
        armMotor.setTargetPosition((int) ARM_WINCH_ROBOT);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void inSub() {
        armMotor.setTargetPosition((int) ARM_GET_SAMPLE);
        ((DcMotorEx) armMotor).setVelocity(1600);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        intake.setPower(INTAKE_OFF);
        sleep(1500);
        intake.setPower(INTAKE_COLLECT);
        viperPosition = VIPER_OUT;
        viperKit.setTargetPosition((int) VIPER_OUT);
        ((DcMotorEx) viperKit).setVelocity(1600);
        viperKit.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        intake.setPower(INTAKE_COLLECT);
        sleep(500);
        armMotor.setTargetPosition((int) ARM_COLLECT);
    }

    public void outSub() {
        viperPosition = 0;
    }
}
