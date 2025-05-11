package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "HighBasketScoring", group = "Robot")
public class AlishbaHighBasket extends LinearOpMode {

    private DcMotorEx backLeft, backRight, armMotor, viperKit;
    private CRServo robotSampleServo;
    private final double ARM_TICKS_PER_DEGREE =
            28 * 250047.0 / 4913.0 * 100.0 / 20.0 * 1 / 360.0;
    private final double ARM_SCORE_SAMPLE_IN_HIGH = 100 * ARM_TICKS_PER_DEGREE;

    @Override
    public void runOpMode() {
        // Initialize motors & servos
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");
        armMotor = hardwareMap.get(DcMotorEx.class, "arm");
        viperKit = hardwareMap.get(DcMotorEx.class, "viperKit");
        robotSampleServo = hardwareMap.get(CRServo.class, "robotSampleServo");

        configureMotor(backLeft);
        configureMotor(backRight);
        configureMotor(armMotor);
        configureMotor(viperKit);

        waitForStart();

        PathBuilder builder = new PathBuilder();

        // Move to the high basket
        PathChain line1 = builder
                .addPath(new BezierLine(
                        new Point(8.538, 105.231, Point.CARTESIAN),
                        new Point(10.846, 130.154, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        // Move arm to high basket position
        moveArmToHighBasket();

        // Drop sample into high basket
        dropSample();

        // Exit path after scoring
        PathChain exitPath = builder
                .addPath(new BezierLine(
                        new Point(10.615, 129.923, Point.CARTESIAN),
                        new Point(60.231, 95.538, Point.CARTESIAN)
                ))
                .setTangentHeadingInterpolation()
                .build();

        // Park after scoring
        driveForward(0.6, 40);
    }

    private void configureMotor(DcMotorEx motor) {
        motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    private void driveForward(double power, int targetTicks) {
        backLeft.setTargetPosition(backLeft.getCurrentPosition() + targetTicks);
        backRight.setTargetPosition(backRight.getCurrentPosition() + targetTicks);

        backLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        backLeft.setPower(power);
        backRight.setPower(power);

        while (backLeft.isBusy() && backRight.isBusy() && opModeIsActive()) {
            telemetry.addData("Driving", "Moving Forward...");
            telemetry.update();
        }

        backLeft.setPower(0);
        backRight.setPower(0);
    }

    private void moveArmToHighBasket() {
        armMotor.setTargetPosition((int) ARM_SCORE_SAMPLE_IN_HIGH);
        armMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        armMotor.setPower(1.0);

        while (armMotor.isBusy() && opModeIsActive()) {
            telemetry.addData("Arm", "Moving to High Basket...");
            telemetry.update();
        }

        armMotor.setPower(0);
    }

    private void dropSample() {
        robotSampleServo.setPower(-1);
        sleep(500);
        robotSampleServo.setPower(0);
    }
}
