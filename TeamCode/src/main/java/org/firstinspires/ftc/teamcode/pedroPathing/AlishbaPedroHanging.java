package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


public class AlishbaPedroHanging extends LinearOpMode {

    private DcMotorEx backLeft, backRight, armMotor, viperKit;
    private CRServo robotSampleServo;

    @Override
    public void runOpMode() {
        // Initialize motors & servos
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");
        armMotor = hardwareMap.get(DcMotorEx.class, "arm");
        viperKit = hardwareMap.get(DcMotorEx.class, "viperKit");
        robotSampleServo = hardwareMap.get(CRServo.class, "robotSampleServo");

        // Encoder and Brake Configurations
        configureMotor(backLeft);
        configureMotor(backRight);
        configureMotor(armMotor);
        configureMotor(viperKit);

        waitForStart();

        // Pedro Pathing - Move to Rod 2
        PathBuilder builder = new PathBuilder();
        PathChain line1 = builder
                .addPath(new BezierLine(
                        new Point(8.000, 80.000, Point.CARTESIAN),
                        new Point(39.923, 79.615, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        // Move Robot Forward (Adjusted Distance for Rod 2)
        driveForward(0.7, 850);

        // Lift Arm for Hanging on Rod 2
        moveArmToRod2();

        // Extend Viper Kit for Stability on Rod 2
        extendViperKit(-60);

        // Drop Sample using Servo
        robotSampleServo.setPower(-1);
        sleep(500);
        robotSampleServo.setPower(0);

        // Park After Hanging
        driveForward(0.6, 40);
    }

    private void configureMotor(DcMotorEx motor) {
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    private void driveForward(double power, int targetTicks) {
        backLeft.setTargetPosition(backLeft.getCurrentPosition() + targetTicks);
        backRight.setTargetPosition(backRight.getCurrentPosition() + targetTicks);

        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        backLeft.setPower(power);
        backRight.setPower(power);

        while (backLeft.isBusy() && backRight.isBusy() && opModeIsActive()) {
            telemetry.addData("Driving", "Moving Forward...");
            telemetry.update();
        }

        backLeft.setPower(0);
        backRight.setPower(0);
    }

    private void moveArmToRod2() {
        int rod2TargetPosition = 950; // Adjusted height for Rod 2
        armMotor.setTargetPosition(rod2TargetPosition);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(1.0);

        while (armMotor.isBusy() && opModeIsActive()) {
            telemetry.addData("Arm", "Lifting to Rod 2...");
            telemetry.update();
        }

        armMotor.setPower(0);
    }

    private void extendViperKit(int targetTicks) {
        viperKit.setTargetPosition(viperKit.getCurrentPosition() + targetTicks);
        viperKit.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        viperKit.setPower(0.8);

        while (viperKit.isBusy() && opModeIsActive()) {
            telemetry.addData("Viper Kit", "Extending for Rod 2...");
            telemetry.update();
        }

        viperKit.setPower(0);
    }
}
