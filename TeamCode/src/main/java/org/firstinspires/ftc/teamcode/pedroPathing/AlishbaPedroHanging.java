package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class AlishbaPedroHanging extends LinearOpMode {

    private DcMotor leftMotor, rightMotor, armMotor;

    @Override
    public void runOpMode() {
        // Initialize motors
        leftMotor = hardwareMap.get(DcMotor.class, "leftMotor");
        rightMotor = hardwareMap.get(DcMotor.class, "rightMotor");
        armMotor = hardwareMap.get(DcMotor.class, "armMotor");

        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        // Pedro Pathing - Move to Hang Bar
        PathBuilder builder = new PathBuilder();
        PathChain line1 = builder
                .addPath(new BezierLine(
                        new Point(8.000, 80.000, Point.CARTESIAN),
                        new Point(39.923, 79.615, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        // Move Robot Forward
        driveForward(0.7, 1000); // Adjust targetTicks as needed

        // Move Arm to Hang
        moveArmToHang();
    }

    private void driveForward(double power, int targetTicks) {
        leftMotor.setTargetPosition(leftMotor.getCurrentPosition() + targetTicks);
        rightMotor.setTargetPosition(rightMotor.getCurrentPosition() + targetTicks);

        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftMotor.setPower(power);
        rightMotor.setPower(power);

        while (leftMotor.isBusy() && rightMotor.isBusy() && opModeIsActive()) {
            telemetry.addData("Driving", "Moving Forward...");
            telemetry.update();
        }

        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }

    private void moveArmToHang() {
        int targetPosition = 1000; // Adjust height for hanging
        armMotor.setTargetPosition(targetPosition);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(1.0);

        while (armMotor.isBusy() && opModeIsActive()) {
            telemetry.addData("Arm", "Lifting...");
            telemetry.update();
        }

        armMotor.setPower(0);
    }
}
