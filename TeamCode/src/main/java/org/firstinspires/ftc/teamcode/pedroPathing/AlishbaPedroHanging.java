package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.util.Constants;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

public class AlishbaPedroHanging extends LinearOpMode {

    private Follower follower;
    private DcMotorEx armMotor, viperKit;
    private CRServo robotSampleServo;
    private PathChain hangingPath, exitPath;

    @Override
    public void runOpMode() {
        // Initialize motors & servos
        armMotor = hardwareMap.get(DcMotorEx.class, "arm");
        viperKit = hardwareMap.get(DcMotorEx.class, "viperKit");
        robotSampleServo = hardwareMap.get(CRServo.class, "robotSampleServo");

        configureMotor(armMotor);
        configureMotor(viperKit);

        // Initialize Pedro Pathing Follower
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);

        PathBuilder builder = new PathBuilder();

        // Path to hanging rod (Rod 2)
        hangingPath = builder.addPath(new BezierLine(
                        new Point(8.000, 80.000, Point.CARTESIAN),
                        new Point(39.923, 79.615, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        // Exit path after hanging
        exitPath = builder.addPath(new BezierLine(
                        new Point(39.923, 79.615, Point.CARTESIAN),
                        new Point(60.231, 95.538, Point.CARTESIAN)
                ))
                .setTangentHeadingInterpolation()
                .build();

        waitForStart();

        // Follow path to hanging position
        follower.followPath(hangingPath);
        while (follower.isBusy() && opModeIsActive()) {
            follower.update();
        }

        // Lift arm to hang
        moveArmToRod2();

        // Extend Viper Kit for stability
        extendViperKit(-60);

        // Drop sample
        robotSampleServo.setPower(-1);
        sleep(500);
        robotSampleServo.setPower(0);

        // Follow exit path
        follower.followPath(exitPath);
        while (follower.isBusy() && opModeIsActive()) {
            follower.update();
        }
    }

    private void configureMotor(DcMotorEx motor) {
        motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    private void moveArmToRod2() {
        int rod2TargetPosition = 950; // Adjusted height for Rod 2
        armMotor.setTargetPosition(rod2TargetPosition);
        armMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        armMotor.setPower(1.0);

        while (armMotor.isBusy() && opModeIsActive()) {
            telemetry.addData("Arm", "Lifting to Rod 2...");
            telemetry.update();
        }

        armMotor.setPower(0);
    }

    private void extendViperKit(int targetTicks) {
        viperKit.setTargetPosition(viperKit.getCurrentPosition() + targetTicks);
        viperKit.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        viperKit.setPower(0.8);

        while (viperKit.isBusy() && opModeIsActive()) {
            telemetry.addData("Viper Kit", "Extending for Rod 2...");
            telemetry.update();
        }

        viperKit.setPower(0);
    }
}