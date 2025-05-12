package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.util.Constants;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.Point;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

@Autonomous(name = "HighBasketScoring", group = "Robot")
public class AlishbaHighBasket extends LinearOpMode {

    private Follower follower;
    private DcMotorEx armMotor, viperKit;
    private CRServo robotSampleServo;
    private final double ARM_TICKS_PER_DEGREE =
            28 * 250047.0 / 4913.0 * 100.0 / 20.0 * 1 / 360.0;
    private final double ARM_SCORE_SAMPLE_IN_HIGH = 100 * ARM_TICKS_PER_DEGREE;
    private PathChain scoringPath, exitPath;

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

        // Path to high basket
        scoringPath = builder.addPath(new BezierLine(
                        new Point(8.538, 105.231, Point.CARTESIAN),
                        new Point(10.846, 130.154, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        // Exit path after scoring
        exitPath = builder.addPath(new BezierLine(
                        new Point(10.615, 129.923, Point.CARTESIAN),
                        new Point(60.231, 95.538, Point.CARTESIAN)
                ))
                .setTangentHeadingInterpolation()
                .build();

        waitForStart();

        // Follow path to high basket
        follower.followPath(scoringPath);
        while (follower.isBusy() && opModeIsActive()) {
            follower.update();
        }

        // Move arm to place sample
        moveArmToHighBasket();

        // Drop sample into basket
        dropSample();

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