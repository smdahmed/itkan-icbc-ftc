package org.firstinspires.ftc.teamcode.pedroPathing.PedroScripts.Ibrahim_Pedro;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Timer;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;


import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.pedropathing.localization.PoseUpdater;
import com.pedropathing.util.DashboardPoseTracker;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@Autonomous(name = "auto pickup", group = "Examples")
public class pedrobrickbucket extends OpMode {
    private Follower follower;
    private Timer pathTimer, opmodeTimer;

    /** This is the variable where we store the state of our auto.
     * It is used by the pathUpdate method. */
    private int pathState;
    public DcMotor  viperKit    = null;
    final double VIPER_TICKS_PER_DEGREE =
            28
                    * 13.7
                    * 1/360.0;
    final double VIPER_INIT                = 5 * VIPER_TICKS_PER_DEGREE;
    double viperPosition = (int) VIPER_INIT;


    /* Create and Define Poses + Paths
     * Poses are built with three constructors: x, y, and heading (in Radians).
     * Pedro uses 0 - 144 for x and y, with 0, 0 being on the bottom left.
     * (For Into the Deep, this would be Blue Observation Zone (0,0) to Red Observation Zone (144,144).)
     * Even though Pedro uses a different coordinate system than RR, you can convert any roadrunner pose by adding +72 both the x and y.
     * This visualizer is very easy to use to find and create paths/pathchains/poses: <https://pedro-path-generator.vercel.app/>
     * Lets assume our robot is 18 by 18 inches
     * Lets assume the Robot is facing the human player and we want to score in the bucket */

    /** Start Pose of our robot */
    private final Pose startPose = new Pose(3, 71, Math.toRadians(0));

    /** Scoring Pose of our robot. It is facing the submersible at a -45 degree (315 degree) angle. */
    private final Pose point1 = new Pose(24, 71, Math.toRadians(0));

    /** Lowest (First) Sample from the Spike Mark */
    private final Pose pickupbrick = new Pose(24, 120, Math.toRadians(0));

    /** Middle (Second) Sample from the Spike Mark */
    private final Pose rotatenow = new Pose(24, 100, Math.toRadians(0));

    /** Highest (Third) Sample from the Spike Mark */
    private final Pose pickup3Pose = new Pose(2.85, 117, Math.toRadians(137));

    /** Park Pose for our robot, after we do all of the scoring. */
    //private final Pose align = new Pose(12, 131, Math.toRadians(137));

    /** Park Control Pose for our robot, this is used to manipulate the bezier curve that we will create for the parking.
     * The Robot will not go to this pose, it is used a control point for our bezier curve. */

    /* These are our Paths and PathChains that we will define in buildPaths() */
    private Path scorebasket;
    private PathChain moveahead, grabPickup2, scorePickup1;
    public DcMotor  armMotor    = null;
    final double ARM_TICKS_PER_DEGREE =
            28 // number of encoder ticks per rotation of the bare motor
                    * 250047.0 / 4913.0 // This is the exact gear ratio of the 50.9:1 Yellow Jacket gearbox
                    * 100.0 / 20.0 // This is the external gear reduction, a 20T pinion gear that drives a 100T hub-mount gear
                    * 1/360.0;
    final double ARM_INIT                  = 80 * ARM_TICKS_PER_DEGREE;
    final double ARM_SCORE_SAMPLE_IN_HIGH  = 120 * ARM_TICKS_PER_DEGREE;
    final double ARM_GET_SAMPLE            = 55 * ARM_TICKS_PER_DEGREE; // Changed so it's easier to pick up samples
    double armPosition = (int) ARM_INIT;


    /** Build the paths for the auto (adds, for example, constant/linear headings while doing paths)
     * It is necessary to do this so that all the paths are built before the auto starts. **/
    public void buildPaths() {

        /* There are two major types of paths components: BezierCurves and BezierLines.
         *    * BezierCurves are curved, and require >= 3 points. There are the start and end points, and the control points.
         *    - Control points manipulate the curve between the start and end points.
         *    - A good visualizer for this is [this](https://pedro-path-generator.vercel.app/).
         *    * BezierLines are straight, and require 2 points. There are the start and end points.
         * Paths have can have heading interpolation: Constant, Linear, or Tangential
         *    * Linear heading interpolation:
         *    - Pedro will slowly change the heading of the robot from the startHeading to the endHeading over the course of the entire path.
         *    * Constant Heading Interpolation:
         *    - Pedro will maintain one heading throughout the entire path.
         *    * Tangential Heading Interpolation:
         *    - Pedro will follows the angle of the path such that the robot is always driving forward when it follows the path.
         * PathChains hold Path(s) within it and are able to hold their end point, meaning that they will holdPoint until another path is followed.
         * Here is a explanation of the difference between Paths and PathChains <https://pedropathing.com/commonissues/pathtopathchain.html> */


        scorebasket = new Path(new BezierLine(new Point(startPose), new Point(point1)));
        scorebasket.setLinearHeadingInterpolation(startPose.getHeading(), point1.getHeading());

        moveahead = follower.pathBuilder()
                .addPath(new BezierLine(new Point(point1), new Point(pickupbrick)))
                .setLinearHeadingInterpolation(point1.getHeading(), pickupbrick.getHeading())
                .build();


        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickupbrick), new Point(rotatenow)))
                .setLinearHeadingInterpolation(0,0)
                .build();

        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(rotatenow), new Point(pickup3Pose)))
                .setConstantHeadingInterpolation(pickup3Pose.getHeading())
                .build();


    }


    /** This switch is called continuously and runs the pathing, at certain points, it triggers the action state.
     * Everytime the switch changes case, it will reset the timer. (This is because of the setPathState() method)
     * The followPath() function sets the follower to run the specific path, but does NOT wait for it to finish before moving on. */
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(scorebasket);
                setPathState(1);
                break;
            case 1:

                /* You could check for
                - Follower State: "if(!follower.isBusy() {}"
                - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
                - Robot Position: "if(follower.getPose().getX() > 36) {}"
                */

                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the point1's position */
                if(!follower.isBusy()) {
                    /* Score Preload */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(moveahead,true);
                    setPathState(2);
                }
                break;
            case 2:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickupbrick's position */
                if(!follower.isBusy()) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(scorePickup1,true);
                    setPathState(3);
                }
                break;
            case 3:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the point1's position */
                if(!follower.isBusy()) {
                    /* Score Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(grabPickup2,true);
                    setPathState(4);
                    viperPosition = 4*360;
                    armPosition = ARM_SCORE_SAMPLE_IN_HIGH;
                }
                break;
        }
    }

    /** These change the states of the paths and actions
     * It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {
        //armMotor = hardwareMap.get(DcMotor.class, "left_arm");

        // These loop the movements of the robot

        follower.update();
        autonomousPathUpdate();
        viperKit.setTargetPosition((int) viperPosition);
        viperKit.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        viperKit.setPower(0.5);
        viperKit.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setTargetPosition((int) (armPosition));
        // Reduced arm velocity so it wouldn't jitter when moving
        ((DcMotorEx) armMotor).setVelocity(1600);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // Feedback to Driver Hub
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }
    @Override
    public void init() {
        viperKit = hardwareMap.get(DcMotor.class, "viper_kit");
        armMotor = hardwareMap.get(DcMotor.class, "left_arm");

        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);
        buildPaths();
    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {}

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {
    }

}
