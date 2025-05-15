package org.firstinspires.ftc.teamcode.pedroPathing;

/***********************************************************************************************
 * Date: 7th May 2025
 *
 * Purpose: To improve the recent autonomous developed in FaaezahAutonomous.java, utlizing the state
 *          machine and trajectory. Inspiration was Taken from Team 2077 Horizon's github repository.
 **********************************************************************************************/


// Import statements (self-explanatory)
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

@Config
@Autonomous(name = "Faaezah AutoImprov")
public class FaaezahAutoImprov extends OpMode {
    private Telemetry telemetryA; // Shows robot data on DriverHub
    private Follower follower; // 'Follows the robot' - makes sure movement is okay
    private Path forwardPath; //Movement num 1
    private Path turnPath; //Movement num 2
    private Path secondForwardPath; //Movement num 3
    private int state = 0;

    @Override
    public void init() {

        Constants.setConstants(FConstants.class, LConstants.class);

        //follower is being initalized and used in the program
        follower = new Follower(hardwareMap);

        /* BezierLine (heart of Pedro Pathing) is advanced mathematics that helps to 'pathing' of
           robot. N.B: Faaezah, start learning about BezierLine mathematics!
         */

        //robots goes forward
        forwardPath = new Path(new BezierLine(
                new Point(0, 0, Point.CARTESIAN),
                new Point(24, 0, Point.CARTESIAN)
        ));
        //tells robot which position to face in
        forwardPath.setConstantHeadingInterpolation(0);


        turnPath = new Path(new BezierLine(
                new Point(24, 0, Point.CARTESIAN),
                new Point(24, 24, Point.CARTESIAN)
        ));
        turnPath.setConstantHeadingInterpolation(Math.PI/2); // 90 degree turn

        secondForwardPath = new Path(new BezierLine(
                new Point(24, 24, Point.CARTESIAN),
                new Point(48, 24, Point.CARTESIAN)
        ));
        secondForwardPath.setConstantHeadingInterpolation(Math.PI/2);

        // Telemetry is set here
        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        // Robot moves using the first path
        follower.followPath(forwardPath);


        telemetryA.addLine("Autonomous Initialized");
        telemetryA.update();
    }

    @Override

    public void loop() {
        follower.update();
//Switch statement handles various scenarios
        switch (state) {

            case 0: // FIRST THING THAT HAPPENS ON ROBOT
                if (!follower.isBusy()) { //Follower is not busy = TRUE
                    state = 1;  // IF ABOVE CONDITIONS = CORRECT then bottom line will execute:
                    follower.followPath(turnPath); // Val state is 1, and turnpath is executed
                }
                break; //prevents the loop from repeating unnecessarily OVER and OVER again

            case 1: // SECOND THING THAT HAPPENS ON THE ROBOT
                if (!follower.isBusy()) {  //Follower is not busy  = FALSE; ROBOT TURN COMPLETE = YES
                    state = 2; // IF ABOVE CONDITIONS = CORRECT then bottom line will execute:
                    follower.followPath(secondForwardPath); //Val state is 2, and second forward
                                                           // path is executed.
                }
                break;

            case 2: // THIRD THING THAT HAPPENS ON THE ROBOT
                if (!follower.isBusy()) { //Second forward on robot = TRUE;
                    state = 3;            //IF TRUE, end state by changing this val
                }
                break;

            case 3: // Robot relaxes and does NOT move.
                break;
        }

        // Debug telemetry
        telemetryA.addData("State", state);
        follower.telemetryDebug(telemetryA);
        telemetryA.update();
    }
}

/* These comments do not provide a comprehensive review what this program can execute. Comments were
written on predictions of what can be completed on the robot - code has not been officially tested.
Faaezah has written these comments for personal use, especially for usage in the debugging
process.
 */