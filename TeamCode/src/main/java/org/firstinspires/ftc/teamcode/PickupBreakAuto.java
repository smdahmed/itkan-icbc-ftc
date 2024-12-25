package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 * <p>
 * This particular OpMode just executes a basic Tank Drive Teleop for a PushBot
 * It includes all the skeletal structure that all linear OpModes contain.
 * <p>
 * Remove a @Disabled the on the next line or two (if present) to add this opmode to the Driver Station OpMode list,
 * or add a @Disabled annotation to prevent this OpMode from being added to the Driver Station
 */

public class PickupBreakAuto extends LinearOpMode {

    // MAKE SURE THAT THIS FLAG IS OFF FOR ROBOT TESTING.
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


    private final double FORWARD_SPEED_ROBOT = 0.4;
    private final double TURN_SPEED_ROBOT = 0.5;
    private final String RIGHT_MOTOR_ROBOT = "right_front_drive";
    private final String LEFT_MOTOR_ROBOT = "left_front_drive";
    private final String ARM_MOTOR_ROBOT = "left_arm";
    private final String WRIST_ROBOT = "wrist";
    private final String INTAKE_ROBOT = "intake";

    private final double FORWARD_SPEED_SIM = 1.0;
    private final double TURN_SPEED_SIM = 1.0;
    private final String RIGHT_MOTOR_SIM = "backRight";
    private final String LEFT_MOTOR_SIM = "backLeft";
    private final String ARM_MOTOR_SIM = "m5";
    private final String WRIST_SIM = "m8";
    private final String INTAKE_SIM = "m6-release sample";

    @Override
    public void runOpMode() {

        backRight = hardwareMap.get(DcMotor.class, USE_SIMULATOR ? RIGHT_MOTOR_SIM : RIGHT_MOTOR_ROBOT);
        backLeft = hardwareMap.get(DcMotor.class, USE_SIMULATOR ? LEFT_MOTOR_SIM : LEFT_MOTOR_ROBOT);
        m5 = hardwareMap.get(DcMotor.class, USE_SIMULATOR ? ARM_MOTOR_SIM : ARM_MOTOR_ROBOT);


        if (USE_SIMULATOR) {
            m8 = hardwareMap.get(DcMotor.class, USE_SIMULATOR ? WRIST_SIM : WRIST_ROBOT);
            m6releasesample = hardwareMap.get(DcMotor.class, USE_SIMULATOR ? INTAKE_SIM : INTAKE_ROBOT);
            leftDrive = hardwareMap.get(DcMotor.class, "frontLeft");
            rightDrive = hardwareMap.get(DcMotor.class, "frontRight");
            leftDrive.setDirection(DcMotorSimple.Direction.FORWARD);
            rightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        } else {
            m8Robot = hardwareMap.get(Servo.class, USE_SIMULATOR ? WRIST_SIM : WRIST_ROBOT);
            m6releasesampleRobot = hardwareMap.get(CRServo.class, USE_SIMULATOR ? INTAKE_SIM : INTAKE_ROBOT);
        }

        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        //Go Straight
        backLeft.setPower(1.0);
        backRight.setPower(1.0);
        while (opModeIsActive() && (runtime.seconds() < 1.0)) {
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        // Turn Right
        backLeft.setPower(-1.0);
        backRight.setPower(1.0);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.5)) {
            telemetry.addData("Path", "Leg 2: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        // Stop - not sure if we need the stop in between each step - but added it
        backLeft.setPower(0);
        backRight.setPower(0);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.1)) {
            telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        //Go Straight
        backLeft.setPower(1.0);
        backRight.setPower(1.0);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.5)) {
            telemetry.addData("Path", "Leg 4: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        // Stop
        backLeft.setPower(0);
        backRight.setPower(0);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.5)) {
            telemetry.addData("Path", "Leg 5: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        // Turn Left
        backLeft.setPower(1.0);
        backRight.setPower(-1.0);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.1)) {
            telemetry.addData("Path", "Leg 6: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        // Stop
        backLeft.setPower(0);
        backRight.setPower(0);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.5)) {
            telemetry.addData("Path", "Leg 7: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        //Go Straight
        backLeft.setPower(1.0);
        backRight.setPower(1.0);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.5)) {
            telemetry.addData("Path", "Leg 8: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        // Stop
        backLeft.setPower(0);
        backRight.setPower(0);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.5)) {
            telemetry.addData("Path", "Leg 9: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        // Turn Right
        backLeft.setPower(-1.0);
        backRight.setPower(1.0);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.5)) {
            telemetry.addData("Path", "Leg 10: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        // Stop
        backLeft.setPower(0);
        backRight.setPower(0);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.5)) {
            telemetry.addData("Path", "Leg 11: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        // Put the arm down  
        m8.setTargetPosition(1);
        m8.setPower(1);

        // Put the wrist into position  
        m5.setTargetPosition(1);
        m5.setPower(1);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 7)) {
            telemetry.addData("Path", "Leg 12: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        m5.setPower(0);
        m8.setPower(0);

        // Turn on motor to pick up sample
        m6releasesample.setPower(1);

        //Go Straight to pick up sample
        backLeft.setPower(1.0);
        backRight.setPower(1.0);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.5)) {
            telemetry.addData("Path", "Leg 13: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        //Stop robot after sample picked up
        backLeft.setPower(0);
        backRight.setPower(00);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.5)) {
            telemetry.addData("Path", "Leg 14: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        m8.setPower(-1);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1)) {
            telemetry.addData("Path", "Leg 15: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        m8.setPower(0);

        // Turn Left
        backLeft.setPower(-1.0);
        backRight.setPower(1.0);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 3.37)) {
            telemetry.addData("Path", "Leg 16: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        //Go Straight
        backLeft.setPower(1.0);
        backRight.setPower(2.0);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 4.0)) {
            telemetry.addData("Path", "Leg 17: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        m8.setPower(-0.5);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1)) {
            telemetry.addData("Path", "Leg 18: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();

        }

        m5.setPower(-0.5);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1)) {
            telemetry.addData("Path", "Leg 19: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
            m6releasesample.setPower(-1);
            // run until the end of the match (driver presses STOP)
            telemetry.addData("Status", "Ending");
            telemetry.update();
        }

    }
}
