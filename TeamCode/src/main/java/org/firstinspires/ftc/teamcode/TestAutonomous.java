/*
Copyright 2021 FIRST Tech Challenge Team 0000

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
associated documentation files (the "Software"), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge, publish, distribute,
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial
portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.SensorSparkFunOTOS;

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

@Autonomous(name = "Robot: Test Auto", group = "Robot")
public class TestAutonomous extends LinearOpMode {

    /* Declare OpMode members. */
    private DcMotor backRight = null;
    private DcMotor backLeft = null;

    private Servo m8;
    private DcMotor m5;
    private CRServo m6releasesample;

    private ElapsedTime runtime = new ElapsedTime();

    private static final boolean USE_SIMULATOR = false;

    private static final double FORWARD_SPEED_ROBOT = 0.4;
    private static final double TURN_SPEED_ROBOT = 0.5;
    private static final String RIGHT_MOTOR_ROBOT = "front_right_motor";
    private static final String LEFT_MOTOR_ROBOT = "front_left_motor";
    private static final String ARM_MOTOR_ROBOT = "left_arm";
    private static final String WRIST_ROBOT = "wrist";
    private static final String INTAKE_ROBOT = "intake";

    private static final double FORWARD_SPEED_SIM = 1.0;
    private static final double TURN_SPEED_SIM = 1.0;
    private static final String RIGHT_MOTOR_SIM = "frontRight";
    private static final String LEFT_MOTOR_SIM = "frontLeft";
    private static final String ARM_MOTOR_SIM = "m5";
    private static final String WRIST_SIM = "m8";
    private static final String INTAKE_SIM = "m6";
    SparkFunOTOS myOtos;
    SensorInterface sensorInterface;

    @Override
    public void runOpMode() {
        SensorInterface sensor = new SensorInterface();
        sensor.configureOtos(telemetry, myOtos);
        setupDevices(USE_SIMULATOR);
        telemetry.addData("Status", "Running");

        // Log the position to the telemetry
        logSensorData();

        // Update the telemetry on the driver station
        telemetry.update();

        drive(USE_SIMULATOR);

        telemetry.addData("Sensor status:", myOtos.getConnectionInfo());
        telemetry.addData("Sensor connection status:", myOtos.isConnected());

        telemetry.addData("Status", "Initialized");
        telemetry.update();

    }

    public void setupDevices(boolean useSimulator) {
        backRight = hardwareMap.get(DcMotor.class, useSimulator ? RIGHT_MOTOR_SIM : RIGHT_MOTOR_ROBOT);
        backLeft = hardwareMap.get(DcMotor.class, useSimulator ? LEFT_MOTOR_SIM : LEFT_MOTOR_ROBOT);
        myOtos = hardwareMap.get(SparkFunOTOS.class, "sensor_otos");

        m8 = hardwareMap.get(Servo.class, useSimulator ? WRIST_SIM : WRIST_ROBOT);
        m5 = hardwareMap.get(DcMotor.class, useSimulator ? ARM_MOTOR_SIM : ARM_MOTOR_ROBOT);
        m6releasesample = hardwareMap.get(CRServo.class, useSimulator ? INTAKE_SIM : INTAKE_ROBOT);

        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);

        myOtos.calibrateImu();
        myOtos.resetTracking();
        telemetry.addLine();
    }

    public void stopMoving() {
        backLeft.setPower(0);
        backRight.setPower(0);
        sleep(100);
    }

    public void turnRight(double speed) {
        backLeft.setPower(-speed);
        backRight.setPower(speed);
        sleep(1400);
    }

    public void turnLeft(double speed) {
        backLeft.setPower(speed);
        backRight.setPower(-speed);
        sleep(1000);
    }

    public void goStraight(double speed) {
        backLeft.setPower(speed);
        backRight.setPower(speed);
        sleep(400);


    }

    public void drive(boolean useSimulator) {
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Turn Right
        double turnRightSpeed = useSimulator ? TURN_SPEED_SIM : TURN_SPEED_ROBOT;
        turnRight(turnRightSpeed);
        // Log the position to the telemetry
        logSensorData();

        // Update the telemetry on the driver station
        telemetry.update();

        // Stop - not sure if we need the stop in between each step - but added it
        stopMoving();

        //Go Straight
        goStraight(turnRightSpeed);
        sleep(400);

        // Stop
        stopMoving();

        // Turn Left
        turnLeft(turnRightSpeed);
        logSensorData();

        // Update the telemetry on the driver station
        telemetry.update();

        // Stop
        stopMoving();

        //Go Straight
        goStraight(turnRightSpeed);
        sleep(1400);
        logSensorData();


        // Update the telemetry on the driver station
        telemetry.update();

        // Stop
        stopMoving();
        logSensorData();


        // Update the telemetry on the driver station
        telemetry.update();

        // Turn Right
        turnRight(turnRightSpeed);
        logSensorData();


        // Update the telemetry on the driver station
        telemetry.update();

        // Stop
        stopMoving();

        // Put the arm down
        m8.setPosition(1.0);
        sleep(4000);

        // TODO - figure out how to intake the sample

        // TODO - add steps to go to the basket and release the sample.

        // Stop
        stopMoving();
    }


    public void logSensorData() {
        SparkFunOTOS.Pose2D pos = myOtos.getPosition();
        telemetry.addData("X coordinate", pos.x);
        telemetry.addData("Y coordinate", pos.y);
        telemetry.addData("Heading angle", pos.h);
    }
}
