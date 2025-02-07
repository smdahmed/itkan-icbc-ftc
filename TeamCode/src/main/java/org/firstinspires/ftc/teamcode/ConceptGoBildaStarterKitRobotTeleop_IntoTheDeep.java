/*   MIT License
 *   Copyright (c) [2024] [Base 10 Assets, LLC]
 *
 *   Permission is hereby granted, free of charge, to any person obtaining a copy
 *   of this software and associated documentation files (the "Software"), to deal
 *   in the Software without restriction, including without limitation the rights
 *   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *   copies of the Software, and to permit persons to whom the Software is
 *   furnished to do so, subject to the following conditions:

 *   The above copyright notice and this permission notice shall be included in all
 *   copies or substantial portions of the Software.

 *   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *   SOFTWARE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

/*
 * This OpMode is an example driver-controlled (TeleOp) mode for the goBILDA 2024-2025 FTC
 * Into The Deep Starter Robot
 * The code is structured as a LinearOpMode
 *
 * This robot has a two-motor differential-steered (sometimes called tank or skid steer) drivetrain.
 * With a left and right drive motor.
 * The drive on this robot is controlled in an "Arcade" style, with the left stick Y axis
 * controlling the forward movement and the right stick X axis controlling rotation.
 * This allows easy transition to a standard "First Person" control of a
 * mecanum or omnidirectional chassis.
 *
 * The drive wheels are 96mm diameter traction (Rhino) or omni wheels.
 * They are driven by 2x 5203-2402-0019 312RPM Yellow Jacket Planetary Gearmotors.
 *
 * This robot's main scoring mechanism includes an arm powered by a motor, a "wrist" driven
 * by a servo, and an intake driven by a continuous rotation servo.
 *
 * The arm is powered by a 5203-2402-0051 (50.9:1 Yellow Jacket Planetary Gearmotor) with an
 * external 5:1 reduction. This creates a total ~254.47:1 reduction.
 * This OpMode uses the motor's encoder and the RunToPosition method to drive the arm to
 * specific setpoints. These are defined as a number of degrees of rotation away from the arm's
 * starting position.
 *
 * Make super sure that the arm is reset into the robot, and the wrist is folded in before
 * you run start the OpMode. The motor's encoder is "relative" and will move the number of degrees
 * you request it to based on the starting position. So if it starts too high, all the motor
 * setpoints will be wrong.
 *
 * The wrist is powered by a goBILDA Torque Servo (2000-0025-0002).
 *
 * The intake wheels are powered by a goBILDA Speed Servo (2000-0025-0003) in Continuous Rotation mode.
 */


@TeleOp(name="FTC Starter Kit Example Robot (INTO THE DEEP)", group="Robot")
//@Disabled
public class ConceptGoBildaStarterKitRobotTeleop_IntoTheDeep extends LinearOpMode {

    /* Declare OpMode members. */
    public DcMotor  leftDrive   = null; //the left drivetrain motor
    public DcMotor  rightDrive  = null; //the right drivetrain motor
    public DcMotor  armMotor    = null; //the arm motor
    public CRServo  intake      = null; //the active intake servo
    // public Servo    wrist       = null; //the wrist servo
    public DcMotor  viperKit    = null; // the viper kit!!!


    /* This constant is the number of encoder ticks for each degree of rotation of the arm.
    To find this, we first need to consider the total gear reduction powering our arm.
    First, we have an external 20t:100t (5:1) reduction created by two spur gears.
    But we also have an internal gear reduction in our motor.
    The motor we use for this arm is a 117RPM Yellow Jacket. Which has an internal gear
    reduction of ~50.9:1. (more precisely it is 250047/4913:1)
    We can multiply these two ratios together to get our final reduction of ~254.47:1.
    The motor's encoder counts 28 times per rotation. So in total you should see about 7125.16
    counts per rotation of the arm. We divide that by 360 to get the counts per degree. */
    final double ARM_TICKS_PER_DEGREE =
            28 // number of encoder ticks per rotation of the bare motor
                    * 250047.0 / 4913.0 // This is the exact gear ratio of the 50.9:1 Yellow Jacket gearbox
                    * 100.0 / 20.0 // This is the external gear reduction, a 20T pinion gear that drives a 100T hub-mount gear
                    * 1/360.0; // we want ticks per degree, not per rotation
    final double VIPER_TICKS_PER_DEGREE =
            28
                    * 13.7
                    * 1/360.0;

    /* These constants hold the position that the arm is commanded to run to.
    These are relative to where the arm was located when you start the OpMode. So make sure the
    arm is reset to collapsed inside the robot before you start the program.

    In these variables you'll see a number in degrees, multiplied by the ticks per degree of the arm.
    This results in the number of encoder ticks the arm needs to move in order to achieve the ideal
    set position of the arm. For example, the ARM_SCORE_SAMPLE_IN_HIGH is set to
    160 * ARM_TICKS_PER_DEGREE. This asks the arm to move 160° from the starting position.
    If you'd like it to move further, increase that number. If you'd like it to not move
    as far from the starting position, decrease it. */

    final double ARM_COLLAPSED_INTO_ROBOT  = 0;
    final double ARM_COLLECT               = 20 * ARM_TICKS_PER_DEGREE; //Changed from 230 --> 30 because of new intake system.
    final double ARM_GET_SAMPLE            = 30 * ARM_TICKS_PER_DEGREE; // Changed so it's easier to pick up samples
    final double ARM_SCORE_SPECIMEN        = 160 * ARM_TICKS_PER_DEGREE;
    final double ARM_SCORE_SAMPLE_IN_HIGH  = 100 * ARM_TICKS_PER_DEGREE;
    final double ARM_ATTACH_HANGING_HOOK   = 140 * ARM_TICKS_PER_DEGREE;
    final double ARM_WINCH_ROBOT           = 15  * ARM_TICKS_PER_DEGREE;
    final double VIPER_OUT                 = (-4 * 360 - 3) * VIPER_TICKS_PER_DEGREE;
    final double ARM_INIT                  = 3 * ARM_TICKS_PER_DEGREE;
    final double VIPER_INIT                = 5 * VIPER_TICKS_PER_DEGREE;

    /* Variables to store the speed the intake servo should be set at to intake, and deposit game elements. */
    final double INTAKE_COLLECT    = -1.0;
    final double INTAKE_OFF        =  0.0;
    final double INTAKE_DEPOSIT    =  0.5;

    /* Variables to store the positions that the wrist should be set to when folding in, or folding out. */
    // final double WRIST_FOLDED_IN   = 0;
    // final double WRIST_FOLDED_OUT  = 0.8;

    /* A number in degrees that the triggers can adjust the arm position by */
    //final double FUDGE_FACTOR = 10 * ARM_TICKS_PER_DEGREE;

    /* Variables that are used to set the arm to a specific position */
    double armPosition = (int) ARM_INIT;


    // Variable used to set the Viper Kit to a specific Position
    double viperPosition = (int) VIPER_INIT;
    //double viperFudgeFactor;

    @Override
    public void runOpMode() {
        /*
        These variables are private to the OpMode, and are used to control the drivetrain.
         */
        double left;
        double right;
        double forward;
        double rotate;
        double max;


        /* Define and Initialize Motors */
        leftDrive = hardwareMap.get(DcMotor.class, "left_front_drive"); //the left drivetrain motor
        rightDrive = hardwareMap.get(DcMotor.class, "right_front_drive"); //the right drivetrain motor
        armMotor = hardwareMap.get(DcMotor.class, "left_arm"); //the arm motor
        viperKit = hardwareMap.get(DcMotor.class, "viper_kit");


        /* Most skid-steer/differential drive robots require reversing one motor to drive forward.
        for this robot, we reverse the right motor.*/
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);


        /* Setting zeroPowerBehavior to BRAKE enables a "brake mode". This causes the motor to slow down
        much faster when it is coasting. This creates a much more controllable drivetrain. As the robot
        stops much quicker. */
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        viperKit.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        /*This sets the maximum current that the control hub will apply to the arm and Viper Kit before throwing a flag */
        ((DcMotorEx) armMotor).setCurrentAlert(5,CurrentUnit.AMPS);
        ((DcMotorEx) viperKit).setCurrentAlert(5, CurrentUnit.AMPS);


        /* Before starting the armMotor. We'll make sure the TargetPosition is set to 40. Changed to 40 because arm kept trying to move unnecessarily
        Then we'll set the RunMode to RUN_TO_POSITION. And we'll ask it to stop and reset encoder.
        If you do not have the encoder plugged into this motor, it will not run in this code. */
        armMotor.setTargetPosition((int) armPosition);
        telemetry.addLine("Ran arm initialization successfully");
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Same for Viper Kit.
        // Setting TargetPosition to 0, setting runMode to RUN_TO_POSITION. Also asking it to stop and reset encoder
        viperKit.setTargetPosition((int) viperPosition);
        viperKit.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        viperKit.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        viperKit.setPower(0.5);

        /* Define and initialize servos.*/
        intake = hardwareMap.get(CRServo.class, "intake");
        // wrist  = hardwareMap.get(Servo.class, "wrist");

        /* Make sure that the intake is off, and the wrist is folded in. */
        intake.setPower(INTAKE_OFF);
        // wrist.setPosition(WRIST_FOLDED_IN);

        /* Send telemetry message to signify robot waiting */
        telemetry.addLine("Robot Ready.");
        telemetry.update();
        /* Wait for the game driver to press play */
        waitForStart();
        // Set the velocity of the motor and use setMode to run
//        ((DcMotorEx) viperKit).setVelocity(1600);
        /* Run until the driver presses stop */
        while (opModeIsActive()) {

            /* Set the drive and turn variables to follow the joysticks on the gamepad.
            the joysticks decrease as you push them up. So reverse the Y axis. */
            forward = -gamepad1.left_stick_y;
            rotate  = gamepad1.right_stick_x; // Flipped sticks to optimize for driver.

            if (forward > 0.5) {
                forward = 0.5;
            }


            /* Here we "mix" the input channels together to find the power to apply to each motor.
            The both motors need to be set to a mix of how much you're retesting the robot move
            forward, and how much you're requesting the robot turn. When you ask the robot to rotate
            the right and left motors need to move in opposite directions. So we will add rotate to
            forward for the left motor, and subtract rotate from forward for the right motor. */

            left  = (forward + rotate);
            right = (forward - rotate);

            /* Normalize the values so neither exceed +/- 0.6 */
            // Changed from 1.0 -> 0.8 to allow for more precision driving (boost button will be for faster driving)
            max = Math.max(Math.abs(left), Math.abs(right));
            if (max > 1)
            {
                telemetry.addData("max_val: ", max);
                left /= max;
                right /= max;
            }

            /* Set the motor power to the variables we've mixed and normalized */
            leftDrive.setPower(left);
            rightDrive.setPower(right);



            /* Here we handle the three buttons that have direct control of the intake speed.
            These control the continuous rotation servo that pulls elements into the robot,
            If the user presses A, it sets the intake power to the final variable that
            holds the speed we want to collect at.
            If the user presses X, it sets the servo to Off.
            And if the user presses B it reveres the servo to spit out the element.*/

            /* TECH TIP: If Else statements:
            We're using an else if statement on "gamepad1.x" and "gamepad1.b" just in case
            multiple buttons are pressed at the same time. If the driver presses both "a" and "x"
            at the same time. "a" will win over and the intake will turn on. If we just had
            three if statements, then it will set the intake servo's power to multiple speeds in
            one cycle. Which can cause strange behavior. */

            //ANYTHING related to arm/intake will be set to gamepad2 so operator can have seperate controller

            if (gamepad2.a) {
                intake.setPower(INTAKE_COLLECT);
            }
            else if (gamepad2.x) {
                intake.setPower(INTAKE_OFF);
            }
            else if (gamepad2.b) {
                intake.setPower(INTAKE_DEPOSIT);
            }
            //Boost Button (WIP)
            if (gamepad1.right_trigger > 0.0) {
                leftDrive.setPower(1.0);
                rightDrive.setPower(1.0);

            }

            /* Here we implement a set of if else statements to set our arm to different scoring positions.
            We check to see if a specific button is pressed, and then move the arm (and sometimes
            intake and wrist) to match. For example, if we click the right bumper we want the robot
            to start collecting. So it moves the armPosition to the ARM_COLLECT position,
            it folds out the wrist to make sure it is in the correct orientation to intake, and it
            turns the intake on to the COLLECT mode.*/

            if(gamepad2.right_bumper){
                /* This is the setup for intaking from the submersible. Press left trigger to extend and pick up samples from submersible */
                armPosition = ARM_COLLECT;
                // wrist.setPosition(WRIST_FOLDED_OUT);
                intake.setPower(INTAKE_COLLECT);
            }

            else if (gamepad2.left_bumper){
                /* This is about 20° up from the collecting position to clear the barrier
                Note here that we don't set the wrist position or the intake power when we
                select this "mode", this means that the intake and wrist will continue what
                they were doing before we clicked left bumper. */
                // Changed the functionality to tilt down from right_bumper position so it's easier to pick up samples from submersible
                armPosition = ARM_GET_SAMPLE;
            }

            else if (gamepad2.y){
                /* This is the correct height to score the sample in the LOW BASKET */
                armPosition = ARM_SCORE_SAMPLE_IN_HIGH;
            }

            else if (gamepad2.dpad_left) {
                /* This turns off the intake, folds in the wrist, and moves the arm
                back to folded inside the robot. This is also the starting configuration */
                armPosition = ARM_INIT;
                intake.setPower(INTAKE_OFF);
                // wrist.setPosition(WRIST_FOLDED_IN);
                viperPosition = VIPER_INIT;
            }

            else if (gamepad2.dpad_up){
                /* This sets the arm to vertical to hook onto the LOW RUNG for hanging */
                armPosition = ARM_ATTACH_HANGING_HOOK;
                intake.setPower(INTAKE_OFF);
                // wrist.setPosition(WRIST_FOLDED_IN);
            }

            else if (gamepad2.dpad_down){
                /* this moves the arm down to lift the robot up once it has been hooked */
                armPosition = ARM_WINCH_ROBOT;
                intake.setPower(INTAKE_OFF);
                // wrist.setPosition(WRIST_FOLDED_IN);
            }
            else if (gamepad2.left_trigger > 0.0){
                // Extends the viper kit out
                viperPosition = VIPER_OUT;
                if (armPosition == ARM_INIT){
                    viperPosition = 0;
                    viperKit.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                }
            }
            else if (gamepad2.right_trigger > 0.0){
                // Retracts the viper kit back in
                viperPosition = 0;
            }
            else if (gamepad2.dpad_right){
                viperPosition = VIPER_INIT;
                viperKit.setPower(0.5);
                viperKit.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }
            //Endgame Auto Hang (By pressing PS Central Button):
            else if (gamepad1.guide){
                autoHang();
                intake.setPower(INTAKE_OFF);
            }

            else if (gamepad2.left_stick_button){
                inSub();
                intake.setPower(INTAKE_COLLECT);
                sleep(500);
                armMotor.setTargetPosition((int) ARM_COLLECT);
            }

            else if (gamepad2.right_stick_button){
                outSub();
            }


            /* Here we create a "fudge factor" for the arm position.
            This allows you to adjust (or "fudge") the arm position slightly with the gamepad triggers.
            We want the left trigger to move the arm up, and right trigger to move the arm down.
            So we add the right trigger's variable to the inverse of the left trigger. If you pull
            both triggers an equal amount, they cancel and leave the arm at zero. But if one is larger
            than the other, it "wins out". This variable is then multiplied by our FUDGE_FACTOR.
            The FUDGE_FACTOR is the number of degrees that we can adjust the arm by with this function. */

            //viperFudgeFactor = FUDGE_FACTOR * (gamepad2.right_trigger + (-gamepad2.left_trigger));

            /* Here we set the target position of our arm to match the variable that was selected
            by the driver.
            We also set the target velocity (speed) the motor runs at, and use setMode to run it.*/

            armMotor.setTargetPosition((int) (armPosition));
            // Reduced arm velocity so it wouldn't jitter when moving
            ((DcMotorEx) armMotor).setVelocity(1600);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Set the target position to the position the driver asked for
            viperKit.setTargetPosition((int) (viperPosition));
            // Set the velocity of the motor and use setMode to run
            ((DcMotorEx) viperKit).setVelocity(1600);
            viperKit.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            /* TECH TIP: Encoders, integers, and doubles
            Encoders report when the motor has moved a specified angle. They send out pulses which
            only occur at specific intervals (see our ARM_TICKS_PER_DEGREE). This means that the
            position our arm is currently at can be expressed as a whole number of encoder "ticks".
            The encoder will never report a partial number of ticks. So we can store the position in
            an integer (or int).
            A lot of the variables we use in FTC are doubles. These can capture fractions of whole
            numbers. Which is great when we want our arm to move to 122.5°, or we want to set our
            servo power to 0.5.

            setTargetPosition is expecting a number of encoder ticks to drive to. Since encoder
            ticks are always whole numbers, it expects an int. But we want to think about our
            arm position in degrees. And we'd like to be able to set it to fractions of a degree.
            So we make our arm positions Doubles. This allows us to precisely multiply together
            armPosition and our armPositionFudgeFactor. But once we're done multiplying these
            variables. We can decide which exact encoder tick we want our motor to go to. We do
            this by "typecasting" our double, into an int. This takes our fractional double and
            rounds it to the nearest whole number.
            */

            /* Check to see if our arm is over the current limit, and report via telemetry. */
            if (((DcMotorEx) armMotor).isOverCurrent()){
                telemetry.addLine("MOTOR EXCEEDED CURRENT LIMIT!");
            }


            /* send telemetry to the driver of the arm's current position and target position */
            telemetry.addData("armTarget: ", armMotor.getTargetPosition());
            telemetry.addData("arm Encoder: ", armMotor.getCurrentPosition());
            telemetry.addData("viperKitTarget: ", viperKit.getTargetPosition());
            telemetry.addData("viperKitCurrPosition: ", viperKit.getCurrentPosition());
            telemetry.update();

        }
    }
        public void autoHang(){
        armMotor.setTargetPosition((int) (ARM_ATTACH_HANGING_HOOK));
        // Reduced arm velocity so it wouldn't jitter when moving
        ((DcMotorEx) armMotor).setVelocity(2100);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        intake.setPower(INTAKE_OFF);
        sleep(1500);
        armMotor.setTargetPosition((int) (ARM_WINCH_ROBOT));
        // Reduced arm velocity so it wouldn't jitter when moving
        ((DcMotorEx) armMotor).setVelocity(2100);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftDrive.setPower(1.0);
        rightDrive.setPower(1.0);
        sleep(1000);
        leftDrive.setPower(0);
        rightDrive.setPower(0);
    }

    public void inSub(){
        // Lift arm
        armMotor.setTargetPosition((int) (ARM_GET_SAMPLE)); // Reduced arm velocity so it wouldn't jitter when moving
        ((DcMotorEx) armMotor).setVelocity(1600);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        intake.setPower(INTAKE_OFF);
        sleep(1500);

        // intake on
        intake.setPower(INTAKE_COLLECT);
        //Extend Viper Kit in submersible
        viperPosition = VIPER_OUT;
        viperKit.setTargetPosition((int) VIPER_OUT);
        ((DcMotorEx) viperKit).setVelocity(1600);
        viperKit.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Bring arm down to intake better
        intake.setPower(INTAKE_COLLECT);
        sleep(500);
        armMotor.setTargetPosition((int) ARM_COLLECT);
    }

    public void outSub(){
        viperPosition = 0;
    }
}
