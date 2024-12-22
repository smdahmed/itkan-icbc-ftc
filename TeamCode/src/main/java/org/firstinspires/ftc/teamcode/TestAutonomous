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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a PushBot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Remove a @Disabled the on the next line or two (if present) to add this opmode to the Driver Station OpMode list,
 * or add a @Disabled annotation to prevent this OpMode from being added to the Driver Station
 */

@Autonomous(name="Robot: Test Auto", group="Robot")
public class TestAutonomous extends LinearOpMode {
  
        /* Declare OpMode members. */
   // private DcMotor         leftDrive   = null;
    //private DcMotor         rightDrive  = null;
    private DcMotor         backRight   = null;
    private DcMotor         backLeft    = null;
  	  //    private DcMotor intakeMotor;

    private DcMotor m8;
    private DcMotor m5;
    private DcMotor m6releasesample;
  
    private ElapsedTime     runtime = new ElapsedTime();


    static final double     FORWARD_SPEED = 0.4;
    static final double     TURN_SPEED    = 0.5;


    @Override
    public void runOpMode() {
      
    //intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
    //intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);  
    //leftDrive  = hardwareMap.get(DcMotor.class, "frontLeft");
    //rightDrive = hardwareMap.get(DcMotor.class, "frontRight");
    backRight = hardwareMap.get(DcMotor.class, "right_front_drive");
    backLeft = hardwareMap.get(DcMotor.class, "left_front_drive");
      
    m8 = hardwareMap.get(DcMotor.class, "m8");
    m5 = hardwareMap.get(DcMotor.class, "m5");
    m6releasesample = hardwareMap.get(DcMotor.class, "m6-release sample");
      
      
    //leftDrive.setDirection(DcMotorSimple.Direction.FORWARD);  
    //rightDrive.setDirection(DcMotorSimple.Direction.REVERSE);  

    backRight.setDirection(DcMotorSimple.Direction.REVERSE);  
    backLeft.setDirection(DcMotorSimple.Direction.FORWARD);

    telemetry.addData("Status", "Initialized");
    telemetry.update();

    // Wait for the game to start (driver presses PLAY)
    waitForStart();
    
    // Turn Right
    backLeft.setPower(-1.0);
    backRight.setPower(1.0);
    sleep(1400);

    // Stop - not sure if we need the stop in between each step - but added it
    backLeft.setPower(0);
    backRight.setPower(0);
    sleep(100)  
      
    //Go Straight
    backLeft.setPower(1.0);
    backRight.setPower(1.0);
    sleep(400);  

    // Stop
    backLeft.setPower(0);
    backRight.setPower(0);
    sleep(100)  
      
    // Turn Left
    backLeft.setPower(1.0);
    backRight.setPower(-1.0);
    sleep(1000);

     // Stop
    backLeft.setPower(0);
    backRight.setPower(0);
    sleep(100)  
 
    //Go Straight
    backLeft.setPower(1.0);
    backRight.setPower(1.0);
    sleep(1400);  

    // Stop
    backLeft.setPower(0);
    backRight.setPower(0);
    sleep(100)  
     
    // Turn Right
    backLeft.setPower(-1.0);
    backRight.setPower(1.0);
    sleep(1400);

    // Stop
    backLeft.setPower(0);
    backRight.setPower(0);
    sleep(100)  
    
    // Put the arm down  
    m8.setPower(1.0);
    sleep(4000)

    // TODO - figure out how to intake the sample
      
    // TODO - add steps to go to the basket and release the sample.
      
    // Stop
    backLeft.setPower(0);
    backRight.setPower(0);  
       
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Status", "Running");
            telemetry.update();

        }
    }
}
