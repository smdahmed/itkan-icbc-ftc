package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Autonomous(name = "HangSampleAndPark", group = "Robot")
public class HangSampleAndPark extends LinearOpMode {
    public DcMotorEx backRight;
    public DcMotorEx backLeft;

    public DcMotorEx arm;

    public CRServo robotSampleServo;
    public DcMotorEx viperKit;

    //Ticks is motor rate * the ratio provided by Gobilda.
    public final double ticks = 537.7;
    //Circumference is based on the wheel size.
    public final double circumference = 301.59;
    public final double ARM_TICKS_PER_DEGREE =
            28 // number of encoder ticks per rotation of the bare motor
                    * 250047.0 / 4913.0 // This is the exact gear ratio of the 50.9:1 Yellow Jacket gearbox
                    * 100.0 / 20.0 // This is the external gear reduction, a 20T pinion gear that drives a 100T hub-mount gear
                    * 1 / 360.0; // we want ticks per degree, not per rotation

    final double ARM_SCORE_SAMPLE_IN_HIGH = 100 * ARM_TICKS_PER_DEGREE;

    @Override
    public void runOpMode() {
        EncoderInterface encoderInterface = new EncoderInterface(this);
        encoderInterface.initializeDevices();
        encoderInterface.setupEncoders();
        waitForStart();

        //Start with sample in basket.
        encoderInterface.moveArmUp();
        encoderInterface.goStraight(38);
        robotSampleServo.setPower(-1);
        encoderInterface.extendViperKit(-55);
        encoderInterface.moveArmDown(50);
        encoderInterface.goBackwards(25);
        encoderInterface.extendViperKit(55);
        robotSampleServo.setPower(0);
        encoderInterface.extendViperKit(0);
        viperKit.setPower(0);

        //Park after hanging sample.
        encoderInterface.goStraightVelocity(20);
        encoderInterface.turnRightVelocity((int) 50); //Value needs to be adjusted after testing
        encoderInterface.goStraightVelocity(95);
        encoderInterface.turnLeft((int) 45);
        encoderInterface.goBackwards(38); //Value needs to be adjusted or line needs to be modified after testing

        telemetry.addData("Status", "Ended");
    }
}
