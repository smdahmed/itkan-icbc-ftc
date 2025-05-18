package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.EncoderInterface.EncoderInterface;

@Autonomous(name = "HangSampleAndPark", group = "Robot")
public class HangSampleAndPark extends LinearOpMode {
    static final double ARM_TICKS_PER_DEGREE =
            28 // number of encoder ticks per rotation of the bare motor
                    * 250047.0 / 4913.0 // This is the exact gear ratio of the 50.9:1 Yellow Jacket gearbox
                    * 100.0 / 20.0 // This is the external gear reduction, a 20T pinion gear that drives a 100T hub-mount gear
                    * 1/360.0; // we want ticks per degree, not per rotation
    public static final Object ARM_SCORE_SAMPLE_IN_HIGH = 120 * ARM_TICKS_PER_DEGREE;
    public DcMotorEx backRight;
    public DcMotorEx backLeft;

    public DcMotorEx arm;
    public Servo claw        = null;

    public CRServo robotSampleServo;
    public DcMotorEx viperKit;

    //Ticks is motor rate * the ratio provided by Gobilda.
    public final double ticks = 537.7;
    //Circumference is based on the wheel size.
    public final double circumference = 301.59;
    final double clawClosed                  = claw.MAX_POSITION;
    final double clawOpen                = claw.MIN_POSITION;

    @Override
    public void runOpMode() {
        EncoderInterface encoderInterface = new EncoderInterface(this);
        encoderInterface.initializeDevices();
        encoderInterface.setupEncoders();
        waitForStart();
        sleep(5000);

        //Start with sample in basket.
        encoderInterface.moveArmUp();
        encoderInterface.goStraight(-38);
        claw.setPosition(clawOpen);
        //encoderInterface.extendViperKit(-55);
        encoderInterface.moveArmDown(50);
        encoderInterface.goBackwards(25);
        //encoderInterface.extendViperKit(55);
        claw.setPosition(clawClosed);
        ///encoderInterface.extendViperKit(0);
        viperKit.setPower(0);
        sleep(500);
        //Park after hanging sample
        //encoderInterface.turnRight((int) -65);
        encoderInterface.goStraight(40);
//        encoderInterface.goStraight(95);
//        encoderInterface.turnLeft((int) 45);
//        encoderInterface.goBackwards(38);
//        telemetry.addData("Status", "Ended");
    }
}
