package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by SmartZalmover on 12/5/16.
 * Autonomous Mode for 10788
 */

@Autonomous(name = "AutoMode", group = "AutoModes")
public class AutoMode extends LinearOpMode {

    // motors
    private DcMotor motorLeft;
    private DcMotor motorRight;
    private DcMotor motorSweeper;
    private DcMotor motorRevolve;

    // servos
    private Servo   servoArm;


    // constants

    public static final double ARM_UPPER_POSITION = 0.8;
    public static final double ARM_LOWER_POSITION = 0.2;


    // power of motor
    public static final double POWER_FULL = 1.0;
    public static final double POWER_SLOW = 0.2;
    public static final double POWER_STOP = 0.0;
    public static final double POWER_SCORER = 0.4;

    @Override
    public void runOpMode() throws InterruptedException {

        motorLeft = hardwareMap.dcMotor.get("motorLeft");
        motorRight = hardwareMap.dcMotor.get("motorRight");
        motorSweeper = hardwareMap.dcMotor.get("motorSweeper");
        motorRevolve = hardwareMap.dcMotor.get("motorRevolve");
        servoArm = hardwareMap.servo.get("servoArm");


        // keeps the car going straight
        motorLeft.setDirection(DcMotor.Direction.REVERSE);

        // init servo position
        raiseArm();

        // DRIVE
        // // TODO: 12/6/16 -
        // // TODO: implement all methods matching the challenge

        driveForwardTime(POWER_SLOW,4000);

        waitForStart();



    }

    public void driveForward(double power) {

        motorLeft.setPower(power);
        motorRight.setPower(power);

    }

    public void turnLeft(double power) {

        motorLeft.setPower(-power);
        motorRight.setPower(power);
    }

    public  void turnRight(double power) {

        turnLeft(-power);

    }

    public void driveForwardTime(double power, long time) throws InterruptedException {

        driveForward(power);
        Thread.sleep(time);

    }

    public void turnLeftTime(double power, long time) throws InterruptedException {

        turnLeft(power);
        Thread.sleep(time);

    }

    public  void turnRightTime(double power, long time) throws InterruptedException {

        turnRight(power);
        Thread.sleep(time);

    }

    public void stopDrive() {

        driveForward(0);

    }

    public void raiseArm() {

        servoArm.setPosition(0.8);
    }

    public void lowerArm() {

        servoArm.setPosition(0.2);

    }
}
