package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by SmartZalmover on 12/5/16.
 * MainTeleOpMode for #10788
 * @version verison 1.00
 * @author Alik Zalmover
 */

@TeleOp (name = "MainTeleOp")
public class MyRobot extends LinearOpMode {

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
    public static final double POWER_STOP = 0.0;
    public static final double POWER_SCORER = 0.1;
    public static final double POWER_REVOLVE = 0.7;

    @Override
    public void runOpMode() throws  InterruptedException {

        ElapsedTime opmodeRunTime = new ElapsedTime();
        telemetry.log().setDisplayOrder(Telemetry.Log.DisplayOrder.NEWEST_FIRST);

        motorLeft = hardwareMap.dcMotor.get("motorLeft");
        motorRight = hardwareMap.dcMotor.get("motorRight");
        motorSweeper = hardwareMap.dcMotor.get("motorSweeper");
        motorRevolve = hardwareMap.dcMotor.get("motorRevolve");
        servoArm = hardwareMap.servo.get("servoArm");


        // keeps the car going straight
        motorLeft.setDirection(DcMotor.Direction.REVERSE);

        // init servo position
        servoArm.setPosition(ARM_LOWER_POSITION);

        int loopCount = 1;


        waitForStart();

        while(opModeIsActive()) {

            double leftPowerUse = -gamepad1.left_stick_y;
            double rightPowerUse = -gamepad1.right_stick_y;
            double sweepPowerUse = -gamepad2.left_stick_y;
            double revolvePowerUse = motorRevolve.getPower();
            double servoPosition = servoArm.getPosition();

            // reversed - wierd!
            motorLeft.setPower(leftPowerUse );
            motorRight.setPower(rightPowerUse);

            // sets power to different motors depending on the buttons pressed

            // servoArms forward or back
            if(gamepad2.a)
                servoArm.setPosition(ARM_UPPER_POSITION);
            if(gamepad2.b)
                servoArm.setPosition(ARM_LOWER_POSITION);

            // sweeper (getting the ball)
            if(gamepad2.x)
                motorSweeper.setPower(POWER_SCORER);
            if(gamepad2.y)
                motorSweeper.setPower(-POWER_SCORER);
            if(gamepad2.right_bumper)
                motorSweeper.setPower(POWER_STOP);

            // revolver (Throwing the ball?)
            if(gamepad1.left_bumper) {
                setPowerTime(-POWER_FULL,1000);
                setPower(-0);

            }



            telemetry.addData("Data", "*** Robot Data***");
            // As an illustration, show some loop timing information
            telemetry.addData("loop count", loopCount);
            telemetry.addData("ms/loop", "%.3f ms", opmodeRunTime.milliseconds() / loopCount);
            // Show Robot Data Values
            telemetry.addData("left pwr", "left pwr: " + String.format("%.2f", leftPowerUse));
            telemetry.addData("right pwr", "right pwr: " + String.format("%.2f", rightPowerUse));
            telemetry.addData("revolve pwr", "revolve pwr: " + String.format("%.2f", revolvePowerUse));
            telemetry.addData("sweep pwr", "sweep pwr: " + String.format("%.2f", sweepPowerUse));
            telemetry.addData("servoArm pwr", "servoArm pwr: " + String.format("%.2f", servoPosition));


            // Updates values for the whole match
            telemetry.update();

            loopCount++;

            idle();

        }

    }


    public void setPower(double power) {

        motorRevolve.setPower(power);

    }

    public void setPowerTime(double power, long time) throws InterruptedException {

        setPower(power);
        Thread.sleep(time);

    }

}
