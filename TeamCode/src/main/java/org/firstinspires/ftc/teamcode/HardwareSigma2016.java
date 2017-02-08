package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 */

public class HardwareSigma2016
{
    public int groundbrightness_test1=0;
    public int groundbrightness_test2=0;
    public int groundbrightness_test3=0;
    public int groundbrightnessAVG = 0;
    public final double CENTER_LIGHT_THRESH = 3.0;

    // speed
    public double targetAngleSpeed = 0;
    public double targetSpeed = 0;
    public double currentAngleSpeed = 0;
    public double currentSpeed = 0;

    /* Public OpMode members. */
    public DcMotor  backLeftMotor = null;
    public DcMotor  backRightMotor = null;
    public DcMotor  frontLeftMotor = null;
    public DcMotor  frontRightMotor = null;
    public DcMotor  flicker = null;
    public DcMotor  intake = null;
    public Servo    pusherL    = null;
    public Servo    pusherR   = null;
    public ColorSensor lineLightSensor = null;
    public ColorSensor beaconColorSensor = null;
    public UltrasonicSensor ultra_front = null;
    public UltrasonicSensor ultra_back = null;

    public static final double PUSHER_L_IN  =  1.0 ;
    public static final double PUSHER_R_IN  =  0.0 ;
    public static final double PUSHER_L_OUT  =  0.0 ;
    public static final double PUSHER_R_OUT  =  1.0 ;
    public static final double PUSHER_STOP = 0.5;

    public static final int ENCODER_TARGET_THRESHOLD = 10;

    ModernRoboticsI2cGyro gyro = null;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HardwareSigma2016(){
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) throws InterruptedException {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        frontLeftMotor = hwMap.dcMotor.get("motor_2");
        frontRightMotor  = hwMap.dcMotor.get("motor_3");
        backLeftMotor  = hwMap.dcMotor.get("motor_1");
        backRightMotor = hwMap.dcMotor.get("motor_4");

        flicker = hwMap.dcMotor.get("flicker");
        intake = hwMap.dcMotor.get("intake");

        frontLeftMotor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        backLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        backRightMotor.setDirection(DcMotor.Direction.REVERSE);

        // Set all motors to zero power
        backRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);

        // Define and initialize ALL installed servos.
        pusherL = hwMap.servo.get("pusher_l");
        pusherR = hwMap.servo.get("pusher_r");
        pusherL.setPosition(PUSHER_L_IN);
        pusherR.setPosition(PUSHER_R_IN);
        Thread.sleep(1300);
        pusherL.setPosition(PUSHER_STOP);
        pusherR.setPosition(PUSHER_STOP);

        // gyro
        gyro = (ModernRoboticsI2cGyro) hwMap.gyroSensor.get("gyro");

        // light sensor on the robot bottom
        lineLightSensor = hwMap.colorSensor.get("line_light");
        lineLightSensor.enableLed(true);
        try {
            Thread.sleep(1000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        groundbrightness_test1 = lineLightSensor.red() + lineLightSensor.green() + lineLightSensor.blue();

        // color sensor on beacon pusher
        beaconColorSensor = hwMap.colorSensor.get("beacon_color");
        beaconColorSensor.enableLed(false);

        // ultrasonic sensor
        ultra_back = hwMap.ultrasonicSensor.get("ultra_back");
        ultra_back.getUltrasonicLevel();  // make ultrasonic sensor ready and give stable output when needed.
        ultra_front = hwMap.ultrasonicSensor.get("ultra_front");
        ultra_front.getUltrasonicLevel(); // make ultrasonic sensor ready and give stable output when needed.

        System.out.println("Sigma2016 -- hardware is initialized!");
    }

    /***
     *
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs  Length of wait cycle in mSec.
     */
    public void waitForTick(long periodMs) {

        long  remaining = periodMs - (long)period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0) {
            try {
                Thread.sleep(remaining);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }

        // Reset the cycle clock for the next pass.
        period.reset();
    }
}

