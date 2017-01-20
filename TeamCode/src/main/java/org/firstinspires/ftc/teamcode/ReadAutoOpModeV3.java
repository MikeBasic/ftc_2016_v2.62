package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.InterruptedIOException;
import java.util.ArrayList;

import android.os.Environment;
import android.util.Log;
import android.widget.TextView;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER;
import static org.firstinspires.ftc.teamcode.HardwareSigma2016.PUSHER_L_IN;
import static org.firstinspires.ftc.teamcode.HardwareSigma2016.PUSHER_L_OUT;
import static org.firstinspires.ftc.teamcode.HardwareSigma2016.PUSHER_R_IN;
import static org.firstinspires.ftc.teamcode.HardwareSigma2016.PUSHER_R_OUT;
import static org.firstinspires.ftc.teamcode.HardwareSigma2016.PUSHER_STOP;

/**
 * Created by sidharth on 10/26/16.
 */

@Autonomous(name="Reader Tele Op - v3", group="Sigma6710")


public class ReadAutoOpModeV3 extends LinearOpMode{

    FileInputStream is;
    BufferedReader reader;
    final File file = new File(Environment.getExternalStorageDirectory() + File.separator + "Auto/Report Files/file.txt");
    String fullFile;
    int pos;
    ArrayList<String> commands = new ArrayList<>();
    String arrayItem;
    double leftThrottle;
    double rightThrottle;
    double gyroVal;
    double leftInch;
    double rightInch;
    double gyroTotal = 0;
    HardwareSigma2016 robot = null;
    ModernRoboticsI2cGyro gyro = null;

    static final double COUNTS_PER_MOTOR_REV = 2250;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 5.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    // These constants define the desired driving/control characteristics
    // The can/should be tweaked to suite the specific robot drive train.
    static final double DRIVE_SPEED = 0.8;     // Nominal speed for better accuracy.
    static final double TURN_SPEED = 0.6;     // Nominal half speed for better accuracy.
    static final double WALL_APPROACHING_SPEED = 0.3;
    static final double WALL_TRACKING_SPEED = 0.06;
    static final double WALL_TRAVELING_SPEED = 0.1;

    static final double HEADING_THRESHOLD = 2;      // As tight as we can make it with an integer gyro
    static final double P_TURN_COEFF = 0.5;     // Larger is more responsive, but also less stable
    static final double P_DRIVE_COEFF = 0.15;     // Larger is more responsive, but also less stable
    static final double P_WALL_TRACKING_COEFF = 0.1;     // Larger is more responsive, but also less stable

    static final double TARGET_WALL_DISTANCE = 13.0;  // ultrasound sensor reading for x inch away from wall
    static final double WALL_DISTANCE_THRESHOLD = 1.0; // no need to adjust if wall distance is within range
    static final double WALL_TRACKING_MAX_HEADING_OFFSET = 3;

    static final int RED_TRESHOLD = 5;
    static final int BLUE_TRESHOLD = 5;


    @Override
    public void runOpMode() throws InterruptedException {
        robot = new HardwareSigma2016();
        robot.init(hardwareMap);

        gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");
        readFile(); //Finds file on the phone and puts the whole file in a string called fullFile
        seperateCommand(); //Cuts the fullFile variable into array items in the array called commands. Cutting = separating at the slash
        waitForStart();
        while (opModeIsActive()) {
            for (int i = 0; i < commands.size(); i++){
                /*telemetry.addData("indside for loop", "true");
                telemetry.addData("Arraylist contents", commands.get(i));
                telemetry.update();*/
                arrayItem = commands.get(i); //Gets the array item
                telemetry.addData("arrayItem:", arrayItem);
                telemetry.update();
                //sleep(1000);
                if(arrayItem.contains("wait")){
                    Thread.sleep(1000);
                }
                else if(arrayItem.contains("beacon_senseR")){
                    ColorDetectionAndButtonPushingRed();
                }
                else if(arrayItem.contains("beacon_senseB")){
                    ColorDetectionAndButtonPushingBlue();
                }
                else if(arrayItem.contains("leftThrottle")){
                    leftThrottle = Double.parseDouble(arrayItem.substring(arrayItem.indexOf("-") + 1, arrayItem.indexOf("rightThrottle")));
                    telemetry.addData("leftThrottle:", leftThrottle);

                    arrayItem = arrayItem.substring(arrayItem.indexOf("rightThrottle"));

                    //Find and parse the rightthrottle value
                    rightThrottle = Double.parseDouble(arrayItem.substring(arrayItem.indexOf("-") + 1, arrayItem.indexOf("gyro")));
                    telemetry.addData("rightThrottle:", rightThrottle);

                    arrayItem = arrayItem.substring(arrayItem.indexOf("gyro"));

                    //Find and parse the gyro value
                    gyroVal = Double.parseDouble(arrayItem.substring(arrayItem.indexOf("-") + 1));

                    if(gyroVal > 180){
                        gyroVal -= 360;
                    }
                    if(gyroVal < -180){
                        gyroVal += 360;
                    }

                    telemetry.addData("gyro:", gyroVal);
                    telemetry.update();
                    //sleep(1000);

                    //Finds distance value for throttles.
                    leftInch = leftThrottle/COUNTS_PER_INCH;
                    rightInch = -rightThrottle/COUNTS_PER_INCH;

                    //Finds if going straight or turning. Currently a PROBLEM -- see above
                    if(leftThrottle > 0 && rightThrottle > 0) {
                        telemetry.addData("leftThrottle:", leftThrottle);
                        telemetry.addData("rightThrottle:", rightThrottle);
                        telemetry.addData("gyro:", gyroVal);
                        telemetry.addData("inside if", "forward");
                        telemetry.update();
                        gyroDrive(0.4, leftInch, gyro.getIntegratedZValue());
                        //Thread.sleep(5000);
                    }
                    else if(leftThrottle < 0 && rightThrottle < 0){
                        telemetry.addData("leftThrottle:", leftThrottle);
                        telemetry.addData("rightThrottle:", rightThrottle);
                        telemetry.addData("gyro:", gyroVal);
                        telemetry.addData("inside if", "backward");
                        telemetry.update();
                        gyroDrive(-0.4, leftInch, gyro.getIntegratedZValue());
                        //Thread.sleep(5000);
                    }
                    else if((leftThrottle > 0 && rightThrottle < 0)){
                        telemetry.addData("leftThrottle:", leftThrottle);
                        telemetry.addData("rightThrottle:", rightThrottle);
                        telemetry.addData("gyro:", gyroVal);
                        telemetry.addData("inside if", "left");
                        telemetry.update();
                        gyroTurn(0.25, -gyroVal);
                        gyroTotal -= gyroVal;
                        //Thread.sleep(5000);
                    }
                    else if((leftThrottle < 0 && rightThrottle > 0)){
                        telemetry.addData("leftThrottle:", leftThrottle);
                        telemetry.addData("rightThrottle:", rightThrottle);
                        telemetry.addData("gyro:", gyroVal);
                        telemetry.addData("inside if", "left");
                        telemetry.update();
                        gyroTurn(0.25, -gyroVal);
                        gyroTotal -= gyroVal;
                        //Thread.sleep(5000);
                    }
                }
                //Find and parse the leftthrottle value

            }
            break;
        }
    }
    public void readFile(){
        if (file.exists()) {
            try {
                is = new FileInputStream(file);
                reader = new BufferedReader(new InputStreamReader(is));
                String line = "";
                fullFile = "";
                while (line != null) {
                    line = reader.readLine();
                    fullFile += line;
                    //telemetry.addData("Line:", fullFile);
                    //telemetry.update();
                }
            } catch (FileNotFoundException e) {
                e.printStackTrace();
            } catch (IOException e) {
                e.printStackTrace();
            }
        }
        else {
            telemetry.addData("File","Not Found");
            telemetry.update();
        }
        //sleep(20000);
    }

    public void seperateCommand(){
        while (fullFile.length() != 1){
            pos = fullFile.indexOf("/");

            telemetry.addData("arrayItem:", fullFile.substring(0,pos));
            telemetry.addData("length:", fullFile.length());
            telemetry.update();
            sleep(200);

            commands.add(fullFile.substring(0,pos));
            if(pos < fullFile.length() - 8){
                telemetry.addData("arrayItem:", fullFile.substring(0,pos));
                telemetry.addData("length:", fullFile.length());
                telemetry.addData("pos:", pos);
                telemetry.addData("fullFile.length() - 1:", fullFile.length() - 1);
                telemetry.addData("insideIf:", "true");
                telemetry.update();

                fullFile = fullFile.substring(pos + 1);

            }
            else{
                break;
            }
        }
    }
    public void gyroDrive(double speed,
                          double distance,
                          double angle) {

        int newLeftTarget;
        int newRightTarget;
        int moveCounts;
        double max;
        double error;
        double steer;
        double leftSpeed;
        double rightSpeed;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // reset encoder
            robot.frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            // set mode
            robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.backLeftMotor.setMode(RUN_WITHOUT_ENCODER);
            robot.backRightMotor.setMode((RUN_WITHOUT_ENCODER));

            // Determine new target position, and pass to motor controller
            moveCounts = (int) (distance * COUNTS_PER_INCH);
            newLeftTarget = robot.frontLeftMotor.getCurrentPosition() + moveCounts;
            newRightTarget = robot.frontRightMotor.getCurrentPosition() + moveCounts;

            // Set Target and Turn On RUN_TO_POSITION
            robot.frontLeftMotor.setTargetPosition(newLeftTarget);
            robot.frontRightMotor.setTargetPosition(newRightTarget);

            robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // determine back motor's direction
            if (distance < 0) {
                if (robot.backLeftMotor.getDirection() == DcMotorSimple.Direction.FORWARD) {
                    robot.backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
                } else {
                    robot.backLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
                }

                if (robot.backRightMotor.getDirection() == DcMotorSimple.Direction.FORWARD) {
                    robot.backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
                } else {
                    robot.backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
                }
            }

            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            robot.frontLeftMotor.setPower(speed);
            robot.frontRightMotor.setPower(speed);
            robot.backRightMotor.setPower(speed);
            robot.backLeftMotor.setPower(speed);

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    (robot.frontLeftMotor.isBusy() && robot.frontRightMotor.isBusy())) {

                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                leftSpeed = speed - steer;
                rightSpeed = speed + steer;

                // Normalize speeds if any one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0) {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }

                robot.frontLeftMotor.setPower(leftSpeed);
                robot.frontRightMotor.setPower(rightSpeed);
                robot.backLeftMotor.setPower(leftSpeed);
                robot.backRightMotor.setPower(rightSpeed);

                // Display drive status for the driver.
                telemetry.addData("Err/St", "%5.1f/%5.1f", error, steer);
                telemetry.addData("Target", "%7d:%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Actual", "%7d:%7d", robot.frontLeftMotor.getCurrentPosition(),
                        robot.frontRightMotor.getCurrentPosition());

                telemetry.addData("Speed", "%5.2f:%5.2f", leftSpeed, rightSpeed);
                telemetry.update();
            }

            // Stop all motion;
            robot.frontLeftMotor.setPower(0);
            robot.frontRightMotor.setPower(0);
            robot.backLeftMotor.setPower(0);
            robot.backRightMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            if (distance < 0) {
                if (robot.backLeftMotor.getDirection() == DcMotorSimple.Direction.FORWARD) {
                    robot.backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
                } else {
                    robot.backLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
                }

                if (robot.backRightMotor.getDirection() == DcMotorSimple.Direction.FORWARD) {
                    robot.backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
                } else {
                    robot.backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
                }
            }
        }
    }
    public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - gyro.getIntegratedZValue();
        while (robotError > 180) robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     *
     * @param error  Error angle in robot relative degrees
     * @param PCoeff Proportional Gain Coefficient
     * @return
     */
    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }
    public void gyroTurn(double speed, double angle) {

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF)) {
            // Update telemetry & Allow time for other processes to run.
            telemetry.update();
        }
    }
    boolean onHeading(double speed, double angle, double PCoeff) {
        double error;
        double steer;
        boolean onTarget = false;
        double leftSpeed;
        double rightSpeed;

        robot.frontLeftMotor.setMode(RUN_WITHOUT_ENCODER);
        robot.frontRightMotor.setMode(RUN_WITHOUT_ENCODER);
        robot.backLeftMotor.setMode(RUN_WITHOUT_ENCODER);
        robot.backRightMotor.setMode((RUN_WITHOUT_ENCODER));

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
            telemetry.addData("INSIDE IF", "TRUE");
            telemetry.addData("error", error);
            telemetry.addData("angle", angle);
            telemetry.addData("speed", speed);
            telemetry.addData("threshold", HEADING_THRESHOLD);
            telemetry.addData("ACTUAL", gyro.getIntegratedZValue());
        } else {
            steer = getSteer(error, PCoeff);
            rightSpeed = speed * steer;
            leftSpeed = -rightSpeed;
            telemetry.addData("INSIDE IF", "FALSE");
            telemetry.addData("error", error);
            telemetry.addData("angle", angle);
            telemetry.addData("speed", speed);
            telemetry.addData("threshold", HEADING_THRESHOLD);
            telemetry.addData("ACTUAL", gyro.getIntegratedZValue());
        }

        // Send desired speeds to motors.
        robot.frontLeftMotor.setPower(leftSpeed);
        robot.frontRightMotor.setPower(rightSpeed);
        robot.backLeftMotor.setPower(leftSpeed);
        robot.backRightMotor.setPower(rightSpeed);

        // Display it for the driver.
        telemetry.addData("Target", "%5.2f", angle);
        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);

        return onTarget;
    }
    public void ColorDetectionAndButtonPushingRed() {

        ElapsedTime holdTimer = new ElapsedTime();
        double holdTime = 2;  // 2 second timeout
        int red, green, blue;
        int redCheck = 0, blueCheck = 0;

        robot.beaconColorSensor.enableLed(false); //led OFF

        // keep looping while we have time remaining.
        holdTimer.reset();
        while (holdTimer.time() < holdTime) {

            red = robot.beaconColorSensor.red();
            green = robot.beaconColorSensor.green();
            blue = robot.beaconColorSensor.blue();

            System.out.println("--BlueNear log-- R:G:B = " + red + ":" + green + ":" + blue);

            if ((red > blue+20) && (red > green+20)) {
                redCheck ++;
            } else {
                redCheck = 0;
            }

            if ((blue > red + 5) && (blue > green)) {
                blueCheck ++;
            } else {
                blueCheck = 0;
            }

            telemetry.addData("ColorRGB:: ", "%d %d %d", red, green, blue);
            telemetry.addData("ColorRC&BC :: ", "%d %d", redCheck, blueCheck);
            telemetry.update();

            // red color detected
            if (redCheck > RED_TRESHOLD) {

                // We are red team
                robot.pusherL.setPosition(PUSHER_L_OUT);
                //wait servo to finish
                sleep(1300);

                // Retrieve the pusher
                robot.pusherL.setPosition(PUSHER_L_IN);
                //wait servo to finish
                sleep(1300);

                robot.pusherL.setPosition(PUSHER_STOP);

                System.out.println("--RedNear log-- red light detected and red button pushed. redCheck=" + redCheck + " blueCheck=" + blueCheck);
                break;
            }

            // blue color detected
            if (blueCheck > BLUE_TRESHOLD) {

                // We are the blue team
                robot.pusherR.setPosition(PUSHER_R_OUT);
                //wait servo to finish
                sleep(1300);

                // Retrieve the pusher
                robot.pusherR.setPosition(PUSHER_R_IN);
                //wait servo to finish
                sleep(1300);
                robot.pusherR.setPosition(PUSHER_STOP);

                System.out.println("--RedNear log-- blue light detected and red button pushed. blueCheck=" + blueCheck + " redCheck=" + redCheck);
                break;
            }

            sleep(10);
            idle();
        }
    }
    public void ColorDetectionAndButtonPushingBlue() {

        ElapsedTime holdTimer = new ElapsedTime();
        double holdTime = 2;  // 2 second timeout
        int red, green, blue;
        int redCheck = 0, blueCheck = 0;

        robot.beaconColorSensor.enableLed(false); //led OFF

        // keep looping while we have time remaining.
        holdTimer.reset();
        while (holdTimer.time() < holdTime) {

            red = robot.beaconColorSensor.red();
            green = robot.beaconColorSensor.green();
            blue = robot.beaconColorSensor.blue();

            System.out.println("--BlueNear log-- R:G:B = " + red + ":" + green + ":" + blue);

            if ((red > blue + 20) && (red > green + 20)){
                redCheck++;
            }
            else {
                redCheck = 0;
            }


            if ((blue > red + 5) && (blue > green + 5)) {
                blueCheck++;
            } else {
                blueCheck = 0;
            }

            telemetry.addData("ColorRGB:: ", "%d %d %d", red, green, blue);
            telemetry.addData("ColorRC&BC :: ", "%d %d", redCheck, blueCheck);
            telemetry.update();

            // red color detected
            if (redCheck > RED_TRESHOLD) {

                // We are blue team
                robot.pusherR.setPosition(PUSHER_R_OUT);
                //wait servo to finish
                sleep(1300);

                // Retrieve the pusher
                robot.pusherR.setPosition(PUSHER_R_IN);
                //wait servo to finish
                sleep(1300);

                robot.pusherR.setPosition(PUSHER_STOP);

                System.out.println("--BlueNear log-- red light detected and blue button pushed. redCheck=" + redCheck + " blueCheck=" + blueCheck);
                break;
            }

            // blue color detected
            if (blueCheck > BLUE_TRESHOLD) {

                // We are the blue team
                robot.pusherL.setPosition(PUSHER_L_OUT);
                //wait servo to finish
                sleep(1300);

                // Retrieve the pusher
                robot.pusherL.setPosition(PUSHER_L_IN);
                //wait servo to finish
                sleep(1300);
                robot.pusherL.setPosition(PUSHER_STOP);

                System.out.println("--BlueNear log-- blue light detected and blue button pushed. blueCheck=" + blueCheck + " redCheck=" + redCheck);
                break;
            }

            sleep(10);
            idle();
        }
    }
}