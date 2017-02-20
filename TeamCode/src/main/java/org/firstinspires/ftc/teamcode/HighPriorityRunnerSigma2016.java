package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.Range;

import static android.R.attr.max;
import static android.R.attr.right;

public class HighPriorityRunnerSigma2016 extends Thread {

    // class variables
    HardwareSigma2016 robot = null;

    boolean bWallTracking = false;
    boolean bLineDetection = false;
    boolean bOnHeading = false;

    double travelDistance = 0.0;
    double targetWallDistance = 0.0;

    double power = 0;

    int ct0;

    public boolean go = false;

    // speed calculation variables
    int lastLeftEncoderPosition = 0;
    int lastRightEncoderPosition = 0;
    double lastAngle = 0.0;
    long lastRunTime = 0;

    // methods
    HighPriorityRunnerSigma2016(HardwareSigma2016 ourHardware) {
        robot = ourHardware;

        this.setPriority(this.MAX_PRIORITY);
        this.go = true;
    }

    public void SetWallTrackingAndLineDetection(boolean bWall,
                                                boolean bLine,
                                                double fPower,
                                                double fDistance,
                                                double fWallDistance) {
        bWallTracking = bWall;
        bLineDetection = bLine;

        power = fPower;

        travelDistance = fDistance;
        targetWallDistance = fWallDistance;

        System.out.println("Sigma2016 -- wall tracking is " + bWall + " and line detection is " + bLine);
        System.out.println("Sigma2016 -- target power is " + fPower + " distance is " + travelDistance);
    }

    public int GetSpeed(long timeDelta) {
        int leftEncoderPosition = 0;
        int rightEncoderPosition = 0;
        double lastSpeed = 0.0;
        double angle = 0;
        double leftSpeed, rightSpeed;

        leftEncoderPosition = robot.LeftMotor.getCurrentPosition();
        rightEncoderPosition = robot.RightMotor.getCurrentPosition();

        leftSpeed = Math.abs(leftEncoderPosition - lastLeftEncoderPosition) / timeDelta;
        rightSpeed = Math.abs(rightEncoderPosition - lastRightEncoderPosition) / timeDelta;

        // calculate current speed -- unit inch per second
        robot.currentSpeed = Math.max(leftSpeed, rightSpeed) * 1000 / robot.COUNTS_PER_INCH;  // inch per second

        lastLeftEncoderPosition = leftEncoderPosition;
        lastRightEncoderPosition = rightEncoderPosition;

        angle = robot.gyro.getIntegratedZValue();
        robot.currentAngleSpeed = Math.abs(angle - lastAngle) * 1000 / timeDelta;  // degree per second
        lastAngle = angle;

        return (0);
    }

    public void run() {
        int lightlevelR, lightlevelG, lightlevelB;
        long curTime, timeInterval;
        long previousRunTime = 0;
        double error, lastErr = 0;
        double steer = 0;
        double leftPower;
        double rightPower;
        double ultraSoundLevel, angleOffset;
        int lightlevel = 0;
        double targetAngleOffset, angleSteer;
        double minPower, maxPower;

        // from Hitecnic color sensor web page
        // The Color Number calculated by the sensor is refreshed approximately 100 times per second.
        int expectedRunInterval = 10; // millisecond

        while (this.go) {

            // check if task runs on time
            curTime = System.currentTimeMillis();
            if (previousRunTime != 0) {
                timeInterval = curTime - previousRunTime;

                if (timeInterval > expectedRunInterval * 2) {
                    System.out.println("Sigma2016 -- sensor reading thread did not run for " + timeInterval + "ms");
                }
            }

            // Get current speed information
            GetSpeed(curTime - previousRunTime);

            previousRunTime = curTime;

            // line detection
            if (bLineDetection) {
                lightlevelB = robot.lineLightSensor.blue();
                lightlevelR = robot.lineLightSensor.red();
                lightlevelG = robot.lineLightSensor.green();

                lightlevel = lightlevelR + lightlevelG + lightlevelB;

                if (ct0 == 0) {
                    System.out.println("Ground Brightness:: " + robot.groundbrightnessAVG
                            + " Light Level:: " + lightlevel);
                }

                if (lightlevel > robot.groundbrightnessAVG * robot.CENTER_LIGHT_THRESH) {
                    System.out.println("Sigma2016 ---> Ground Brightness:: " + robot.groundbrightnessAVG
                            + " DETECTED Light Level:: " + lightlevel);

                    // Stop the wall and line tracking
                    bWallTracking = false;
                    bLineDetection = false;
                }
            }

            // wall tracking
            if (bWallTracking) {

                minPower = -Math.abs(power);
                maxPower = Math.abs(power);

                if (travelDistance < 0) {
                    ultraSoundLevel = robot.ultra_back.getUltrasonicLevel();
                } else {
                    ultraSoundLevel = robot.ultra_front.getUltrasonicLevel();
                }

                error = ultraSoundLevel - targetWallDistance;

                // get angle offset of the wall
                angleOffset = robot.gyro.getIntegratedZValue();

                ct0++;
                if (error != lastErr) {
                    ct0 = 51;
                }

                lastErr = error;

                if (ct0 > 50) {
                    ct0 = 0;

                    System.out.println("--Sigma2016-- ultrasoniclevel=" + ultraSoundLevel
                            + " error=" + error
                            + " angleOffset=" + angleOffset
                            + " distance=" + travelDistance);
                }

                if (ultraSoundLevel == 255) {
                    // error reading. Ignore.
                    continue;
                }

                // adjust angle pointing based on ultrasound reading.
                if (Math.abs(error) >= 2) {
                    // distance off by more than 2. Need steep 10 degree angle driving back to expected distance
                    targetAngleOffset = 0.0 - Math.signum(travelDistance) * Math.signum(error) * 4.0;

                    angleSteer = targetAngleOffset - angleOffset;

                    steer = Math.signum(travelDistance) * angleSteer * 0.04;

                    leftPower = power - steer;
                    rightPower = power + steer;

                    leftPower = Range.clip(leftPower, minPower, maxPower);
                    rightPower = Range.clip(rightPower, minPower, maxPower);

                    robot.LeftMotor.setPower(leftPower);
                    robot.RightMotor.setPower(rightPower);

                    if (ct0 == 0) {
                        System.out.println("--Sigma2016-- error=" + String.format(Double.toString(error), "%5.2f")
                                + " leftPower=" + String.format(Double.toString(leftPower), "%5.2f")
                                + " rightPower=" + String.format(Double.toString(rightPower), "%5.2f")
                                + " curAngle=" + angleOffset
                                + " targetAngle=" + targetAngleOffset);
                    }
                } else if (Math.abs(error) >= 1) {

                    // distance off by 1. Need mild angle driving back to expected distance
                    // 3.0 is the expected front/back ultrasound sensor difference
                    targetAngleOffset = 0.0 - Math.signum(travelDistance) * Math.signum(error) * 2.0;

                    angleSteer = targetAngleOffset - angleOffset;

                    steer = Math.signum(travelDistance) * angleSteer * 0.02;

                    leftPower = robot.targetSpeed - steer;
                    rightPower = robot.targetSpeed + steer;

                    leftPower = Range.clip(leftPower, minPower, maxPower);
                    rightPower = Range.clip(rightPower, minPower, maxPower);

                    robot.LeftMotor.setPower(leftPower);
                    robot.RightMotor.setPower(rightPower);

                    if (ct0 == 0) {
                        System.out.println("--Sigma2016-- error=" + String.format(Double.toString(error), "%5.2f")
                                + " leftPower=" + String.format(Double.toString(leftPower), "%5.2f")
                                + " rightPower=" + String.format(Double.toString(rightPower), "%5.2f")
                                + " curAngle=" + angleOffset
                                + " targetAngle=" + targetAngleOffset);
                    }
                } else {

                    // distance on target. Need to keep 0 angle offset
                    // 3.0 is the expected front/back ultrasound sensor difference
                    targetAngleOffset = 0.0;

                    angleSteer = targetAngleOffset - angleOffset;

                    steer = Math.signum(travelDistance) * angleSteer * 0.04;

                    leftPower = robot.targetSpeed - steer;
                    rightPower = robot.targetSpeed + steer;

                    leftPower = Range.clip(leftPower, minPower, maxPower);
                    rightPower = Range.clip(rightPower, minPower, maxPower);

                    robot.LeftMotor.setPower(leftPower);
                    robot.RightMotor.setPower(rightPower);

                    if (ct0 == 0) {
                        System.out.println("--Sigma2016-- error=" + String.format(Double.toString(error), "%5.2f")
                                + " leftPower=" + String.format(Double.toString(leftPower), "%5.2f")
                                + " rightPower=" + String.format(Double.toString(rightPower), "%5.2f")
                                + " curAngle=" + angleOffset
                                + " targetAngle=" + targetAngleOffset);
                    }
                }

                // Monitor speed and adjust power if necessary
                if (robot.currentSpeed < robot.targetSpeed) {
                    power += 0.05;
                } else if (robot.currentSpeed > robot.targetSpeed) {
                    power -= 0.05;
                }
            }

//            System.out.println("Sigma2016 -- Sensor reading thread is running. Priority = " + this.getPriority());
//            System.out.println("ReadSensors thread cur time = " + System.currentTimeMillis());

            try {
                sleep(expectedRunInterval);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }
}
