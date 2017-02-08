package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.Range;

public class HighPriorityRunnerSigma2016 extends Thread {

    // class variables
    HardwareSigma2016 robot = null;

    boolean bWallTracking = false;
    boolean bLineDetection = false;

    double targetSpeed = 0.0;
    double travelDistance = 0.0;
    double targetWallDistance = 0.0;

    int ct0;

    // methods
    HighPriorityRunnerSigma2016(HardwareSigma2016 ourHardware) {
        robot = ourHardware;
        this.setPriority(this.MAX_PRIORITY);
    }

    public void SetWallTrackingAndLineDetection(boolean bWall,
                                                boolean bLine,
                                                double fSpeed,
                                                double fDistance,
                                                double fWallDistance) {
        bWallTracking = bWall;
        bLineDetection = bLine;

        targetSpeed = fSpeed;
        travelDistance = fDistance;
        targetWallDistance = fWallDistance;

        System.out.println("Sigma2016 -- wall tracking is " + bWall + " and line detection is " + bLine);
        System.out.println("Sigma2016 -- speed is " + targetSpeed + " distance is " + travelDistance);
    }

    public int SpeedControl() {
        double speedTime1;
        double speedTime2;
        int position1;
        int position2;
        int angle1;
        int angle2;
        double currentSpeed;
        double currentAngleSpeed;

        speedTime1 = System.currentTimeMillis();
        position1 = robot.frontLeftMotor.getCurrentPosition();
        speedTime2 = System.currentTimeMillis();
        position2 = robot.frontLeftMotor.getCurrentPosition();

        robot.currentSpeed = Math.abs(position2 - position1) / (speedTime2 - speedTime1);

        speedTime1 = System.currentTimeMillis();
        angle1 = robot.gyro.getHeading();
        speedTime2 = System.currentTimeMillis();
        angle2 = robot.gyro.getHeading();

        robot.currentAngleSpeed = Math.abs(angle2 - angle1) / (speedTime2 - speedTime1);

        return(0);
    }

    public void run() {
        int lightlevelR, lightlevelG, lightlevelB;
        long curTime, timeInterval;
        long previousRunTime = 0;
        double error, lastErr = 0;
        double steer = 0;
        double leftSpeed;
        double rightSpeed;
        double ultraSoundLevel, angleOffset;
        int lightlevel = 0;
        double targetAngleOffset, angleSteer;
        double minSpeed = -Math.abs(targetSpeed);
        double maxSpeed = Math.abs(targetSpeed);

        // from Hitecnic color sensor web page
        // The Color Number calculated by the sensor is refreshed approximately 100 times per second.
        int expectedRunInterval = 10; // millisecond

        while (true) {

            // check if task runs on time
            curTime = System.currentTimeMillis();
            if (previousRunTime != 0) {
                timeInterval = curTime - previousRunTime;

                if (timeInterval > expectedRunInterval * 2) {
                    System.out.println("Sigma2016 -- sensor reading thread did not run for " + timeInterval + "ms");
                }
            }
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
                curTime = System.currentTimeMillis();
                if (previousRunTime != 0) {
                    timeInterval = curTime - previousRunTime;

                    if (timeInterval > expectedRunInterval * 2) {
                        System.out.println("Sigma2016 -- Blue While did not run for " + timeInterval + "ms");
                    }
                }
                previousRunTime = curTime;

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

                    leftSpeed = targetSpeed - steer;
                    rightSpeed = targetSpeed + steer;

                    leftSpeed = Range.clip(leftSpeed, minSpeed, maxSpeed);
                    rightSpeed = Range.clip(rightSpeed, minSpeed, maxSpeed);

                    robot.frontLeftMotor.setPower(leftSpeed);
                    robot.frontRightMotor.setPower(rightSpeed);
                    robot.backLeftMotor.setPower(leftSpeed);
                    robot.backRightMotor.setPower(rightSpeed);

                    if (ct0 == 0) {
                        System.out.println("--Sigma2016-- error=" + String.format(Double.toString(error), "%5.2f")
                                + " leftspeed=" + String.format(Double.toString(leftSpeed), "%5.2f")
                                + " rightSpeed=" + String.format(Double.toString(rightSpeed), "%5.2f")
                                + " curAngle=" + angleOffset
                                + " targetAngle=" + targetAngleOffset);
                    }
                } else if (Math.abs(error) >= 1) {

                    // distance off by 1. Need mild angle driving back to expected distance
                    // 3.0 is the expected front/back ultrasound sensor difference
                    targetAngleOffset = 0.0 - Math.signum(travelDistance) * Math.signum(error) * 2.0;

                    angleSteer = targetAngleOffset - angleOffset;

                    steer = Math.signum(travelDistance) * angleSteer * 0.02;

                    leftSpeed = targetSpeed - steer;
                    rightSpeed = targetSpeed + steer;

                    leftSpeed = Range.clip(leftSpeed, minSpeed, maxSpeed);
                    rightSpeed = Range.clip(rightSpeed, minSpeed, maxSpeed);

                    robot.frontLeftMotor.setPower(leftSpeed);
                    robot.frontRightMotor.setPower(rightSpeed);
                    robot.backLeftMotor.setPower(leftSpeed);
                    robot.backRightMotor.setPower(rightSpeed);

                    if (ct0 == 0) {
                        System.out.println("--Sigma2016-- error=" + String.format(Double.toString(error), "%5.2f")
                                + " leftspeed=" + String.format(Double.toString(leftSpeed), "%5.2f")
                                + " rightSpeed=" + String.format(Double.toString(rightSpeed), "%5.2f")
                                + " curAngle=" + angleOffset
                                + " targetAngle=" + targetAngleOffset);
                    }
                } else {

                    // distance on target. Need to keep 0 angle offset
                    // 3.0 is the expected front/back ultrasound sensor difference
                    targetAngleOffset = 0.0;

                    angleSteer = targetAngleOffset - angleOffset;

                    steer = Math.signum(travelDistance) * angleSteer * 0.04;

                    leftSpeed = targetSpeed - steer;
                    rightSpeed = targetSpeed + steer;

                    leftSpeed = Range.clip(leftSpeed, minSpeed, maxSpeed);
                    rightSpeed = Range.clip(rightSpeed, minSpeed, maxSpeed);

                    robot.frontLeftMotor.setPower(leftSpeed);
                    robot.frontRightMotor.setPower(rightSpeed);
                    robot.backLeftMotor.setPower(leftSpeed);
                    robot.backRightMotor.setPower(rightSpeed);

                    if (ct0 == 0) {
                        System.out.println("--Sigma2016-- error=" + String.format(Double.toString(error), "%5.2f")
                                + " leftspeed=" + String.format(Double.toString(leftSpeed), "%5.2f")
                                + " rightSpeed=" + String.format(Double.toString(rightSpeed), "%5.2f")
                                + " curAngle=" + angleOffset
                                + " targetAngle=" + targetAngleOffset);
                    }
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
