package org.firstinspires.ftc.teamcode;

public class ReadSensorsSigma2016 extends Thread {

    boolean bReadCenterLightSensor = false;
    HardwareSigma2016 robot = null;


    ReadSensorsSigma2016(HardwareSigma2016 ourHardware) {
        robot = ourHardware;
        this.setPriority(this.MAX_PRIORITY);
    }

    public void SetCenterLightSensorRead(boolean onOff) {
        bReadCenterLightSensor = onOff;

        robot.centerLightSensorLevelMax = 0;
        robot.centerLightSensorLevel = 0;

        System.out.println("Sigma2016 -- center light sensor read is set to " + onOff);
    }

    public void run() {
        int lightlevelR, lightlevelG, lightlevelB;
        long curTime, timeInterval;
        long previousRunTime = 0;

        // from Hitecnic color sensor web page
        // The Color Number calculated by the sensor is refreshed approximately 100 times per second.
        int expectedRunInterval = 10; // millisecond

        while (true) {

            curTime = System.currentTimeMillis();
            if (previousRunTime != 0)
            {
                timeInterval = curTime - previousRunTime;

                if (timeInterval > expectedRunInterval*2)
                {
                    System.out.println("Sigma2016 -- sensor reading thread did not run for " + timeInterval + "ms");
                }
            }
            previousRunTime = curTime;

            if (bReadCenterLightSensor)
            {
                lightlevelB = robot.lineLightSensor.blue();
                lightlevelR = robot.lineLightSensor.red();
                lightlevelG = robot.lineLightSensor.green();

                robot.centerLightSensorLevel = lightlevelB + lightlevelR + lightlevelG;
                if (robot.centerLightSensorLevel > robot.centerLightSensorLevelMax)
                {
                    robot.centerLightSensorLevelMax = robot.centerLightSensorLevel;
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
