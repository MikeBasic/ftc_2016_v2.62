package org.firstinspires.ftc.teamcode;

import android.os.Environment;
import android.util.Log;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.PrintWriter;

public class LoggerSigma2016 {
    public PrintWriter printWriter;

    public LoggerSigma2016(String fileName) throws FileNotFoundException {

        if (isExternalStorageWritable() == false)
        {
            System.out.println("No valid external storage found!");

            System.exit(0);
        }

        String logFilePath = Environment.getExternalStorageDirectory().getAbsolutePath() + "/ctu_log/";

        try {
            File path = new File(logFilePath);
            if (!path.exists()) {
                path.mkdirs();
            }
        } catch (Exception e) {
            Log.e("Log file path creation", e.getMessage());
        }

        printWriter = new PrintWriter(logFilePath + fileName);
    }

    public void logLine(String sLine) {
        printWriter.println(sLine);
    }

    public void close() {
        printWriter.close();
    }

    /* Checks if external storage is available for read and write */
    public boolean isExternalStorageWritable() {
        String state = Environment.getExternalStorageState();
        if (Environment.MEDIA_MOUNTED.equals(state)) {
            return true;
        }
        return false;
    }
}
