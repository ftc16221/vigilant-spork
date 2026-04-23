package org.firstinspires.ftc.teamcode.subassemblies;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Locale;

//@Config
public class Watchdog {

    public static boolean WRITE_TO_LOG_FILE = false;
    public static String LOG_FILE_DIR = AppUtil.FIRST_FOLDER + "/mmmlogs/";
    public static String LOG_FILE_NAME = "watchdog";

    private final OpMode opMode;

    private final File logFile;
    private static PrintWriter logWriter = null;
    private static final SimpleDateFormat dateFormat = new SimpleDateFormat("yyyy-MM-dd HH:mm:ss", Locale.US);

    public Watchdog(OpMode opMode) {
        this.opMode = opMode;

        if (WRITE_TO_LOG_FILE) {
            File dir = new File(LOG_FILE_DIR);
            if (!dir.exists()) dir.mkdirs();

            logFile = new File(dir, generateFileName()); // create reference to target file
            try {
                FileWriter fw = new FileWriter(logFile, true);
                BufferedWriter bw = new BufferedWriter(fw);
                logWriter = new PrintWriter(bw);
            } catch (IOException e) {
                RobotLog.ee("Watchdog", e, "Failed to open log file");
            }
        } else {
            logFile = null;
        }
    }

    private String generateFileName() {
        String logFilePrefix;
        if (opMode.getClass().isAnnotationPresent(Autonomous.class)) {
            logFilePrefix = "auto";
        } else if (opMode.getClass().isAnnotationPresent(TeleOp.class)) {
            logFilePrefix = "teleop";
        } else {
            logFilePrefix = "opmode";
        }

        return LOG_FILE_NAME + "_" + logFilePrefix + "_" + new SimpleDateFormat("yyyyMMdd_HHmmss", Locale.US).format(new Date()) + ".log";
    }

    public static void log(String message) {
        if (logWriter != null) {
                logWriter.println(dateFormat.format(new Date()) + " " + message);
                logWriter.flush();
        }
    }

    public static void i(String message) {
        RobotLog.i(message);
        log(message);
    }

    public static void w(String message) {
        RobotLog.w(message);
        log("WARNING: " + message);
    }

    public static void e(String message) {
        RobotLog.e(message);
        log("ERROR: " + message);
    }

    public void stop() {
        if (logWriter != null) {
            logWriter.close();
            logWriter = null;
        }
    }

    @Deprecated
    public void update() {
        // intentionally blank, functionality has been removed
    }
}