package org.firstinspires.ftc.teamcode.subassemblies;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.util.Subassembly;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.lang.reflect.Field;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;
import java.util.HashSet;
import java.util.List;
import java.util.Locale;
import java.util.Set;

@Config
public class Watchdog {

    public static boolean WRITE_TO_LOG_FILE = false;
    public static String LOG_FILE_PATHNAME = ""; // TODO: find spot for log file
    public static String LOG_FILE_NAME = "watchdog.log";

    private final File logFile;
    private FileWriter logWriter = null;
    private final Set<String> prevNormalizedIssues = new HashSet<>();
    private final SimpleDateFormat dateFormat = new SimpleDateFormat("yyyy-MM-dd HH:mm:ss", Locale.US);

    private final OpMode opMode;
    private final Subassembly[] subassemblies;

    private final MultipleTelemetry telemetry;

    /** creates a watchdog object with the subassembly fields (public or private) automatically detected within opMode */
    public Watchdog(OpMode opMode) {
        this(opMode, findSubassemblies(opMode));
    }

    /** creates a watchdog object with the supplied subassemblies. Use if subassemblies are not declared as a field, or a subassembly should be excluded */
    public Watchdog(OpMode opMode, Subassembly... subassemblies) {
        this.opMode = opMode;
        this.subassemblies = subassemblies;

        if (WRITE_TO_LOG_FILE) {
            logFile = new File(LOG_FILE_PATHNAME + LOG_FILE_NAME); // create reference to target file
            if (logFile.getParentFile() != null) logFile.getParentFile().mkdirs(); // ensure parent directories are existent

            try {
                logWriter = new FileWriter(logFile, true);
            } catch (IOException e) {
                RobotLog.e("Failed to open log file");
            }
        } else {
            logFile = null;
        }

        telemetry = new MultipleTelemetry(opMode.telemetry, FtcDashboard.getInstance().getTelemetry());

        ArrayList<String> subassemblyStrings = new ArrayList<>();
        for (Subassembly subassembly : subassemblies) {
            subassemblyStrings.add(subassembly.getClass().getTypeName());
        }
        log("Watchdog initialized with the following subassembly types: " + subassemblyStrings);
    }

    public void update() {

        Set<String> issues = new HashSet<>();

        for (Subassembly subassembly : subassemblies) {
            List<String> subIssues = subassembly.findIssues();
            // check if a new issue is discovered, and if so, add it log and activeIssues
            if (subIssues != null && !subIssues.isEmpty()) {
                issues.addAll(subIssues);
            }
        }

        Set<String> normalizedIssues = new HashSet<>();

        for (String issue : issues) {
            String normalizedIssue = issue.replaceAll("[0-9]", ""); // this is a message with numbers removed
            normalizedIssues.add(normalizedIssue);
            // add new issues
            if (!prevNormalizedIssues.contains(normalizedIssue)) {
                prevNormalizedIssues.add(normalizedIssue);
                log("ISSUE DISCOVERED: " + issue);
            }

            // add all issues to telemetry
            telemetry.addLine("Warning: " + issue);
        }

        // remove old issues, and display message that they have been cleared
        prevNormalizedIssues.removeIf(i -> {
            boolean stillActive = normalizedIssues.contains(i); // check if our tracked issues were polled this cycle
            if (!stillActive) {
                log("ISSUE RESOLVED: " + i);
            }
            return !stillActive;
        });
    }

    private static Subassembly[] findSubassemblies(OpMode opMode) {
        List<Subassembly> subassemblies = new ArrayList<>();

        for (Field field : opMode.getClass().getDeclaredFields()) {
            if (Subassembly.class.isAssignableFrom(field.getType())) { // find fields that inherit subassembly
                field.setAccessible(true); // allow access to both private and public fields
                try {
                    Object value = field.get(opMode); // gets the value of the field (ie. MecDriveBase instance or null if it hasn't been initialized)
                    if (value != null) {
                        subassemblies.add((Subassembly) value);
                    }
                } catch (IllegalAccessException ignored) {}
            }
        }

        return subassemblies.toArray(new Subassembly[0]);
    }

    public void log(String message) {
        RobotLog.e(message);
        if (logWriter != null) {
            try {
                logWriter.write(dateFormat.format(new Date()) + " " + message + "\n");
                logWriter.flush();
            } catch (IOException e) {
                RobotLog.e("Failed writing to log: ", e);
            }
        }
    }

    public void stop() {
        if (logWriter != null) {
            try {
                logWriter.close();
            } catch (IOException e) {
                RobotLog.e("Failed closing log file", e);
            }
        }
    }

    public Set<String> getNormalizedIssues() {
        return new HashSet<>(prevNormalizedIssues); // issues are up to date, ignore the prev
    }
}