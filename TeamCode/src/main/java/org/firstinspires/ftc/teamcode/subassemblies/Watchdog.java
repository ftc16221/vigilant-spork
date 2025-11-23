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
import java.time.LocalDateTime;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

@Config
public class Watchdog {

    public static boolean WRITE_TO_LOG_FILE = false;
    public static String LOG_FILE_PATHNAME = ""; // TODO: find spot for log file
    public static String LOG_FILE_NAME = "watchdog.log";

    private final File logFile;
    private final Set<String> prevIssues = new HashSet<>();

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
            Set<String> subIssues = subassembly.checkIssues();
            // check if a new issue is discovered, and if so, add it log and activeIssues
            if (subIssues != null) {
                issues.addAll(subIssues);
            }
        }

        for (String issue : issues) {
            // add new issues
            if (!prevIssues.contains(issue)) {
                prevIssues.add(issue);
                log("ISSUE DISCOVERED: " + issue);
            }

            // add all issues to telemetry
            telemetry.addLine("Warning: " + issue);
        }

        // remove old issues, and display message that they have been cleared
        prevIssues.removeIf(i -> {
            boolean stillActive = issues.contains(i); // check if our tracked issues were polled this cycle
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
        if (WRITE_TO_LOG_FILE && logFile != null) {
            try (FileWriter writer = new FileWriter(logFile, true)) {
                writer.write(LocalDateTime.now() + " " + message + "\n");
            } catch (IOException e) {
                RobotLog.e("Failed writing to log: ", e);
            }
        }
    }

    public Set<String> getIssues() {
        return prevIssues; // issues are up to date, ignore the prev
    }
}