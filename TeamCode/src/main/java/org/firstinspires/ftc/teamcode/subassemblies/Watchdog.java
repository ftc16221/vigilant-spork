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
import java.time.LocalDateTime;
import java.util.ArrayList;
import java.util.HashSet;
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