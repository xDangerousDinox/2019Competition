package frc.util.logger;

import java.io.*;
import java.util.*;

public abstract class Logger {

    private static final String FILENAME = "jerrylog.txt";

    private static File logFile;
    private static FileOutputStream out;

    private static boolean ready = false;

    public static void initLogger() {
        try {
            Map<String, String> env = System.getenv();
            logFile = new File("home/lvuser/" + FILENAME);

            if (!logFile.exists()) {
                System.out.println(logFile.createNewFile());
            }

            out = new FileOutputStream(logFile);
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public static void log(String string) {
        if (ready) {
            try {
                out.write(string.getBytes());
            } catch (IOException e) {
                e.printStackTrace();
            }
        }
    }

}