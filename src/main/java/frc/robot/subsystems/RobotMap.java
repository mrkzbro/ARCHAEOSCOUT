package frc.robot.subsystems;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.cscore.CvSource;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayDeque;
import java.util.Deque;
import java.util.Locale;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

/** WPILib 2020: displays the existing Position entries; never commands motors.
 * Local frame: +X = initial forward, +Y = initial left, +heading = left turn.
 * Trail is held in memory only. This is wheel odometry, not a terrain map.
 */
public final class RobotMap extends SubsystemBase implements AutoCloseable {
    private static final String P = "Robot Map/";
    private static final int MAX_POINTS = 2000;
    private static final Scalar BG = new Scalar(31, 20, 12);
    private static final Scalar GRID = new Scalar(65, 53, 40);
    private static final Scalar WHITE = new Scalar(235, 235, 235);
    private static final Scalar CYAN = new Scalar(210, 230, 60);
    private static final Scalar GOLD = new Scalar(80, 190, 250);
    private volatile Snapshot latest;
    private volatile boolean closed;
    private final Thread renderer;
    private long lastSample, generation;
    private boolean previousTicksNonzero;

    public RobotMap() {
        SmartDashboard.putNumber(P + "Half width m", 5);
        SmartDashboard.putBoolean(P + "Clear trail", false);
        SmartDashboard.putString(P + "Video status", "Starting map");
        renderer = new Thread(this::renderLoop, "ArchaeoScout-map");
        renderer.setDaemon(true);
        renderer.start();
    }

    @Override
    public void periodic() {
        if (closed) return;
        long now = System.nanoTime();
        if (now - lastSample < 100_000_000L) return;
        lastSample = now;
        double x = SmartDashboard.getNumber("Position/X m", Double.NaN);
        double y = SmartDashboard.getNumber("Position/Y m", Double.NaN);
        double h = SmartDashboard.getNumber("Position/Heading deg", Double.NaN);
        boolean valid = SmartDashboard.getBoolean("Position/Calibrated", false)
                && Double.isFinite(x) && Double.isFinite(y) && Double.isFinite(h)
                && Math.abs(x) < 1e6 && Math.abs(y) < 1e6;
        double width = SmartDashboard.getNumber(P + "Half width m", 5);
        width = Double.isFinite(width) ? Math.max(1, Math.min(1000, width)) : 5;

        // Recognize the existing DriveTrain's published software reset, even
        // when its Reset origin toggle has already returned to false.
        double l = SmartDashboard.getNumber("Position/Left ticks", Double.NaN);
        double r = SmartDashboard.getNumber("Position/Right ticks", Double.NaN);
        boolean ticksValid = Double.isFinite(l) && Double.isFinite(r);
        boolean ticksNonzero = ticksValid && (l != 0 || r != 0);
        boolean originReset = valid && ticksValid && previousTicksNonzero
                && !ticksNonzero && x == 0 && y == 0 && h == 0;
        previousTicksNonzero = ticksNonzero;
        if (originReset || SmartDashboard.getBoolean(P + "Clear trail", false)) {
            generation++;
            SmartDashboard.putBoolean(P + "Clear trail", false);
        }
        latest = new Snapshot(x, y, h, width, valid, now, generation);
    }

    private void renderLoop() {
        CvSource source = null;
        Mat frame = null;
        Deque<Snapshot> trail = new ArrayDeque<>();
        long seenGeneration = -1;
        try {
            Thread.sleep(2000); // Let the robot start before loading display code.
            if (closed) return;
            System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
            frame = new Mat(480, 480, CvType.CV_8UC3);
            source = CameraServer.getInstance().putVideo("ArchaeoScout Map", 480, 480);
            while (!closed) {
                Snapshot s = latest;
                boolean usable = s != null && s.valid
                        && System.nanoTime() - s.time < 1_000_000_000L;
                if (s != null && s.generation != seenGeneration) {
                    trail.clear();
                    seenGeneration = s.generation;
                }
                if (usable) {
                    Snapshot previous = trail.peekLast();
                    if (previous == null || Math.hypot(s.x - previous.x, s.y - previous.y) >= 0.03) {
                        if (trail.size() >= MAX_POINTS) trail.removeFirst();
                        trail.addLast(s);
                    }
                } else {
                    // Do not join a trail across an interruption in valid data.
                    trail.clear();
                }
                draw(frame, s, usable, trail);
                source.putFrame(frame);
                SmartDashboard.putNumber(P + "Trail points", trail.size());
                SmartDashboard.putString(P + "Video status", usable
                        ? "Map streaming: wheel position estimate"
                        : "Map streaming: waiting for calibrated position updates");
                Thread.sleep(333); // About 3 FPS, alongside LiDAR and RGB video.
            }
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        } catch (Exception | LinkageError e) {
            SmartDashboard.putString(P + "Video status", "Map error: " + e.toString());
        } finally {
            if (frame != null) frame.release();
            if (source != null) source.close();
        }
    }

    private static void draw(Mat frame, Snapshot s, boolean usable, Deque<Snapshot> trail) {
        frame.setTo(BG);
        double width = s == null ? 5 : s.width;
        double scale = 170.0 / width;
        // Plot box: horizontal 70..410, vertical 90..430; origin (240,260).
        for (int i = -5; i <= 5; i++) {
            Scalar color = i == 0 ? WHITE : GRID;
            double offset = i * 34.0;
            Imgproc.line(frame, new Point(70, 260 + offset), new Point(410, 260 + offset), color, 1);
            Imgproc.line(frame, new Point(240 + offset, 90), new Point(240 + offset, 430), color, 1);
        }
        text(frame, "ARCHAEOSCOUT / LIVE MAP", 12, 24, 0.60, WHITE);
        text(frame, String.format(Locale.US, "View +/-%.1fm | grid %.2fm | wheel estimate", width, width / 5),
                12, 46, 0.38, WHITE);
        text(frame, "+X / initial forward", 166, 77, 0.38, WHITE);
        text(frame, "+Y", 37, 261, 0.4, WHITE);
        text(frame, "-Y", 417, 261, 0.4, WHITE);
        Snapshot previous = null;
        for (Snapshot p : trail) {
            if (previous != null && inside(previous, width) && inside(p, width)) {
                Imgproc.line(frame, project(previous.x, previous.y, scale),
                        project(p.x, p.y, scale), CYAN, 2);
            }
            previous = p;
        }
        Imgproc.circle(frame, new Point(240, 260), 4, WHITE, 1);
        if (usable) {
            if (inside(s, width)) {
                Point center = project(s.x, s.y, scale);
                double a = Math.toRadians(s.heading);
                Point tip = new Point(center.x - 17 * Math.sin(a), center.y - 17 * Math.cos(a));
                Imgproc.circle(frame, center, 5, GOLD, -1);
                Imgproc.line(frame, center, tip, GOLD, 3);
                Imgproc.circle(frame, tip, 2, WHITE, -1);
            } else {
                text(frame, "Outside view: increase Half width m", 84, 108, 0.4, GOLD);
            }
            text(frame, String.format(Locale.US, "X %.2fm   Y %.2fm   Heading %.1f deg", s.x, s.y, s.heading),
                    12, 451, 0.43, WHITE);
        } else {
            text(frame, "Waiting for calibrated position updates", 12, 451, 0.43, GOLD);
        }
        text(frame, "White: origin | Cyan: trail | Gold: robot", 12, 472, 0.39, WHITE);
    }

    private static Point project(double x, double y, double scale) {
        return new Point(240 - y * scale, 260 - x * scale);
    }

    private static boolean inside(Snapshot s, double width) {
        return Math.abs(s.x) <= width && Math.abs(s.y) <= width;
    }

    private static void text(Mat frame, String value, int x, int y, double size, Scalar color) {
        Imgproc.putText(frame, value, new Point(x, y), 0 /* Hershey Simplex */, size, color, 1);
    }

    @Override
    public void close() {
        closed = true;
        renderer.interrupt();
    }

    private static final class Snapshot {
        final double x, y, heading, width;
        final boolean valid;
        final long time, generation;
        Snapshot(double x, double y, double heading, double width, boolean valid, long time, long generation) {
            this.x = x; this.y = y; this.heading = heading; this.width = width;
            this.valid = valid; this.time = time; this.generation = generation;
        }
    }
}