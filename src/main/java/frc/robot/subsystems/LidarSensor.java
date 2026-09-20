package frc.robot.subsystems;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.cscore.CvSource;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.lang.reflect.Array;
import java.lang.reflect.Field;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

/** USB Studica LiDAR -> filtered scan -> Shuffleboard camera stream.
 * Targets the project's WPILib 2020 API. Does not control the arm or drivetrain.
 */
public class LidarSensor extends SubsystemBase implements AutoCloseable {
    // Studica documentation: top USB 2.0 = kUSB1, bottom USB 2.0 = kUSB2.
    private static final String USB_PORT = "kUSB1";
    private static final long STALE_NS = 2_000_000_000L;
    private volatile boolean closed;
    private volatile boolean runRequested = true;
    private volatile LidarScanView.Settings settings = new LidarScanView.Settings(180, 0, .15, 4);
    private volatile Snapshot latest;
    private volatile String driverStatus = "Starting USB LiDAR";
    private volatile String videoStatus = "Starting LiDAR Front stream";
    private long lastDashboardUpdate;
    private final Thread reader;
    private final Thread renderer;

    public LidarSensor() {
        SmartDashboard.putBoolean("LiDAR/Run", true);
        SmartDashboard.putNumber("LiDAR/Field of view deg", 180);
        SmartDashboard.putNumber("LiDAR/Forward raw angle deg", 0);
        SmartDashboard.putNumber("LiDAR/Min range m", .15);
        SmartDashboard.putNumber("LiDAR/Max range m", 4);
        SmartDashboard.putString("LiDAR/USB port", USB_PORT);
        reader = daemon("Studica-LiDAR-reader", this::readLoop);
        renderer = daemon("Studica-LiDAR-view", this::renderLoop);
        reader.start();
        renderer.start();
    }

    @Override
    public void periodic() {
        long now = System.nanoTime();
        if (now - lastDashboardUpdate < 200_000_000L) return;
        lastDashboardUpdate = now;
        runRequested = SmartDashboard.getBoolean("LiDAR/Run", true);
        settings = new LidarScanView.Settings(
                SmartDashboard.getNumber("LiDAR/Field of view deg", 180),
                SmartDashboard.getNumber("LiDAR/Forward raw angle deg", 0),
                SmartDashboard.getNumber("LiDAR/Min range m", .15),
                SmartDashboard.getNumber("LiDAR/Max range m", 4));
        Snapshot snapshot = latest;
        boolean usable = usable(snapshot, now);
        LidarScanView.Scan scan = currentScan(snapshot, usable, settings);
        SmartDashboard.putString("LiDAR/Status", status(snapshot, usable));
        SmartDashboard.putString("LiDAR/Video status", videoStatus);
        SmartDashboard.putBoolean("LiDAR/Recent driver read", usable);
        SmartDashboard.putBoolean("LiDAR/Has front return", scan.frontNearest >= 0);
        SmartDashboard.putNumber("LiDAR/Shown points", scan.points.size());
        SmartDashboard.putNumber("LiDAR/Nearest in view m", scan.nearest);
        SmartDashboard.putNumber("LiDAR/Nearest front 30 deg m", scan.frontNearest);
        SmartDashboard.putNumber("LiDAR/Driver read age ms",
                snapshot == null ? -1 : (now - snapshot.receivedNs) / 1_000_000.0);
    }

    private void readLoop() {
        Driver driver = null;
        boolean spinning = false;
        try {
            while (!closed) {
                if (!runRequested) {
                    latest = null;
                    if (driver != null && spinning) { driver.stop(); spinning = false; }
                    driverStatus = "LiDAR stopped";
                    Thread.sleep(100);
                    continue;
                }
                if (driver == null) {
                    driver = new Driver(USB_PORT); // Vendor constructor starts rotation.
                    spinning = true;
                } else if (!spinning) {
                    driver.start();
                    spinning = true;
                }
                Snapshot snapshot = driver.read();
                if (runRequested && !closed) {
                    latest = snapshot;
                    driverStatus = "Driver reads active";
                }
                Thread.sleep(100);
            }
        } catch (InterruptedException interrupted) {
            Thread.currentThread().interrupt();
        } catch (Exception | LinkageError error) {
            latest = null;
            driverStatus = "LiDAR error: " + describe(error) + "; correct issue, restart robot code";
        } finally {
            latest = null;
            if (driver != null) {
                try { driver.stop(); } catch (Exception | LinkageError ignored) { }
            }
        }
    }

    private void renderLoop() {
        CvSource source = null;
        Mat frame = null;
        try {
            // Allow robot startup before initializing the display libraries.
            Thread.sleep(1500);
            if (closed) return;

            videoStage("1/4: loading OpenCV");
            System.loadLibrary(Core.NATIVE_LIBRARY_NAME);

            videoStage("2/4: allocating image");
            frame = new Mat(480, 480, CvType.CV_8UC3);

            videoStage("3/4: starting CameraServer");
            source = CameraServer.getInstance().putVideo("LiDAR Front", 480, 480);

            Scalar background = new Scalar(31, 20, 12);
            Scalar grid = new Scalar(85, 70, 50);
            Scalar white = new Scalar(235, 235, 235);
            Scalar dots = new Scalar(210, 230, 60);
            Point center = new Point(240, 245);
            boolean firstFrame = true;

            videoStage("4/4: drawing and sending first frame");
            while (!closed) {
                Snapshot snapshot = latest;
                LidarScanView.Settings config = settings;
                boolean fresh = usable(snapshot, System.nanoTime());
                LidarScanView.Scan scan = currentScan(snapshot, fresh, config);

                frame.setTo(background);
                double scale = 175.0 / config.maxRange;

                for (int ring = 1; ring <= 4; ring++) {
                    int radius = 175 * ring / 4;
                    Imgproc.circle(frame, center, radius, grid, 1);
                }
                Imgproc.line(frame, new Point(65, 245), new Point(415, 245), grid, 1);
                Imgproc.line(frame, new Point(240, 70), new Point(240, 420), grid, 1);

                if (config.fieldOfView < 360) {
                    double half = Math.toRadians(config.fieldOfView / 2.0);
                    double dx = 175 * Math.sin(half);
                    double dy = 175 * Math.cos(half);
                    Imgproc.line(frame, center, new Point(240 - dx, 245 - dy), white, 1);
                    Imgproc.line(frame, center, new Point(240 + dx, 245 - dy), white, 1);
                }

                for (LidarScanView.Point p : scan.points) {
                    Imgproc.circle(frame,
                            new Point(240 + p.x * scale, 245 - p.y * scale),
                            2, dots, -1);
                }
                Imgproc.circle(frame, center, 5, new Scalar(80, 190, 250), -1);

                Imgproc.putText(frame, "LiDAR FRONT", new Point(15, 25),
                        0 /* Hershey Simplex */, 0.65, white, 1);
                Imgproc.putText(frame,
                        String.format(java.util.Locale.US, "View %.0f deg | range %.1f m",
                                config.fieldOfView, config.maxRange),
                        new Point(15, 47), 0 /* Hershey Simplex */, 0.45, white, 1);
                Imgproc.putText(frame, "FRONT", new Point(215, 64),
                        0 /* Hershey Simplex */, 0.4, white, 1);
                if (config.fieldOfView <= 180) {
                    Imgproc.putText(frame, "REAR EXCLUDED", new Point(165, 355),
                            0 /* Hershey Simplex */, 0.5, grid, 1);
                }
                String reading = scan.nearest < 0 ? "No valid returns"
                        : String.format(java.util.Locale.US, "%d points | nearest %.2f m",
                                scan.points.size(), scan.nearest);
                Imgproc.putText(frame, reading, new Point(15, 445),
                        0 /* Hershey Simplex */, 0.45, white, 1);
                String message = status(snapshot, fresh);
                if (message.length() > 58) message = message.substring(0, 55) + "...";
                Imgproc.putText(frame, message, new Point(15, 466),
                        0 /* Hershey Simplex */, 0.35, white, 1);

                source.putFrame(frame);
                if (firstFrame) {
                    videoStage("LiDAR Front streaming");
                    firstFrame = false;
                }
                Thread.sleep(200);
            }
        } catch (InterruptedException interrupted) {
            Thread.currentThread().interrupt();
        } catch (Exception | LinkageError error) {
            videoStage("Video error: " + describe(error));
            error.printStackTrace();
        } finally {
            if (frame != null) frame.release();
            if (source != null) source.close();
        }
    }

    private void videoStage(String message) {
        videoStatus = message;
        SmartDashboard.putString("LiDAR/Video status", message);
        System.out.println("[LiDAR video] " + message);
        System.out.flush();
    }

    private boolean usable(Snapshot snapshot, long now) {
        return !closed && runRequested && snapshot != null && now - snapshot.receivedNs <= STALE_NS;
    }

    private LidarScanView.Scan currentScan(Snapshot snapshot, boolean usable, LidarScanView.Settings config) {
        return usable ? LidarScanView.filter(snapshot.angles, snapshot.distances, config)
                : LidarScanView.filter(null, null, config);
    }

    private String status(Snapshot snapshot, boolean usable) {
        if (closed) return "LiDAR service closed";
        if (!runRequested) return "LiDAR stop requested";
        if (snapshot != null && !usable) return "Driver read timed out; old points hidden";
        return driverStatus;
    }

    @Override
    public void close() {
        closed = true;
        latest = null;
        reader.interrupt();
        renderer.interrupt();
    }

    private static Thread daemon(String name, Runnable work) {
        Thread thread = new Thread(work, name);
        thread.setDaemon(true);
        return thread;
    }

    private static String describe(Throwable error) {
        while (error instanceof InvocationTargetException && error.getCause() != null) error = error.getCause();
        return error.getClass().getSimpleName() + ": " + String.valueOf(error.getMessage());
    }

    static final class Snapshot {
        final double[] angles, distances;
        final long receivedNs;
        Snapshot(double[] angles, double[] distances) {
            this.angles = angles;
            this.distances = distances;
            receivedNs = System.nanoTime();
        }
    }

    /** Uses only the documented Studica interface. Late binding lets older vendor
     * installations report a missing LiDAR driver without preventing robot startup.
     * A compatible Studica Java AND native LiDAR library must still be installed.
     */
    static final class Driver {
        private final Object lidar;
        private final Method getData, start, stop;
        private Field angleField, distanceField;

        Driver(String portName) throws Exception {
            Class<?> type;
            try {
                type = Class.forName("com.studica.frc.Lidar");
            } catch (ClassNotFoundException error) {
                throw new IllegalStateException("Studica library lacks com.studica.frc.Lidar; matching Java/native driver required", error);
            }
            Class<?> portType = Class.forName("com.studica.frc.Lidar$Port");
            Object port = portType.getField(portName).get(null);
            getData = type.getMethod("getData");
            start = type.getMethod("start");
            stop = type.getMethod("stop");
            lidar = type.getConstructor(portType).newInstance(port);
        }

        void start() throws Exception { start.invoke(lidar); }
        void stop() throws Exception { stop.invoke(lidar); }

        Snapshot read() throws Exception {
            Object data = getData.invoke(lidar);
            if (data == null) throw new IllegalStateException("getData returned null");
            if (angleField == null) {
                angleField = data.getClass().getField("angle");
                distanceField = data.getClass().getField("distance");
            }
            double[] angles = copyArray(angleField.get(data));
            double[] distances = copyArray(distanceField.get(data));
            if (angles.length != distances.length) throw new IllegalStateException("Scan angle/distance lengths differ");
            return new Snapshot(angles, distances);
        }

        private static double[] copyArray(Object array) {
            if (array == null || !array.getClass().isArray()) throw new IllegalStateException("Scan field is not an array");
            int length = Array.getLength(array);
            if (length > 8192) throw new IllegalStateException("Unexpected scan length " + length);
            double[] values = new double[length];
            for (int i = 0; i < length; ++i) {
                Object value = Array.get(array, i);
                if (!(value instanceof Number)) throw new IllegalStateException("Non-numeric scan value");
                values[i] = ((Number) value).doubleValue();
            }
            return values;
        }
    }
}
