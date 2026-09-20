package frc.robot.subsystems;

import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Font;
import java.awt.Graphics2D;
import java.awt.RenderingHints;
import java.awt.geom.Arc2D;
import java.awt.image.BufferedImage;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Locale;

public final class LidarScanView {

    public static final int WIDTH = 640;
    public static final int HEIGHT = 640;

    public static final class Settings {
        public final double fieldOfView;
        public final double forwardAngle;
        public final double minRange;
        public final double maxRange;

        public Settings(
                double fieldOfView,
                double forwardAngle,
                double minRange,
                double maxRange) {

            this.fieldOfView =
                    clamp(finiteOr(fieldOfView, 180), 30, 360);

            this.forwardAngle =
                    wrap(finiteOr(forwardAngle, 0));

            this.minRange =
                    clamp(finiteOr(minRange, 0.15), 0.12, 7.9);

            this.maxRange =
                    clamp(finiteOr(maxRange, 4), this.minRange + 0.1, 8);
        }
    }

    public static final class Point {
        public final double bearing;
        public final double distance;
        public final double x;
        public final double y;

        Point(double bearing, double distance) {
            this.bearing = bearing;
            this.distance = distance;

            // Positive x = right. Positive y = forward.
            this.x = distance * Math.sin(Math.toRadians(bearing));
            this.y = distance * Math.cos(Math.toRadians(bearing));
        }
    }

    public static final class Scan {
        public final List<Point> points;
        public final double nearest;
        public final double frontNearest;

        Scan(List<Point> points, double nearest, double frontNearest) {
            this.points = Collections.unmodifiableList(points);
            this.nearest = nearest;
            this.frontNearest = frontNearest;
        }
    }

    public static Scan filter(
            double[] angles,
            double[] distancesMm,
            Settings settings) {

        List<Point> points = new ArrayList<>();

        double nearest = Double.POSITIVE_INFINITY;
        double frontNearest = Double.POSITIVE_INFINITY;

        if (angles != null
                && distancesMm != null
                && angles.length == distancesMm.length) {

            for (int i = 0; i < angles.length; ++i) {
                if (!Double.isFinite(angles[i])
                        || !Double.isFinite(distancesMm[i])) {
                    continue;
                }

                double distance = distancesMm[i] / 1000.0;
                double bearing = wrap(
                        angles[i] - settings.forwardAngle);

                if (distance < settings.minRange
                        || distance > settings.maxRange
                        || Math.abs(bearing) > settings.fieldOfView / 2.0) {
                    continue;
                }

                points.add(new Point(bearing, distance));
                nearest = Math.min(nearest, distance);

                if (Math.abs(bearing) <= 15) {
                    frontNearest = Math.min(frontNearest, distance);
                }
            }
        }

        return new Scan(
                points,
                Double.isFinite(nearest) ? nearest : -1,
                Double.isFinite(frontNearest) ? frontNearest : -1
        );
    }

    public static BufferedImage draw(
            Scan scan,
            Settings settings,
            String status,
            boolean usable) {

        BufferedImage frame = new BufferedImage(
                WIDTH,
                HEIGHT,
                BufferedImage.TYPE_3BYTE_BGR
        );

        Graphics2D g = frame.createGraphics();

        g.setRenderingHint(
                RenderingHints.KEY_ANTIALIASING,
                RenderingHints.VALUE_ANTIALIAS_ON
        );

        g.setColor(new Color(12, 20, 31));
        g.fillRect(0, 0, WIDTH, HEIGHT);

        text(
                g, "LiDAR / FRONT VIEW",
                24, 32, 21,
                new Color(228, 240, 249)
        );

        text(
                g,
                String.format(
                        Locale.US,
                        "%.0f deg view   |   %.2f - %.1f m",
                        settings.fieldOfView,
                        settings.minRange,
                        settings.maxRange
                ),
                24, 56, 14,
                new Color(153, 176, 192)
        );

        int cx = 320;
        int cy = 334;
        int radius = 232;

        double scale = radius / settings.maxRange;

        g.setColor(new Color(23, 34, 46));
        g.fillOval(
                cx - radius,
                cy - radius,
                radius * 2,
                radius * 2
        );

        g.setColor(new Color(23, 54, 65));

        g.fill(new Arc2D.Double(
                cx - radius,
                cy - radius,
                radius * 2,
                radius * 2,
                90 - settings.fieldOfView / 2,
                settings.fieldOfView,
                Arc2D.PIE
        ));

        g.setStroke(new BasicStroke(1));

        for (int ring = 1; ring <= 4; ++ring) {
            int r = radius * ring / 4;

            g.setColor(new Color(50, 72, 86));
            g.drawOval(cx - r, cy - r, 2 * r, 2 * r);

            text(
                    g,
                    String.format(
                            Locale.US,
                            "%.1fm",
                            settings.maxRange * ring / 4
                    ),
                    cx + 6,
                    cy - r + 15,
                    11,
                    new Color(139, 167, 181)
            );
        }

        g.setColor(new Color(50, 72, 86));

        g.drawLine(cx - radius, cy, cx + radius, cy);
        g.drawLine(cx, cy - radius, cx, cy + radius);

        text(g, "FRONT", cx - 23, cy - radius - 12, 13, Color.WHITE);

        text(
                g, "LEFT",
                18, cy + 5, 12,
                new Color(153, 176, 192)
        );

        text(
                g, "RIGHT",
                573, cy + 5, 12,
                new Color(153, 176, 192)
        );

        if (settings.fieldOfView <= 180) {
            text(
                    g, "REAR EXCLUDED",
                    cx - 61, cy + 150, 14,
                    new Color(144, 151, 163)
            );
        }

        if (usable) {
            g.setColor(new Color(62, 231, 213));

            for (Point p : scan.points) {
                int x = (int) Math.round(cx + p.x * scale);
                int y = (int) Math.round(cy - p.y * scale);

                g.fillOval(x - 2, y - 2, 5, 5);
            }
        }

        g.setColor(new Color(249, 194, 94));

        g.fillPolygon(
                new int[]{cx, cx - 7, cx + 7},
                new int[]{cy - 12, cy + 7, cy + 7},
                3
        );

        String measurement;

        if (usable && !scan.points.isEmpty()) {
            measurement = String.format(
                    Locale.US,
                    "%d returns   |   nearest %.2f m",
                    scan.points.size(),
                    scan.nearest
            );
        } else {
            measurement = "No valid returns shown";
        }

        text(
                g, measurement,
                24, 596, 16,
                new Color(228, 240, 249)
        );

        String shortStatus = status.length() > 77
                ? status.substring(0, 74) + "..."
                : status;

        text(
                g, shortStatus,
                24, 622, 12,
                usable
                        ? new Color(153, 176, 192)
                        : new Color(249, 194, 94)
        );

        g.dispose();

        return frame;
    }

    private static void text(
            Graphics2D g,
            String value,
            int x,
            int y,
            int size,
            Color color) {

        g.setFont(new Font(Font.SANS_SERIF, Font.PLAIN, size));
        g.setColor(color);
        g.drawString(value, x, y);
    }

    public static double wrap(double angle) {
        return ((angle % 360) + 540) % 360 - 180;
    }

    private static double finiteOr(double value, double fallback) {
        return Double.isFinite(value) ? value : fallback;
    }

    private static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}