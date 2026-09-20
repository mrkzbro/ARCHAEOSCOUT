package frc.robot.subsystems;

import com.studica.frc.TitanQuad;
import com.studica.frc.TitanQuadEncoder;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/** Local wheel-encoder odometry for the existing two-motor drive.
 * X is initial forward, Y is initial left; heading is positive counterclockwise.
 * Calibration values are signed encoder counts for one metre FORWARD.
 * Position is an estimate: sideways motion and wheel slip are not measured.
 */
public class DriveTrain extends SubsystemBase {
    private final TitanQuad leftMotor;
    private final TitanQuad rightMotor;
    private TitanQuadEncoder leftEncoder;
    private TitanQuadEncoder rightEncoder;
    private boolean encodersOK;
    private boolean haveBaseline;
    private String error = "";
    private double previousLeft, previousRight, zeroLeft, zeroRight;
    private double x, y, heading;
    private double leftScale, rightScale, trackWidth;
    private int publishCounter;
    private static final String P = "Position/";

    public DriveTrain() {
        leftMotor = new TitanQuad(Constants.TITAN_ID, Constants.LEFT_MOTOR_PORT);
        rightMotor = new TitanQuad(Constants.TITAN_ID, Constants.RIGHT_MOTOR_PORT);
        rightMotor.setInverted(true);

        // Preserve settings within this robot-program session. These entries
        // are not guaranteed to survive a restart: record calibrated values.
        SmartDashboard.putNumber(P + "Left ticks per meter", 0);
        SmartDashboard.putNumber(P + "Right ticks per meter", 0);
        SmartDashboard.putNumber(P + "Track width m", 0);
        SmartDashboard.putBoolean(P + "Reset origin", false);
        SmartDashboard.putString(P + "Status", "Checking encoders");
        SmartDashboard.putNumber(P + "X m", 0);
        SmartDashboard.putNumber(P + "Y m", 0);
        SmartDashboard.putNumber(P + "Heading deg", 0);
        SmartDashboard.putBoolean(P + "Calibrated", false);
        try {
            // A distance-per-tick of 1.0 makes the reported distance a count.
            leftEncoder = new TitanQuadEncoder(leftMotor, Constants.LEFT_MOTOR_PORT, 1.0);
            rightEncoder = new TitanQuadEncoder(rightMotor, Constants.RIGHT_MOTOR_PORT, 1.0);
            encodersOK = true;
        } catch (RuntimeException | LinkageError e) {
            error = e.toString();
        }
    }

    // Original movement methods and motor inversion are preserved.
    public void drive(double speed) {
        leftMotor.set(speed);
        rightMotor.set(speed);
    }

    public void stop() {
        drive(0);
    }

    public void arcadeDrive(double forward, double turn) {
        double leftSpeed = forward + turn;
        double rightSpeed = forward - turn;
        leftSpeed = Math.max(-1.0, Math.min(1.0, leftSpeed));
        rightSpeed = Math.max(-1.0, Math.min(1.0, rightSpeed));
        leftMotor.set(leftSpeed);
        rightMotor.set(rightSpeed);
    }

    private boolean calibrated() {
        return Double.isFinite(leftScale) && Math.abs(leftScale) > 0
            && Double.isFinite(rightScale) && Math.abs(rightScale) > 0
            && Double.isFinite(trackWidth) && trackWidth > 0;
    }

    private void reset(double left, double right) {
        x = y = heading = 0;
        previousLeft = zeroLeft = left;
        previousRight = zeroRight = right;
        haveBaseline = true;
    }

    @Override
    public void periodic() {
        if (!encodersOK) {
            if (++publishCounter % 10 == 0) {
                SmartDashboard.putBoolean(P + "Calibrated", false);
                SmartDashboard.putString(P + "Status", "Encoder error: " + error);
            }
            return;
        }
        try {
            double left = leftEncoder.getEncoderDistance();
            double right = rightEncoder.getEncoderDistance();
            if (!Double.isFinite(left) || !Double.isFinite(right)) {
                throw new IllegalStateException("Non-finite encoder reading");
            }
            if (!haveBaseline) reset(left, right);

            // Apply calibration and origin reset only while the robot is disabled.
            if (RobotState.isDisabled()) {
                double ls = SmartDashboard.getNumber(P + "Left ticks per meter", 0);
                double rs = SmartDashboard.getNumber(P + "Right ticks per meter", 0);
                double width = SmartDashboard.getNumber(P + "Track width m", 0);
                boolean changed = Double.compare(ls, leftScale) != 0
                    || Double.compare(rs, rightScale) != 0
                    || Double.compare(width, trackWidth) != 0;
                leftScale = ls;
                rightScale = rs;
                trackWidth = width;
                if (changed || SmartDashboard.getBoolean(P + "Reset origin", false)) {
                    reset(left, right);
                    SmartDashboard.putBoolean(P + "Reset origin", false);
                }
            }

            double dl = left - previousLeft;
            double dr = right - previousRight;
            previousLeft = left;
            previousRight = right;
            if (calibrated()) {
                dl /= leftScale;
                dr /= rightScale;
                double distance = (dl + dr) / 2.0;
                double rotation = (dr - dl) / trackWidth;
                // Exact constant-curvature integration, including straight travel.
                double half = rotation / 2.0;
                double arcScale = Math.abs(half) < 1e-9 ? 1.0 : Math.sin(half) / half;
                x += distance * arcScale * Math.cos(heading + half);
                y += distance * arcScale * Math.sin(heading + half);
                heading = Math.atan2(Math.sin(heading + rotation), Math.cos(heading + rotation));
            }
            if (++publishCounter % 5 == 0) {
                SmartDashboard.putNumber(P + "Left ticks", left - zeroLeft);
                SmartDashboard.putNumber(P + "Right ticks", right - zeroRight);
                SmartDashboard.putNumber(P + "X m", x);
                SmartDashboard.putNumber(P + "Y m", y);
                SmartDashboard.putNumber(P + "Heading deg", Math.toDegrees(heading));
                SmartDashboard.putBoolean(P + "Calibrated", calibrated());
                SmartDashboard.putString(P + "Status", calibrated()
                    ? "Wheel estimate: verify both encoders; slip causes drift"
                    : "Calibrate: drive 1 m, record signed ticks; enter scales and width while disabled");
            }
        } catch (RuntimeException | LinkageError e) {
            encodersOK = false;
            error = e.toString();
            SmartDashboard.putBoolean(P + "Calibrated", false);
            SmartDashboard.putString(P + "Status", "Encoder error: " + error);
        }
    }
}