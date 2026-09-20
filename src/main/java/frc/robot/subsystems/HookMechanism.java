package frc.robot.subsystems;

import com.studica.frc.Servo;
import com.studica.frc.TitanQuad;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.util.function.LongSupplier;

/**
 * Normal controller operation. Install with the matching RobotContainer.
 * User traced both current VMX input cables after swapping power-block inputs:
 * small hand = physical CH18 / PWM6; whole-arm lift = physical CH19 / PWM7.
 * Finger assignments remain PWM4/5. Base = Titan M3.
 * These assignments are fixed; old selector preferences are ignored.
 *
 * Requires standard POSITION-mode servos. First activation requests 150 degrees
 * from an unknown physical position. Support the unloaded arm and establish
 * mounted travel limits before normal use. Bounds 0..300 are electrical limits,
 * not verified mechanical limits. Disabling signals may release holding torque.
 * Robot.robotPeriodic must keep running CommandScheduler.run().
 */
public class HookMechanism extends SubsystemBase {
    private static final int LIFT_PWM_CHANNEL = 7;
    private static final int WRIST_PWM_CHANNEL = 6;
    // Uses DriveTrain's controller. Change only if the base has a separate Titan.
    private static final int BASE_TITAN_CAN_ID = Constants.TITAN_ID;
    // User confirmed the large left/right arm motor is wired to Titan M3.
    private static final int BASE_MOTOR_PORT = 3;
    private static final boolean BASE_INVERTED = false;

    private static final double SERVO_DEGREES_PER_SECOND = 60.0;
    private static final double BASE_MAX_OUTPUT = 0.15;
    private static final long REQUEST_TIMEOUT_NS = 100_000_000L;
    private static final double MAX_UPDATE_SECONDS = 0.05;

    // Keep the already-working software channels and motion parameters.
    private final Joint port4 = new Joint("Existing control / PWM 4", 4, 150, 0, 300);
    private final Joint port5 = new Joint("Existing control / PWM 5", 5, 150, 0, 300);
    private final Joint armLift = new Joint("Whole arm lift",
        LIFT_PWM_CHANNEL, 150, 0, 300, 20.0, 40.0);
    private final Joint smallHook = new Joint("Small hand", WRIST_PWM_CHANNEL, 150, 0, 300);
    private final Joint[] joints = {port4, port5, smallHook, armLift};
    private boolean wristReady, liftReady;
    private final BaseRotation armRotation;
    private final LongSupplier nanoClock;
    private long lastPeriodicNs;

    public HookMechanism() {
        this(System::nanoTime, BASE_TITAN_CAN_ID, BASE_MOTOR_PORT);
    }

    // Clock and configuration injection for deterministic software tests.
    HookMechanism(LongSupplier clock, int baseCanId, int basePort) {
        nanoClock = clock;
        lastPeriodicNs = clock.getAsLong();
        armRotation = new BaseRotation(baseCanId, basePort);
        SmartDashboard.putString("Arm/Program", "NORMAL CONTROLS v6 - lift CH19, wrist CH18");
        SmartDashboard.putString("Arm/Base configuration", armRotation.wheelOutput
            ? "INACTIVE: base output conflicts with a DriveTrain wheel output"
            : armRotation.configured
                ? "Titan CAN " + baseCanId + " / M" + basePort + " (verify wiring)"
                : "INACTIVE: set BASE_MOTOR_PORT and verify Titan CAN ID");
        SmartDashboard.putNumber("Arm/Lift PWM", LIFT_PWM_CHANNEL);
        SmartDashboard.putString("Arm/Lift configuration", "Whole arm: PWM7 / physical CH19");
        SmartDashboard.putNumber("Arm/Wrist PWM", WRIST_PWM_CHANNEL);
        SmartDashboard.putString("Arm/Wrist configuration", "Small hand: PWM6 / physical CH18");
        SmartDashboard.putBoolean("Arm/Selecting wrist", false);
        SmartDashboard.putBoolean("Arm/Selecting lift", false);
        SmartDashboard.putNumber("Arm/Lift input", 0);
        SmartDashboard.putNumber("Arm/Wrist input", 0);
        SmartDashboard.putNumber("Arm/Base input", 0);
    }

    public void setPort4Speed(double speed) { request(port4, speed); }
    public void setPort5Speed(double speed) { request(port5, speed); }
    public void setArmLiftSpeed(double speed) {
        SmartDashboard.putNumber("Arm/Lift input", clean(speed));
        if (!manualModeEnabled()) {
            liftReady = false;
            request(armLift, 0);
            return;
        }
        if (clean(speed) == 0) { liftReady = true; }
        request(armLift, liftReady ? speed : 0);
    }

    // Retain port methods while routing each channel through its single owner.
    public void setPort3Speed(double speed) { setSmallHookSpeed(speed); }
    public void setPort6Speed(double speed) { setSmallHookSpeed(speed); }
    public void setPort7Speed(double speed) { setArmLiftSpeed(speed); }

    public void setSmallHookSpeed(double speed) {
        SmartDashboard.putNumber("Arm/Wrist input", clean(speed));
        if (!manualModeEnabled()) {
            wristReady = false;
            request(smallHook, 0);
            return;
        }
        if (clean(speed) == 0) { wristReady = true; }
        request(smallHook, wristReady ? speed : 0);
    }

    public void setArmRotationSpeed(double speed) {
        SmartDashboard.putNumber("Arm/Base input", clean(speed));
        if (manualModeEnabled()) {
            armRotation.speed = clean(speed);
            armRotation.requestNs = nanoClock.getAsLong();
            armRotation.hasRequest = true;
        } else {
            armRotation.disable();
        }
    }

    private static double clean(double value) {
        return Double.isFinite(value) ? Math.max(-1, Math.min(1, value)) : 0;
    }

    private static boolean manualModeEnabled() {
        return RobotState.isEnabled() && !RobotState.isAutonomous() && !RobotState.isTest();
    }

    private static boolean fresh(boolean hasRequest, long now, long requestNs) {
        return hasRequest && now >= requestNs && now - requestNs < REQUEST_TIMEOUT_NS;
    }

    private void request(Joint joint, double speed) {
        if (!manualModeEnabled()) {
            joint.disable();
            return;
        }
        joint.speed = clean(speed);
        joint.requestNs = nanoClock.getAsLong();
        joint.hasRequest = true;
    }

    @Override
    public void periodic() {
        long now = nanoClock.getAsLong();
        double seconds = Math.max(0, Math.min(MAX_UPDATE_SECONDS,
            (now - lastPeriodicNs) / 1_000_000_000.0));
        lastPeriodicNs = now;
        boolean enabled = manualModeEnabled();
        if (!enabled) { wristReady = false; liftReady = false; }
        for (Joint joint : joints) {
            if (joint == null) { continue; }
            if (enabled) { joint.update(now, seconds); }
            else { joint.disable(); }
            SmartDashboard.putNumber("Arm/" + joint.name + "/Target deg", joint.target);
            SmartDashboard.putBoolean("Arm/" + joint.name + "/Active", joint.active);
            SmartDashboard.putNumber("Arm/" + joint.name + "/Requested speed",
                enabled && fresh(joint.hasRequest, now, joint.requestNs) ? joint.speed : 0);
        }
        if (enabled) { armRotation.update(now); }
        else { armRotation.disable(); }
        SmartDashboard.putNumber("Arm/Base output", armRotation.lastOutput);
    }

    private static final class Joint {
        final String name;
        final int channel;
        final double minimum, maximum, maxRate, maxAcceleration;
        double target, speed, velocity;
        long requestNs;
        boolean hasRequest, active, failed;
        Servo output;

        Joint(String name, int channel, double start, double minimum, double maximum) {
            this(name, channel, start, minimum, maximum, SERVO_DEGREES_PER_SECOND,
                Double.POSITIVE_INFINITY);
        }

        Joint(String name, int channel, double start, double minimum, double maximum,
                double maxRate, double maxAcceleration) {
            this.maxRate = maxRate;
            this.maxAcceleration = maxAcceleration;
            if (!Double.isFinite(start) || !Double.isFinite(minimum)
                    || !Double.isFinite(maximum) || minimum < 0 || maximum > 300
                    || minimum > start || start > maximum) {
                throw new IllegalArgumentException("Invalid joint angles: " + name);
            }
            this.name = name; this.channel = channel; target = start;
            this.minimum = minimum; this.maximum = maximum;
        }

        void update(long now, double seconds) {
            if (failed || !fresh(hasRequest, now, requestNs) || speed == 0) {
                velocity = 0;
                return; // PWM remains on the last target while Teleop is enabled.
            }
            if (output == null) {
                try {
                    output = new Servo(channel);
                    output.setDisabled();
                } catch (RuntimeException error) {
                    failed = true;
                    DriverStation.reportError("Cannot create " + name + " on PWM "
                        + channel + ": " + error.getMessage(), false);
                    return;
                }
            }
            if (active && seconds > 0) {
                double desiredVelocity = speed * maxRate;
                double maxChange = maxAcceleration * seconds;
                velocity += Math.max(-maxChange, Math.min(maxChange, desiredVelocity - velocity));
                target = Math.max(minimum, Math.min(maximum, target + velocity * seconds));
            } else { velocity = 0; }
            output.setAngle(target);
            active = true;
        }

        void disable() {
            speed = 0; velocity = 0; hasRequest = false; active = false;
            if (output != null) { output.setDisabled(); }
        }
    }

    private static final class BaseRotation {
        final int canId, port;
        final boolean configured, wheelOutput;
        TitanQuad output;
        double speed, lastOutput;
        long requestNs;
        boolean hasRequest, failed;

        BaseRotation(int canId, int port) {
            this.canId = canId; this.port = port;
            wheelOutput = canId == Constants.TITAN_ID
                && (port == Constants.LEFT_MOTOR_PORT || port == Constants.RIGHT_MOTOR_PORT);
            configured = canId >= 0 && port >= 0 && port <= 3 && !wheelOutput;
        }

        void update(long now) {
            double value = fresh(hasRequest, now, requestNs) ? speed * BASE_MAX_OUTPUT : 0;
            if (!configured || failed) { lastOutput = 0; return; }
            // Never create a motor on startup or for a zero/expired request.
            if (output == null && value != 0) {
                try {
                    output = new TitanQuad(canId, port);
                    output.set(0);
                    output.setInverted(BASE_INVERTED);
                } catch (RuntimeException error) {
                    failed = true;
                    DriverStation.reportError("Cannot create arm base motor: "
                        + error.getMessage(), false);
                    return;
                }
            }
            if (output != null) { output.set(value); }
            lastOutput = output == null ? 0 : value;
        }

        void disable() {
            speed = 0; hasRequest = false; lastOutput = 0;
            if (output != null) { output.set(0); }
        }
    }
}
