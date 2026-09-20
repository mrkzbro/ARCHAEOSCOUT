package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.HookMechanism;
import frc.robot.subsystems.LidarSensor;
import frc.robot.subsystems.RobotMap;

public class RobotContainer {
    private final DriveTrain driveTrain = new DriveTrain();
    private final HookMechanism hookMechanism = new HookMechanism();
    private final Joystick controller = new Joystick(0);
    private final LidarSensor lidarSensor = new LidarSensor();
    private final RobotMap robotMap = new RobotMap();
    public RobotContainer() {
        driveTrain.setDefaultCommand(new RunCommand(
            () -> driveTrain.arcadeDrive(-controller.getRawAxis(1), controller.getRawAxis(0)),
            driveTrain
        ));

        hookMechanism.setDefaultCommand(new RunCommand(
            () -> {
                // Port 4 — LB / LT
                if (controller.getRawButton(5)) {
                    hookMechanism.setPort4Speed(0.5);
                } else if (controller.getRawAxis(2) > 0.5) {
                    hookMechanism.setPort4Speed(-0.5);
                } else {
                    hookMechanism.setPort4Speed(0);
                }

                // Whole-arm lift: physical CH19 = PWM7 (user-traced input cable).
                if (controller.getRawButton(6)) {
                    hookMechanism.setArmLiftSpeed(1.0);
                } else if (controller.getRawAxis(3) > 0.5) {
                    hookMechanism.setArmLiftSpeed(-1.0);
                } else {
                    hookMechanism.setArmLiftSpeed(0);
                }

                // Port 5 — Y / A
                if (controller.getRawButton(4)) {
                    hookMechanism.setPort5Speed(0.5);
                } else if (controller.getRawButton(1)) {
                    hookMechanism.setPort5Speed(-0.5);
                } else {
                    hookMechanism.setPort5Speed(0);
                }

                // Small hand: physical CH18 = PWM6; separate from whole-arm lift.
                if (controller.getPOV() == 0) {
                    hookMechanism.setSmallHookSpeed(0.5);
                } else if (controller.getPOV() == 180) {
                    hookMechanism.setSmallHookSpeed(-0.5);
                } else {
                    hookMechanism.setSmallHookSpeed(0);
                }

                // Added: whole-arm left/right through the configured Titan output.
                if (controller.getPOV() == 270) {
                    hookMechanism.setArmRotationSpeed(-1.0);
                } else if (controller.getPOV() == 90) {
                    hookMechanism.setArmRotationSpeed(1.0);
                } else {
                    hookMechanism.setArmRotationSpeed(0);
                }

                // Read-only input diagnostics for the earlier RB/RT issue.
                SmartDashboard.putBoolean("Arm/Input/RB button 6", controller.getRawButton(6));
                SmartDashboard.putNumber("Arm/Input/RT axis 3", controller.getRawAxis(3));
            },
            hookMechanism
        ));
    }

    public Command getAutonomousCommand() {
        return new RunCommand(() -> driveTrain.drive(0.3), driveTrain)
                .withTimeout(2)
                .andThen(() -> driveTrain.stop());
    }
}
