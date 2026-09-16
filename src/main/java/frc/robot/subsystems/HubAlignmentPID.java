package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.MathUtils;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;


public class HubAlignmentPID extends SubsystemBase {

    public static boolean manualMode = false; // not even remotely related to this but it exists here

    public PIDController controller;
    private CommandSwerveDrivetrain swerve;
    private static final double MAX_ROTATION_SPEED = 2 * Math.PI;

    private double targetAngle, rotation;

    public HubAlignmentPID(CommandSwerveDrivetrain swerve) {
        this.swerve = swerve;
        controller = new PIDController(
            Constants.SwerveConstants.HUB_ROTATION_PID_KP,
            Constants.SwerveConstants.HUB_ROTATION_PID_KI,
            Constants.SwerveConstants.HUB_ROTATION_PID_KD
        ); // Tuned PID gains
        controller.setTolerance(0.05); // Tolerance for stopping rotation
        controller.setIntegratorRange(-Math.PI, Math.PI);
        controller.enableContinuousInput(-Math.PI, Math.PI);
    }

    public PIDController getController() {
        return controller;
    }

    public void addSetpoint(double setpoint) {
        controller.setSetpoint(setpoint + Math.PI);
    }

    public double getPIDCalculation() {
        return rotation;
    }

    public boolean getManualMode() {
        return manualMode;
    }

    public void setManualMode(boolean mode) {
        manualMode = mode;
    }

    @Override
    public void periodic() {

         Translation2d target = DriverStation.getAlliance().get().compareTo(Alliance.Blue) == 0 ? Constants.AutoDriveConstants.BLUE_HUB_CENTER : Constants.AutoDriveConstants.RED_HUB_CENTER;

         double deltaX = target.getX() - swerve.getState().Pose.getX();
         double deltaY = target.getY() - swerve.getState().Pose.getY();

        // // Compute absolute angle to the target
         targetAngle = Math.atan2(deltaY, deltaX) + Math.PI;

        // targetAngle = MathUtils.findClosestTarget(
        //         swerve.getState().Pose, 
        //         (DriverStation.getAlliance().get().compareTo(DriverStation.Alliance.Blue) == 0) ?
        //         Constants.AutoDriveConstants.BLUE_HUB_POSES : Constants.AutoDriveConstants.RED_HUB_POSES).getRotation().getRadians();

        controller.setSetpoint(targetAngle);

        rotation = controller.calculate(swerve.getState().Pose.getRotation().getRadians());
        rotation = Math.max(-MAX_ROTATION_SPEED, Math.min(MAX_ROTATION_SPEED, rotation));

        SmartDashboard.putNumber("Rotational Hub Speed", rotation);
        SmartDashboard.putNumber("Swerve Angle", swerve.getState().Pose.getRotation().getRadians());
        SmartDashboard.putNumber("Target Angle", targetAngle);
    }
}
