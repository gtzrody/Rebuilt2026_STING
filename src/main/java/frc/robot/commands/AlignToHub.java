package frc.robot.commands;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import frc.robot.subsystems.HubAlignmentPID;
import frc.robot.subsystems.autoalignhood.Shootercalculations;

public class AlignToHub extends Command {

    private final CommandSwerveDrivetrain swerve;
    private final HubAlignmentPID hubPID;
    private final Shootercalculations shooterCalc;

    private final SwerveRequest.SwerveDriveBrake m_brake = new SwerveRequest.SwerveDriveBrake();

    public AlignToHub(CommandSwerveDrivetrain swerve, HubAlignmentPID hubPID, Shootercalculations shootercalculations) {

        this.swerve = swerve;
        this.hubPID = hubPID;
        this.shooterCalc = shootercalculations;

        addRequirements(swerve, hubPID);
    }

    @Override
    public void initialize() {
        SwerveRequest.FieldCentricFacingAngle fieldCentricRequest = new SwerveRequest.FieldCentricFacingAngle()
            .withVelocityX(0)
            .withVelocityY(0)
            .withTargetDirection(swerve.getState().RawHeading)
            .withCenterOfRotation(new Translation2d(0, 0));
        swerve.setControl(fieldCentricRequest);
    }

    @Override
    public void execute() {
        SmartDashboard.putNumber("Interpolated Hood Angle", shooterCalc.getHoodAngle(swerve.getDistanceToHub()));
        SmartDashboard.putNumber("Interpolated Flywheel RPM", shooterCalc.getFlywheelRPM(swerve.getDistanceToHub()));

        SwerveRequest.FieldCentric fieldCentricRequest = new SwerveRequest.FieldCentric()
            .withVelocityX(0)
            .withVelocityY(0)
            .withRotationalRate(hubPID.getPIDCalculation())
            .withCenterOfRotation(new Translation2d(0, 0));
        swerve.setControl(fieldCentricRequest);
    }

    @Override
    public void end(boolean interrupted) {
        swerve.setControl(m_brake);
    }

    @Override
    public boolean isFinished() {
        return hubPID.getController().atSetpoint();
    }
    
}

