package frc.robot.commands;


import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.HubAlignmentPID;
import frc.robot.subsystems.autoalignhood.Shootercalculations;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;


public class ShootCommand extends SequentialCommandGroup {

private static final double SHOOTER_RPM = 2200.0;

private static final double RPM_TOLERANCE = 100.0;
private static final double ANGLE_TOLERANCE = 0.001;

private final SwerveRequest.FieldCentric m_fieldCentric  = new SwerveRequest.FieldCentric();
private final SwerveRequest.SwerveDriveBrake m_brake     = new SwerveRequest.SwerveDriveBrake();

public ShootCommand(
        CommandSwerveDrivetrain drivetrain,
        HubAlignmentPID         hubPID,
        Shooter                 shooter,
        Hood                    hood,
        Indexer                 indexer,
        Hopper                  hopper,
        Shootercalculations     shooterCalc) {


      DoubleSupplier distanceSupplier  = () -> drivetrain.getDistanceToHub();
      DoubleSupplier hoodAngleSupplier = () -> shooterCalc.getHoodAngle(distanceSupplier.getAsDouble());
     DoubleSupplier flywheelRPMSupplier = () -> shooterCalc.getFlywheelRPM(distanceSupplier.getAsDouble());

    addCommands(


       new ParallelCommandGroup(
 
                new AlignToHub(drivetrain, hubPID, shooterCalc),
 
            new FunctionalCommand(
                () -> shooter.setVelocitySetpoint(RPM.of(flywheelRPMSupplier.getAsDouble())),
                () -> {},
                interrupted -> {},
                () -> (Math.abs(shooter.getVelocity().in(RPM) - flywheelRPMSupplier.getAsDouble()) < RPM_TOLERANCE),
                shooter
            ),
            new FunctionalCommand(
                () -> hood.setAngleAndStop(Degrees.of(hoodAngleSupplier.getAsDouble()), Degrees.of(ANGLE_TOLERANCE)),
                () -> {},
                interrupted -> {},
                () -> false,
                hood
            )
        ),
 
            new ParallelCommandGroup(
 
                drivetrain.applyRequest(() -> m_brake),
 
                new FunctionalCommand(
                    () -> shooter.setVelocitySetpoint(RPM.of(flywheelRPMSupplier.getAsDouble())),
                    () -> shooter.setVelocitySetpoint(RPM.of(flywheelRPMSupplier.getAsDouble())),
                    interrupted -> {},
                     () -> false,
                    shooter
                ),
 
                new FunctionalCommand(
                    () -> hood.setAngleSetpoint(Degrees.of(hoodAngleSupplier.getAsDouble())),
                    () -> hood.setAngleSetpoint(Degrees.of(hoodAngleSupplier.getAsDouble())),
                    interrupted -> {},
                    () -> false,
                    hood
                ),
 
                indexer.set(0.60),
                hopper.set(0.80)
            )
        );
    }
}