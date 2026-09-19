package frc.robot.auto;

import java.io.IOException;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ShootCommand;
import frc.robot.constants.Constants;
import frc.robot.subsystems.HubAlignmentPID;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.intake.IntakePivot;
import frc.robot.subsystems.HubAlignmentPID;
import frc.robot.subsystems.autoalignhood.Shootercalculations;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;

public class CenterShootAuto extends SequentialCommandGroup {
     public CenterShootAuto(CommandSwerveDrivetrain swerve, Shooter shooter, Hood hood, Hopper hopper, Indexer indexer, Intake intake, IntakePivot Pivot, Shootercalculations Shootercalc, HubAlignmentPID hubpid) {

        Command resetPose = new InstantCommand(), CenterBackUpShoot;
        Pose2d startingPose;

        try {
            CenterBackUpShoot = swerve.driveAlongPath(PathPlannerPath.fromPathFile("DriveBackPath"));
            if (DriverStation.getAlliance().get().equals(DriverStation.Alliance.Blue)) {
                startingPose = PathPlannerPath.fromPathFile("DriveBackPath").getStartingHolonomicPose().get();
                resetPose = new InstantCommand(() -> swerve.resetPose(startingPose), swerve);
            } else {
                startingPose = PathPlannerPath.fromPathFile("DriveBackPath").flipPath().getStartingHolonomicPose().get();
                resetPose = new InstantCommand(() -> swerve.resetPose(startingPose), swerve);
            }
        } catch (IOException | ParseException e) {
            e.printStackTrace();
            CenterBackUpShoot = null; // or handle the error appropriately
        }
     


        addRequirements(swerve);

        addCommands(
            resetPose,
            new ParallelDeadlineGroup(
                CenterBackUpShoot,
                Pivot.set(0.-2)
            ),

            new ParallelDeadlineGroup(
                Commands.waitSeconds(2.0),
                new ShootCommand(swerve, hubpid, shooter, hood, Shootercalc)
            ),

            new ParallelCommandGroup(
                new ShootCommand(swerve, hubpid, shooter, hood, Shootercalc),
                hopper.set(0.80),
                indexer.set(0.80),
                intake.set(0.60)
            )
    
        );

    }
}
