package frc.robot.auto;

import java.io.IOException;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.commands.ShootCommand;
import frc.robot.subsystems.HubAlignmentPID;
import frc.robot.subsystems.autoalignhood.Shootercalculations;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;

public class PathAndShootAuto extends SequentialCommandGroup {

    public PathAndShootAuto(
            CommandSwerveDrivetrain drivetrain,
            HubAlignmentPID hubPID,
            Shooter shooter,
            Hood hood,
            Indexer indexer,
            Hopper hopper,
            Shootercalculations shooterCalc,
            String pathName) {

        try {

            PathPlannerPath path =
                    PathPlannerPath.fromPathFile(pathName);


            Pose2d startingPose;

            if (DriverStation.getAlliance().orElse(Alliance.Blue)
                    == Alliance.Red) {

                startingPose =
                        path.flipPath()
                            .getStartingHolonomicPose()
                            .get();

            } else {

                startingPose =
                        path.getStartingHolonomicPose()
                            .get();
            }


            addCommands(

          
                Commands.runOnce(
                    () -> drivetrain.resetPose(startingPose),
                    drivetrain
                ),



                drivetrain.driveAlongPath(path),


       
                Commands.waitSeconds(0.20),



                new ShootCommand(
                    drivetrain,
                    hubPID,
                    shooter,
                    hood,
                    indexer,
                    hopper,
                    shooterCalc
                ).withTimeout(4.0)
            );

        } catch (IOException | ParseException e) {

            DriverStation.reportError(
                "Unable to load autonomous path: " + pathName,
                e.getStackTrace()
            );
        }
    }
}
