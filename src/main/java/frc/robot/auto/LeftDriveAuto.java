package frc.robot.auto;

import java.io.IOException;
import org.json.simple.parser.ParseException;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;

public class LeftDriveAuto extends SequentialCommandGroup {

    public LeftDriveAuto(CommandSwerveDrivetrain swerve) {

        Command resetPose = new InstantCommand(), LeftDisrupter;
        Pose2d startingPose;

        try {
            LeftDisrupter = swerve.driveAlongPath(PathPlannerPath.fromPathFile("LeftDisrupterPath"));
            if (DriverStation.getAlliance().get().equals(DriverStation.Alliance.Blue)) {
                startingPose = PathPlannerPath.fromPathFile("LeftDisrupterPath").getStartingHolonomicPose().get();
                resetPose = new InstantCommand(() -> swerve.resetPose(startingPose), swerve);
            } else {
                startingPose = PathPlannerPath.fromPathFile("LeftDisrupterPath").flipPath().getStartingHolonomicPose().get();
                resetPose = new InstantCommand(() -> swerve.resetPose(startingPose), swerve);
            }
        } catch (IOException | ParseException e) {
            e.printStackTrace();
            LeftDisrupter = null; // or handle the error appropriately
        }

        addRequirements(swerve);

        addCommands(
            resetPose,
            new ParallelCommandGroup(
                LeftDisrupter
            )
        );

    }

}