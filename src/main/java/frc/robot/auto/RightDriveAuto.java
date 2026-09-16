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

public class RightDriveAuto extends SequentialCommandGroup {

    public RightDriveAuto(CommandSwerveDrivetrain swerve) {

        Command resetPose = new InstantCommand(), RightDisrupter;
        Pose2d startingPose;

        try {
            RightDisrupter = swerve.driveAlongPath(PathPlannerPath.fromPathFile("RightDisrupterPath"));
            if (DriverStation.getAlliance().get().equals(DriverStation.Alliance.Blue)) {
                startingPose = PathPlannerPath.fromPathFile("RightDisrupterPath").getStartingHolonomicPose().get();
                resetPose = new InstantCommand(() -> swerve.resetPose(startingPose), swerve);
            } else {
                startingPose = PathPlannerPath.fromPathFile("RightDisrupterPath").flipPath().getStartingHolonomicPose().get();
                resetPose = new InstantCommand(() -> swerve.resetPose(startingPose), swerve);
            }
        } catch (IOException | ParseException e) {
            e.printStackTrace();
            RightDisrupter = null; // or handle the error appropriately
        }

        addRequirements(swerve);

        addCommands(
            resetPose,
            RightDisrupter
        );

    }

}