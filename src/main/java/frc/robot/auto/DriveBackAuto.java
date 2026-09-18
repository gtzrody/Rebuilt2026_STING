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

public class DriveBackAuto extends SequentialCommandGroup {

    public DriveBackAuto(CommandSwerveDrivetrain swerve) {

        Command resetPose = new InstantCommand(), DriveBack;
        Pose2d startingPose;

        try {
            DriveBack = swerve.driveAlongPath(PathPlannerPath.fromPathFile("DriveBackPath"));
            if (DriverStation.getAlliance().get().equals(DriverStation.Alliance.Blue)) {
                startingPose = PathPlannerPath.fromPathFile("DriveBackPath").getStartingHolonomicPose().get();
                resetPose = new InstantCommand(() -> swerve.resetPose(startingPose), swerve);
            } else {
                startingPose = PathPlannerPath.fromPathFile("DriveBackPath").flipPath().getStartingHolonomicPose().get();
                resetPose = new InstantCommand(() -> swerve.resetPose(startingPose), swerve);
            }
        } catch (IOException | ParseException e) {
            e.printStackTrace();
            DriveBack = null; // or handle the error appropriately
        }

        addRequirements(swerve);

        addCommands(
            resetPose,
            new ParallelCommandGroup(
                DriveBack
            )
        );

    }

}