package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.function.Supplier;

public class Autoalign {
    public Command generate(Supplier<Pose2d> targetPoseSupplier, Supplier<Pose2d> robotPoseSupplier) {
        // return Commands.run(() -> {
        //     Pose2d targetPose = targetPoseSupplier.get();
        //     Pose2d robotPose = robotPoseSupplier.get();

        //     List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(robotPose, targetPose);
        //     PathPlannerPath path = new PathPlannerPath(
        //             waypoints,
        //             new PathConstraints(10000.0, 10000.0, 100000.0 * Math.PI, 100000.0 * Math.PI),
        //             new IdealStartingState(0, robotPose.getRotation()),
        //             new GoalEndState(0.0, targetPose.getRotation()));
        //     path.preventFlipping = true;
        //     AutoBuilder.followPath(path).schedule();
        // });
        return Commands.run(() -> {
            Pose2d targetPose = targetPoseSupplier.get();

            AutoBuilder.pathfindToPose(
                            targetPose, new PathConstraints(10000.0, 10000.0, 100000.0 * Math.PI, 100000.0 * Math.PI))
                    .schedule();
        });
    }
}
