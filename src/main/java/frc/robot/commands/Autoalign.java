package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.AprilTagRegion;
import java.util.ArrayList;
import java.util.Arrays;

public class Autoalign {
    private final Drive drive;

    public static ArrayList<Pose2d> blueReefTagPoses = new ArrayList<>();
    public static ArrayList<Pose2d> redReefTagPoses = new ArrayList<>();
    public static ArrayList<Pose2d> allReefTagPoses = new ArrayList<>();
    public static AprilTagFieldLayout field = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    public Autoalign(Drive drive) {
        this.drive = drive;
        Arrays.stream(AprilTagRegion.kReef.blue()).forEach((i) -> {
            field.getTagPose(i).ifPresent((p) -> {
                blueReefTagPoses.add(new Pose2d(
                        p.getMeasureX(), p.getMeasureY(), p.getRotation().toRotation2d()));
            });
        });

        Arrays.stream(AprilTagRegion.kReef.red()).forEach((i) -> {
            field.getTagPose(i).ifPresent((p) -> {
                redReefTagPoses.add(new Pose2d(
                        p.getMeasureX(), p.getMeasureY(), p.getRotation().toRotation2d()));
            });
        });

        Arrays.stream(AprilTagRegion.kReef.both()).forEach((i) -> {
            field.getTagPose(i).ifPresent((p) -> {
                allReefTagPoses.add(new Pose2d(
                        p.getMeasureX(), p.getMeasureY(), p.getRotation().toRotation2d()));
            });
        });
    }

    public Pose2d getClosestReefAprilTag() {
        var alliance = DriverStation.getAlliance();

        ArrayList<Pose2d> reefPoseList;
        if (alliance.isEmpty()) {
            reefPoseList = allReefTagPoses;
        } else {
            reefPoseList = alliance.get() == Alliance.Blue ? blueReefTagPoses : redReefTagPoses;
        }

        return drive.getPose().nearest(reefPoseList);
    }

    public Command generate() {
        return Commands.runOnce(() -> {
            Pose2d tagPose = getClosestReefAprilTag();
            Translation2d offset = new Translation2d(
                    0.7 * Math.cos(tagPose.getRotation().getRadians()),
                    0.7 * Math.sin(tagPose.getRotation().getRadians()));
            Pose2d targetPose = new Pose2d(tagPose.getTranslation().plus(offset), tagPose.getRotation());
            AutoBuilder.pathfindToPose(
                            targetPose, new PathConstraints(10000.0, 10000.0, 100000.0 * Math.PI, 100000.0 * Math.PI))
                    .schedule();
        });
    }
}
