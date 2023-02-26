package frc.robot.commands.autonomous;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveDrivetrain;

public class DoubleChargeSpin {
    private static final PathPlannerTrajectory traj = PathPlanner.loadPath("Double Charge with spin", AutoUtils.getDefaultConstraints());
    private static final PathPlannerTrajectory testTraj = PathPlanner.loadPath("Test For Heading", AutoUtils.getDefaultConstraints());

    public static Command getPath(SwerveDrivetrain drive) {
        return AutoUtils.getAutoRoutine(traj, drive);
    }

    public static  Command getTestPath(SwerveDrivetrain drive) {
        return AutoUtils.getAutoRoutine(testTraj, drive);
    }
}
