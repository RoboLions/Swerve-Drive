package frc.robot.autos;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;

public class Trajectories {
    public static Trajectory getToTSecondBall() {
        String trajectoryJSON = "output/Unnamed.wpilib.json";
        Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
        Trajectory toTSecondBall = new Trajectory();
        try {
            toTSecondBall = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
        }
        return toTSecondBall;
    }

    public static class terminalTwoBall {
        public static final Trajectory toSecondBall = getToTSecondBall();
    }
}
