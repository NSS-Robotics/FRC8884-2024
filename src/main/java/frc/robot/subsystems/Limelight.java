package frc.robot.subsystems;

import java.util.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
public class Limelight extends SubsystemBase {

    public NetworkTable table;

    public double tv = 0;
    public double tx = 0;
    public double ty = 0;
    public double ta = 0;
    public double pipeline;

    public String name;

    public Pose2d botPose = new Pose2d(0, 0, new Rotation2d());

    public Limelight(String name) {
        this.name = "limelight-" + name;
        table = NetworkTableInstance.getDefault().getTable(this.name);
    }

    public void turnLimelightLED(boolean on) {
        table.getEntry("ledMode").setNumber(on ? 3 : 1);
    }

    public void updateLimelightTracking() {
        tv = table.getEntry("tv").getDouble(0);
        tx = table.getEntry("tx").getDouble(0);
        ty = table.getEntry("ty").getDouble(0);
        ta = table.getEntry("ta").getDouble(0);

        double[] pos = table
            .getEntry("botpose_wpiblue")
            .getDoubleArray(new double[6]);

        if (pos.length < 6) {
            return;
        }

        double rz = pos[5] + 180;
        if (rz > 180) {
            while (rz > 180) {
                rz -= 360;
            }
        } else if (rz < -180) {
            while (rz < -180) {
                rz += 360;
            }
        }
        botPose =
            new Pose2d(pos[0], pos[1], new Rotation2d(Math.toRadians(rz)));

        String[] names = {
            "pos x",
            "pos y",
            "pos z",
            "rot x",
            "rot y",
            "rot z",
        };
        for (int i = 0; i < names.length; i++) {
            SmartDashboard.putNumber(names[i], pos[i]);
        }
    }

    public double getPipeline() {
        pipeline = table.getEntry("getpipe").getDouble(0);
        return pipeline;
    }

    public void setPipeline(double pipeline) {
        table.getEntry("pipeline").setNumber(pipeline);
        SmartDashboard.putNumber("Pipeline", pipeline);
    }

    public double gettx() {
        updateLimelightTracking();
        return tx;
    }

    @Override
    public void periodic() {
        updateLimelightTracking();
    }
}
