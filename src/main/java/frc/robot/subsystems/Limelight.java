package frc.robot.subsystems;

import java.util.Arrays;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Limelight extends SubsystemBase {
  public NetworkTable table;

  public double tv = 0;
  public double tx = 0;
  public double ty = 0;
  public double ta = 0;
  public double pipeline;

  public double[] pos = new double[6];
  public double dist = 0;
  public Swerve swerve = new Swerve();

  public double targetHeight;

  public Limelight setTargetHeight(double targetHeight) {
    this.targetHeight = targetHeight;
    return this;
  }

  public Limelight() {
    table = NetworkTableInstance.getDefault().getTable("limelight");
  }

  public void turnLimelightLED(boolean on) {
    table.getEntry("ledMode").setNumber(on ? 3 : 1);
  }

  public void updateLimelightTracking() {
    pos = table.getEntry("botpose").getDoubleArray(new double[6]);
    for (int x = 0; x < pos.length; x++) {
      SmartDashboard.putNumber(String.format("lol %d", x), pos[x]);
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


  @Override
  public void periodic() {
    updateLimelightTracking();
    //System.out.println("hypdist,latdist:" + estimateDistance() + ", " + lateralDistance());
  }
}
