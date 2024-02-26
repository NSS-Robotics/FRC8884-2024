package frc.robot.subsystems;

import au.grapplerobotics.LaserCan;
import au.grapplerobotics.ConfigurationFailedException;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class LaserCAN extends SubsystemBase {
    LaserCan lasercan = new LaserCan(Constants.LaserCanConstants.laserCan);

    public LaserCAN() {
        try {
            lasercan.setRangingMode(LaserCan.RangingMode.SHORT);
            lasercan.setRegionOfInterest(new LaserCan.RegionOfInterest(0, 0, 0, 0));
            lasercan.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
        } catch (ConfigurationFailedException e) {
            System.out.println("LASERCAN CONFIG FAILED");
        }
    }

    @Override
    public void periodic() {
        LaserCan.Measurement m = lasercan.getMeasurement();
        SmartDashboard.putNumber("LaserCan MM", m.distance_mm);
    }

}
