// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface.Measurement;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DXsensor extends SubsystemBase {

  private LaserCan sensor;
  private LinearFilter filter;
  private Measurement last_Measurement; 
  private double value;
  private Timer measurementTimer;
  private static final double MEASUREMENT_INTERVAL = 0.100;
  private static final int MEASUREMENT_LIMIT = 5;
  private int measurementCount = 0;
  
  public DXsensor(int canId) {
    sensor = new LaserCan(canId);
    filter = LinearFilter.movingAverage(MEASUREMENT_LIMIT);
    measurementTimer = new Timer();
    measurementTimer.start();

    try {
      sensor.setRangingMode(LaserCan.RangingMode.LONG);
      sensor.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 16, 16)); //~20deg FOV 
      sensor.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_100MS);
    } catch (Exception e) {
      System.out.println("LaserCan Configuration failed! " + e);
    }

    last_Measurement = sensor.getMeasurement();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (measurementTimer.hasElapsed(MEASUREMENT_INTERVAL)) {
      update();
      measurementTimer.restart();
    }
  }

  private void update(){
    last_Measurement = sensor.getMeasurement();
    if(last_Measurement != null && last_Measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
      double newValue =  last_Measurement.distance_mm * 0.001;
      value = filter.calculate(newValue);
      if (measurementCount < MEASUREMENT_LIMIT) { measurementCount++; }
    }else{
      if (measurementCount > 0) { measurementCount--; }
    }
  }

  public boolean isValid(){
    return measurementCount > (MEASUREMENT_LIMIT / 2);
  }

  public double getDistanceMeters() {
    return value;
  }

  @Override
  public void initSendable(SendableBuilder builder){
    super.initSendable(builder);
    builder.addStringProperty("Distance (m)", () -> isValid() ? String.format("%3.2f m", value) : "---- m", null);
    builder.addDoubleProperty("distance", ()-> isValid() ? value : Double.NaN, null);
  }

}
