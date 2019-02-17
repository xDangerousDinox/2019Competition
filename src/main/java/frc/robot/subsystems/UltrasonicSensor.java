/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Add your docs here.
 */
public class UltrasonicSensor extends Subsystem {
 
  private AnalogInput ultrasonicSensor = new AnalogInput(0);

//int raw = ultrasonicSensor.getValue();
//int averageRaw = ultrasonicSensor.getAverageValue();
//double averageVolts = ultrasonicSensor.getAverageVoltage();



public double getDistance(){
 double volts = ultrasonicSensor.getVoltage();
 double distance = ((5.0/1024.0)*5.0)/volts;
 return distance;
}

public double getVoltage() {
  return ultrasonicSensor.getVoltage();
}


  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
