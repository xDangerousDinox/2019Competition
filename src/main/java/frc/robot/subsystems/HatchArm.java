/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class HatchArm extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private CANSparkMax hatchArm = new CANSparkMax(RobotMap.ARM_HATCH, MotorType.kBrushed);
  private Encoder hatchEncoder = new Encoder(RobotMap.HATCH_ENCODER_A, RobotMap.HATCH_ENCODER_B);

  private DoubleSolenoid hatchRelease = new DoubleSolenoid(RobotMap.HATCH_SOLENDOID_CHANNEL_IN, RobotMap.HATCH_SOLENOID_CHANNEL_OUT);

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public void setHatchArm(double speed) {
    hatchArm.set(speed);
  }

  //TODO get actual angle
  public double getAngle() {
    return hatchEncoder.getDistance();
  }

  public void forward() {
    hatchRelease.set(DoubleSolenoid.Value.kForward);
  }

  public void reverse() {
    hatchRelease.set(DoubleSolenoid.Value.kReverse);
  }

  public void stop() {
    hatchRelease.set(DoubleSolenoid.Value.kOff);
  }
}
