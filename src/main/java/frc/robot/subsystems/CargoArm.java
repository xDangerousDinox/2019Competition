/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class CargoArm extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private CANSparkMax cargoArm = new CANSparkMax(RobotMap.ARM_CARGO, MotorType.kBrushed);
  private CANSparkMax shoot = new CANSparkMax(RobotMap.SHOOT, MotorType.kBrushed);

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public void setCargoArm(double speed) {
    cargoArm.set(speed);
  }

  public void shootOut() {
    shoot.set(0.7);
  }

  public void stopShoot() {
    shoot.set(0);
  }

  public void shootIn() {
    shoot.set(-0.7);
  }
}
