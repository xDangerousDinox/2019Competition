/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.command.*;
import edu.wpi.first.wpilibj.drive.*;
import frc.robot.*;
import frc.robot.commands.*;

/**
 * Add your docs here.
 */
public class Drivetrain extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  // Left motor controllers
  private CANSparkMax left1 = new CANSparkMax(RobotMap.LEFT_DRIVE_1, MotorType.kBrushless);
  private CANSparkMax left2 = new CANSparkMax(RobotMap.LEFT_DRIVE_2, MotorType.kBrushless);
  private CANSparkMax left3 = new CANSparkMax(RobotMap.LEFT_DRIVE_3, MotorType.kBrushless);

  private SpeedControllerGroup leftMotors = new SpeedControllerGroup(left1, left2, left3);

  // Right motor controllers
  private CANSparkMax right1 = new CANSparkMax(RobotMap.RIGHT_DRIVE_1, MotorType.kBrushless);
  private CANSparkMax right2 = new CANSparkMax(RobotMap.RIGHT_DRIVE_2, MotorType.kBrushless);
  private CANSparkMax right3 = new CANSparkMax(RobotMap.RIGHT_DRIVE_3, MotorType.kBrushless);

  private SpeedControllerGroup rightMotors = new SpeedControllerGroup(right1, right2, right3);

  // Drive controller
  private DifferentialDrive drive = new DifferentialDrive(leftMotors, rightMotors);

  private ADXRS450_Gyro gyro = new ADXRS450_Gyro();

  public Drivetrain() {
    // Set up gyro
    gyro.calibrate();

    // Enable drivetrain
    drive.setSafetyEnabled(false);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new JoystickDrive());
  }

  public void tank(double left, double right) {
    drive.tankDrive(left, right);
  }

  public void arcade(double xSpeed, double zRotation) {
    // System.out.println("This is left encoder value:" + leftEncoder.get());
    // System.out.println("This is right encoder value:" + rightEncoder.get());
    //System.out.println("This is total distance travelled (left):" + leftEncoder.getDistance());
    //System.out.println("This is total distance travelled (right):" + rightEncoder.getDistance());

    //System.out.println("Gyro: " + gyro.getAngle());
    drive.arcadeDrive(xSpeed, zRotation);
  }

  public ADXRS450_Gyro getGyro() {
    return gyro;
  }

  public double getAngle() {
    return this.gyro.getAngle();
  }

}
