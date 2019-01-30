/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.RobotMap;
import frc.robot.commands.JoystickDrive;

/**
 * Add your docs here.
 */
public class Drivetrain extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  // Left motor controllers
  private Talon frontLeft = new Talon(RobotMap.FRONT_LEFT_TALON);
  private Talon backLeft = new Talon(RobotMap.BACK_LEFT_TALON);
  private SpeedControllerGroup left = new SpeedControllerGroup(frontLeft, backLeft);

  // Right motor controllers
  private Talon frontRight = new Talon(RobotMap.FRONT_RIGHT_TALON);
  private Talon backRight = new Talon(RobotMap.BACK_RIGHT_TALON);
  private SpeedControllerGroup right = new SpeedControllerGroup(frontRight, backRight);

  // Drive controller
  private DifferentialDrive drive = new DifferentialDrive(left, right);

  // Drive-related sensors
  private Encoder leftEncoder = new Encoder(0, 1, false, Encoder.EncodingType.k4X);
  private Encoder rightEncoder = new Encoder(2, 3, false, Encoder.EncodingType.k4X);

  private ADXRS450_Gyro gyro = new ADXRS450_Gyro();

  public Drivetrain() {
    // Set up encoders
		leftEncoder.setMaxPeriod(.1);
		leftEncoder.setMinRate(10);
		leftEncoder.setDistancePerPulse(0.0254 * 6 * Math.PI / 360);
		leftEncoder.setReverseDirection(true);
		leftEncoder.setSamplesToAverage(7);
		rightEncoder.setMaxPeriod(.1);
		rightEncoder.setMinRate(10);
		rightEncoder.setDistancePerPulse(0.0254 * 6 * Math.PI / 250);
		rightEncoder.setReverseDirection(false);
    rightEncoder.setSamplesToAverage(7);
    
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

  public Encoder getLeftEncoder() {
    return leftEncoder;
  }

  public Encoder getRightEncoder() {
    return rightEncoder;
  }

  public ADXRS450_Gyro getGyro() {
    return gyro;
  }

}
