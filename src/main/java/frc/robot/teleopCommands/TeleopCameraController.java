/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.teleopCommands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class TeleopCameraController extends Command {

  private boolean isForward;
  public TeleopCameraController() {
    // Use requires() here to declare subsystem dependencies
    requires(Robot.cameraController);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    isForward = true;
    Robot.cameraController.setX(0.5);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if (Robot.oi.driveStick.getRawButtonReleased(11)) {
      if (isForward == true) {
        isForward = false;
      } else {
        isForward = true;
      }
    }
    if (isForward == true) {
      Robot.cameraController.setZ(1);
    } else {
      Robot.cameraController.setZ(0);
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
