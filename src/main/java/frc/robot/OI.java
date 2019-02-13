package frc.robot;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.command.*;
import edu.wpi.first.wpilibj.buttons.*;

import frc.robot.subsystems.*;
import frc.robot.*;
import frc.robot.commands.*;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
    public final Joystick driveStick;
    public final Joystick operatorStick;
    private Button autoAlignButton;

    public static final int AUTOALIGNBUTTON = 3;
    
    public OI() {
        this.driveStick = new Joystick(0);
        this.operatorStick = new Joystick(1);

        this.autoAlignButton = new JoystickButton(driveStick, OI.AUTOALIGNBUTTON);
        this.autoAlignButton.whenPressed(new AutoAlignCommand());
      }

      // Drive Stick
    public double getDriveX() {
        return this.driveStick.getX();
    }

    public double getDriveY() {
        return this.driveStick.getY();
    }

    public double getDriveZ() {
        return this.driveStick.getZ();
    }
    public double getDriveThrottle() {
        return this.driveStick.getThrottle();
    }
    public boolean isDriveButtonDown(int buttonNumber) {
        return this.driveStick.getRawButton(buttonNumber);
    }
    
    public double getOperatorX() {
        return this.operatorStick.getX();
    }
    public double getOperatorY() {
        return this.operatorStick.getY();
    }
    public double getOperatorZ() {
        return this.operatorStick.getZ();
    }
    public double getOperatorThrottle() {
        return this.operatorStick.getThrottle();
    }
    public boolean isOperatorButtonDown(int buttonNumber) {
        return this.operatorStick.getRawButton(buttonNumber);
    }
    public int getOperatorPOV() {
        return this.operatorStick.getPOV();
    }
}
