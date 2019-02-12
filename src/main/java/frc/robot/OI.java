package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.robot.commands.AutoAlignCommand;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
    public final Joystick driveStick;
    public final Joystick operatorStick;
    private Button driveTrigger;
    private Button driveSideButton;
    private Button autoAlignButton;

    private Button operatorTrigger;//Eject
    private Button operatorSideButton;//Suck

    public static final int AUTOALIGNBUTTON = 3;
    
    public OI() {
        this.driveStick = new Joystick(0);
        this.operatorStick = new Joystick(1);
        this.driveTrigger = new JoystickButton(driveStick, 1);
        this.driveSideButton = new JoystickButton(driveStick, 2);

        this.autoAlignButton = new JoystickButton(driveStick, OI.AUTOALIGNBUTTON);
        this.autoAlignButton.whenPressed(new AutoAlignCommand());

        this.operatorTrigger = new JoystickButton(operatorStick, 1);
        this.operatorSideButton = new JoystickButton(operatorStick, 2);
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
