package frc.robot.commands;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;

public class TurnCommand extends Command {

    private PIDController pid;

    public TurnCommand(double angle) {
        this.requires(Robot.drivetrain);
        this.pid = new PIDController(
            Constants.GYRO_TURN_P, 
            Constants.GYRO_TURN_I, 
            Constants.GYRO_TURN_D, 
            new PIDSource(){
                
                private PIDSourceType s;

                @Override
                public void setPIDSourceType(PIDSourceType pidSource) {
                    this.s = pidSource;
                }
            
                @Override
                public double pidGet() {
                    //return Robot.drivetrain.getAngle();
                    return 0;
                    //don't leave this in frankie! fix it!
                }
            
                @Override
                public PIDSourceType getPIDSourceType() {
                    return this.s;
                }
            }, 
            new PIDOutput(){
            
                @Override
                public void pidWrite(double output) {
                    Robot.drivetrain.arcade(0, output * Constants.GYRO_TURN_CONSTANT);
                }
            });

            this.pid.setSetpoint(angle);
            this.pid.setAbsoluteTolerance(0.2);
            
    }

    @Override
    protected void initialize() {
        this.pid.enable();
    }

    @Override
    public boolean isFinished() {
        return this.pid.onTarget();
    }

    @Override
    public void end() {
        this.pid.disable();
    }

    @Override
    public void interrupted() {
        this.end();
    }

}