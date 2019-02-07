package frc.robot.commands;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.AutoAlign;
import frc.robot.Constants;
import frc.robot.Robot;

/**
 * This command orients the robot towards the centerpoint.
 */
public class OrientTowardsCenterpointCommand extends Command {

    private PIDController pid;

    private class Source implements PIDSource {

        private PIDSourceType type;

        @Override
        public void setPIDSourceType(PIDSourceType pidSource) {
            this.type = pidSource;
        }

        @Override
        public PIDSourceType getPIDSourceType() {
            return this.type;
        }

        @Override
        public double pidGet() {
            double[][] ps = Robot.camera.getPoints();

            return AutoAlign.getError(ps[0][0], ps[0][1], ps[1][0], ps[1][1]);
        }

    }

    private class Output implements PIDOutput {

        @Override
        public void pidWrite(double output) {
            Robot.drivetrain.arcade(0.0, output * Constants.CENTERPOINT_ROTATION_CONSTANT);
        }

    }

    public OrientTowardsCenterpointCommand() {
        this.pid = new PIDController(
            Constants.CENTERPOINT_P, 
            Constants.CENTERPOINT_I, 
            Constants.CENTERPOINT_D, 
            new Source(), 
            new Output());

        this.pid.disable();
        this.pid.setAbsoluteTolerance(0.2);
        
        this.requires(Robot.camera);
        this.requires(Robot.drivetrain);
    }

    @Override
    protected void initialize() {
        this.pid.setSetpoint(0.0);
        this.pid.enable();
    }
    
    @Override
    protected void execute() {
    }

    @Override
    protected void end() {
        this.pid.setSetpoint(0.0);
        this.pid.disable();
    }

    @Override
    protected boolean isFinished() {
        return false;
    }
}
