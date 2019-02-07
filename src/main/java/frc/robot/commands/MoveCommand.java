package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;

public class MoveCommand extends Command {

    private double target;
    private double distance;

    public MoveCommand(double target) {
        this.requires(Robot.drivetrain);
        this.target = target;
        this.distance = 0.0;
    }

    @Override
    protected void initialize() {
        Robot.drivetrain.getLeftEncoder().reset();
        Robot.drivetrain.getRightEncoder().reset();

        Robot.drivetrain.arcade(Constants.ALIGN_MOVE_SPEED, 0);
    }

    @Override
    protected void execute() {
        this.distance = (Robot.drivetrain.getLeftEncoder().getDistance() + Robot.drivetrain.getRightEncoder().getDistance()) / 2;
    }

    @Override
    protected boolean isFinished() {
        return this.target - this.distance < Constants.ALIGN_MOVE_TOLERANCE;
    }

    @Override
    protected void end() {
        Robot.drivetrain.arcade(0, 0);
    }

    @Override
    protected void interrupted() {
        this.end();
    }

}