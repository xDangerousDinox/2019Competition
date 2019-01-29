package frc.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.AutoAlign;
import frc.robot.Robot;

public class AutoAlignCommand extends CommandGroup {

    public AutoAlignCommand() {
        this.requires(Robot.camera);
        this.addSequential(new OrientTowardsCenterpointCommand());

        AutoAlign align = new AutoAlign(Robot.camera.getPoints(), Robot.camera.getD());

        this.addSequential(new TurnCommand(align.getThetaB()));
        this.addSequential(new MoveCommand(align.getA()));
        this.addSequential(new TurnCommand(90));
        this.addSequential(new MoveCommand(align.getB()));
    }

}