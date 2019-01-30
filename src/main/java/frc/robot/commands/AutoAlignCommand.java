package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.AutoAlign;
import frc.robot.Robot;

public class AutoAlignCommand extends CommandGroup {

    private class Aligner extends Command {
        
        private AutoAlignCommand c;

        public Aligner(AutoAlignCommand c) {
            this.c = c;
        }

        @Override
        protected void initialize() {
            c.align = new AutoAlign(Robot.camera.getPoints(), Robot.camera.getD());
        }

        @Override
        protected boolean isFinished() {
            return true;
        }
        
    }

    private AutoAlign align;

    public AutoAlignCommand() {
        this.requires(Robot.camera);
        this.addSequential(new OrientTowardsCenterpointCommand());
        this.addSequential(new Aligner(this));
        this.addSequential(new TurnCommand(this.align.getThetaB()));
        this.addSequential(new MoveCommand(this.align.getA()));
        this.addSequential(new TurnCommand(-90));
        this.addSequential(new MoveCommand(this.align.getB()));
    }

}