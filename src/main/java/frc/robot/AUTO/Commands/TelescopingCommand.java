package frc.robot.AUTO.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.MECHANISMS.ARM.Arm;
public class TelescopingCommand extends CommandBase {
    
    private double angle;

    public TelescopingCommand(double angle) {
      this.angle = angle;
      // Use addRequirements() here to declare subsystem dependencies.
    }
  
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
      Arm.getInstance().pidTelescope(angle);
    }
  
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return false;
    }
  }
  