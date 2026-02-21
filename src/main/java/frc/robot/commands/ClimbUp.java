
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Climber;

public class ClimbUp extends Command {
  private Climber climb;

  public ClimbUp(Climber climber) {
    climb = climber;
    addRequirements(climb);
  }

  @Override
  public void initialize() {
    climb.setPosition(Constants.Climber.lvl1Position);
  }

  @Override
  public boolean isFinished() {
    return Math.abs(climb.getPosition() - Constants.Climber.lvl1Position) < Constants.Climber.climbThreshold;
  }
}
