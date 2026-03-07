
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Turret;

public class ToggleHood extends Command {
  private Turret turret;
  
  public ToggleHood(Turret turret) {
    this.turret = turret;
    addRequirements(turret);
  }

  @Override
  public void initialize() {
    turret.setHoodPosition(!turret.getHoodPosition());
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
