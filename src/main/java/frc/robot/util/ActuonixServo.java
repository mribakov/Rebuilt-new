package frc.robot.util;

public class ActuonixServo extends LinearServo
{
    public ActuonixServo(int channel)
    {
        super(channel, 1000, 2000);
    }
}
