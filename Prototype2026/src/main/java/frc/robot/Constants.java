package frc.robot;

public class Constants
{

  public static final class INConsts
  {
    /** Intake roller modes */
    public enum INRollerMode
    {
      STOP,    // Stop all rotation
      ACQUIRE, // Speed for acquiring a game piece
      EXPEL,   // Speed for expelling a game piece
      SHOOT,   // Speed for putting game piece into shooter 
      HANDOFF, // Speed for putting game piece into feeder 
      HOLD     // Maintain existing speed setting
    }
  }

}
