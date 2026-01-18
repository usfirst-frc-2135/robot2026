package frc.robot;

public class Constants
{
  public static final String kCompSN     = "00000000";
  public static final String kPracticeSN = "99999999";

  public static final class INConsts
  {
    /** Intake roller modes */
    public enum RollerMode
    {
      STOP,    // Stop all rotation
      FUELACQUIRE, // Speed for acquiring a game piece
      FUELEXPEL,   // Speed for expelling a game piece
      SHOOT,   // Speed for putting game piece into shooter 
      HANDOFF, // Speed for putting game piece into feeder 
      FUELHOLD    // Maintain existing speed setting
    }
  }

}
