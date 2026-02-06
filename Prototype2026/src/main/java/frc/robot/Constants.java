package frc.robot;

public class Constants
{
  public static final String kCompSN               = "00000000";
  public static final String kPracticeSN           = "99999999";

  // Phoenix firmware versions expected
  public static final int    kPhoenix5MajorVersion = ((22 * 256) + 0);
  public static final int    kPhoenix6MajorVersion = 26;

  public static final class INConsts
  {
    /** Intake roller modes */
    public enum RollerMode
    {
      STOP,         // Stop all rotation
      ACQUIRE,      // Speed for acquiring a game piece
      EXPEL,        // Speed for expelling a game piece
      HOLD          // Maintain existing speed setting
    }
  }

}
