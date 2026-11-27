//
// Phoenix 6 initialization utilities
//
package first.robot.lib.phoenix;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;

import org.wpilib.system.DataLogManager;
import org.wpilib.system.Timer;

/****************************************************************************
 * 
 * Phoenix 6 initialization class
 */
public class PhoenixUtil
{
  public static final int     kMajorVersion = 26;       // Major version of Phoenix 6 firmware expected
  private static final int    kRetries      = 5;        // Number of version check attempts

  private static final String kClassName    = "PhoenixUtil6";
  private static final String kName         = "PhoenixV6";
  private static final double kCANTimeout   = 0.250;

  private static PhoenixUtil  m_instance    = null;

  /****************************************************************************
   * 
   * Constructor
   */
  PhoenixUtil( )
  {}

  /****************************************************************************
   * 
   * Retrieve instance of this class (singleton)
   * 
   * @return reference to this (singleton) class
   */
  public static PhoenixUtil getInstance( )
  {
    if (m_instance == null)
      m_instance = new PhoenixUtil( );

    return m_instance;
  }

  /****************************************************************************
   * 
   * Initialize a Talon FX motor and display the result
   * 
   * @param talonFX
   *          reference to an FX controlled motor
   * @param name
   *          descriptive name of the FX controlled motor
   * @param config
   *          the talon FX configuration to be programmed
   * @return true if successfully initialized
   */
  public boolean talonFXInitialize(TalonFX talonFX, String name, TalonFXConfiguration config)
  {
    StatusCode status = StatusCode.StatusCodeNotInitialized;
    int devID = 0;
    int fwvMajor = 0;
    int fwvMinor = 0;
    int fwvBugfix = 0;
    int fwvBuild = 0;
    boolean talonValid;

    // Display Talon firmware versions
    devID = talonFX.getDeviceID( );

    // This can take multiple attempts before ready
    for (int i = 0; i < kRetries && fwvMajor == 0; i++)
    {
      StatusSignal<Integer> statusSignal = talonFX.getVersion( ).waitForUpdate(0.250);
      status = statusSignal.getStatus( );
      if (status.isOK( ))
      {
        fwvMajor = (statusSignal.getValue( ) >> 24) & 0xff;
        fwvMinor = (statusSignal.getValue( ) >> 16) & 0xff;
        fwvBugfix = (statusSignal.getValue( ) >> 8) & 0xff;
        fwvBuild = (statusSignal.getValue( ) >> 0) & 0xff;
      }
      else
      {
        Timer.delay(0.100);
      }
    }

    // Assume true if major version is correct, otherwise false
    talonValid = (fwvMajor >= kMajorVersion);

    // If a configuration is provided, apply it to the TalonFX
    if (config != null)
    {
      if ((status = talonFX.getConfigurator( ).apply(config, kCANTimeout)) != StatusCode.OK)
      {
        DataLogManager.log(String.format("%s: FX ID %2d %15s (%12s) getConfigurator: '%s'", kName, devID, name,
            talonFX.getDescription( ), status.getDescription( )));
        talonValid = false;
      }

      if ((status = talonFX.setControl(new VoltageOut(0))) != StatusCode.OK)
      {
        DataLogManager.log(String.format("%s: FX ID %2d %15s (%12s) setControl: '%s'", kName, devID, name,
            talonFX.getDescription( ), status.getDescription( )));
        talonValid = false;
      }

      // Configure sensor settings
      if ((status = talonFX.setPosition(0.0)) != StatusCode.OK)
      {
        DataLogManager.log(String.format("%s: FX ID %2d %15s (%12s) setPosition: '%s'", kName, devID, name,
            talonFX.getDescription( ), status.getDescription( )));
        talonValid = false;
      }

      talonFX.setSafetyEnabled(false);
    }

    DataLogManager.log(
        String.format("%s: FX ID %2d %15s (%12s) ver: %d.%d.%d.%d is '%s' - %s", kName, devID, name, talonFX.getDescription( ),
            fwvMajor, fwvMinor, fwvBugfix, fwvBuild, status.getDescription( ), (talonValid) ? "VALID" : "error: UNRESPONSIVE!"));

    return talonValid;
  }

  /****************************************************************************
   * 
   * Initialize a CANcoder and display the result
   * 
   * @param canCoder
   *          reference to a CANcoder
   * @param name
   *          descriptive name of the CANcoder
   * @param config
   *          the CANcoder configuration to be programmed
   * @return true if successfully initialized
   */
  public boolean canCoderInitialize(CANcoder canCoder, String name, CANcoderConfiguration config)
  {
    StatusCode status = StatusCode.StatusCodeNotInitialized;
    int devID = 0;
    int fwvMajor = 0;
    int fwvMinor = 0;
    int fwvBugfix = 0;
    int fwvBuild = 0;
    boolean canCoderValid;

    // Display Talon firmware versions
    devID = canCoder.getDeviceID( );

    // This can take multiple attempts before ready
    for (int i = 0; i < kRetries && fwvMajor == 0; i++)
    {
      StatusSignal<Integer> statusSignal = canCoder.getVersion( );
      status = statusSignal.getStatus( );
      if (status.isOK( ))
      {
        fwvMajor = (statusSignal.getValue( ) >> 24) & 0xff;
        fwvMinor = (statusSignal.getValue( ) >> 16) & 0xff;
        fwvBugfix = (statusSignal.getValue( ) >> 8) & 0xff;
        fwvBuild = (statusSignal.getValue( ) >> 0) & 0xff;
      }
      else
      {
        Timer.delay(0.100);
      }
    }

    // Assume true if major version is correct, otherwise false
    canCoderValid = (fwvMajor >= kMajorVersion);

    if (config != null)
    {
      if ((status = canCoder.getConfigurator( ).apply(config, kCANTimeout)) != StatusCode.OK)
      {
        DataLogManager.log(String.format("%s: CC ID %2d %15s (%12s) getConfigurator: '%s'", kName, devID, name,
            "CTRE desc unsupp", status.getDescription( )));
        canCoderValid = false;
      }
    }

    DataLogManager.log(
        String.format("%s: CC ID %2d %15s (%12s) ver: %d.%d.%d.%d is '%s' - %s", kName, devID, name, "CTRE desc unsupp", fwvMajor,
            fwvMinor, fwvBugfix, fwvBuild, status.getDescription( ), (canCoderValid) ? "VALID" : "error: UNRESPONSIVE!"));

    return canCoderValid;
  }

  /****************************************************************************
   * 
   * Initialize a Pigeon2 IMU and display the result
   * 
   * @param pigeon2
   *          reference to a pigeon2
   * @param config
   *          the pigeon2 configuration to be programmed
   * @return true if successfully initialized
   */
  public boolean pigeon2Initialize(Pigeon2 pigeon2, Pigeon2Configuration config)
  {
    StatusCode status = StatusCode.StatusCodeNotInitialized;
    int devID = 0;
    int fwvMajor = 0;
    int fwvMinor = 0;
    int fwvBugfix = 0;
    int fwvBuild = 0;
    boolean pigeon2Valid;

    // Display Talon firmware versions
    devID = pigeon2.getDeviceID( );

    Timer.delay(0.250);

    // This can take multiple attempts before ready
    for (int i = 0; i < kRetries && fwvMajor == 0; i++)
    {
      StatusSignal<Integer> statusSignal = pigeon2.getVersion( );
      status = statusSignal.getStatus( );
      if (status.isOK( ))
      {
        fwvMajor = (statusSignal.getValue( ) >> 24) & 0xff;
        fwvMinor = (statusSignal.getValue( ) >> 16) & 0xff;
        fwvBugfix = (statusSignal.getValue( ) >> 8) & 0xff;
        fwvBuild = (statusSignal.getValue( ) >> 0) & 0xff;
      }
      else
      {
        Timer.delay(0.100);
      }
    }

    // Assume true if major version is correct, otherwise false
    pigeon2Valid = (fwvMajor >= kMajorVersion);

    if (config != null)
    {
      if ((status = pigeon2.getConfigurator( ).apply(config, kCANTimeout)) != StatusCode.OK)
      {
        DataLogManager.log(String.format("%s: P2 ID %2d %15s (%12s) getConfigurator: '%s'", kName, devID, "CTRE desc unsupp",
            status.getDescription( )));
        pigeon2Valid = false;
      }

      // Configure sensor settings
      if ((status = pigeon2.setYaw(0.0)) != StatusCode.OK)
      {
        DataLogManager.log(
            String.format("%s: P2 ID %2d %15s (%12s) setYaw: '%s'", kName, devID, "CTRE desc unsupp", status.getDescription( )));
        pigeon2Valid = false;
      }
    }

    DataLogManager.log(String.format("%s: P2 ID %2d %15s (%12s) ver: %d.%d.%d.%d is '%s' - %s", kName, devID, "CTRE desc unsupp",
        fwvMajor, fwvMinor, fwvBugfix, fwvBuild, status.getDescription( ), (pigeon2Valid) ? "VALID" : "error: UNRESPONSIVE!"));

    return pigeon2Valid;
  }

  /****************************************************************************
   * 
   * Initialize a CANdle and display the result
   * 
   * @param candle
   *          reference to a candle
   * @param config
   *          the candle configuration to be programmed
   * @return true if successfully initialized
   */
  public boolean candleInitialize(CANdle candle, String name, CANdleConfiguration config)
  {
    StatusCode status = StatusCode.StatusCodeNotInitialized;
    int devID = 0;
    int fwvMajor = 0;
    int fwvMinor = 0;
    int fwvBugfix = 0;
    int fwvBuild = 0;
    boolean candleValid;

    // Display Talon firmware versions
    devID = candle.getDeviceID( );

    Timer.delay(0.250);

    // This can take multiple attempts before ready
    for (int i = 0; i < kRetries && fwvMajor == 0; i++)
    {
      StatusSignal<Integer> statusSignal = candle.getVersion( );
      status = statusSignal.getStatus( );
      if (status.isOK( ))
      {
        fwvMajor = (statusSignal.getValue( ) >> 24) & 0xff;
        fwvMinor = (statusSignal.getValue( ) >> 16) & 0xff;
        fwvBugfix = (statusSignal.getValue( ) >> 8) & 0xff;
        fwvBuild = (statusSignal.getValue( ) >> 0) & 0xff;
      }
      else
      {
        Timer.delay(0.100);
      }
    }

    // Assume true if major version is correct, otherwise false
    candleValid = (fwvMajor >= kMajorVersion);

    if (config != null)
    {
      if ((status = candle.getConfigurator( ).apply(config, kCANTimeout)) != StatusCode.OK)
      {
        DataLogManager.log(String.format("%s: CN ID %2d %15s (%12s) getConfigurator: '%s'", kName, devID, name,
            "CTRE desc unsupp", status.getDescription( )));
        candleValid = false;
      }
    }

    DataLogManager
        .log(String.format("%s: CN ID %2d %15s (%12s) ver: %d.%d.%d.%d is '%s' - %s", kName, devID, name, "CTRE desc unsupp",
            fwvMajor, fwvMinor, fwvBugfix, fwvBuild, status.getDescription( ), (candleValid) ? "VALID" : "error: UNRESPONSIVE!"));

    return candleValid;
  }

  /****************************************************************************
   * 
   * Print fault flags for a talonFX
   * 
   * @param talonFX
   *          reference to a talonFX motor controller
   * @param name
   *          descriptive name of the talonFX
   */
  public void talonFXPrintFaults(TalonFX talonFX, String name)
  {
    DataLogManager.log(String.format("%s: ------------------ DUMP FAULTS ------------------", name));
    DataLogManager.log(String.format("  FaultField ......................... 0x%08x 0x%08x", talonFX.getFaultField( ).getValue( ),
        talonFX.getStickyFaultField( ).getValue( )));
    DataLogManager.log(String.format("  Hardware ........................... %5s %5s", talonFX.getFault_Hardware( ).getValue( ),
        talonFX.getStickyFault_Hardware( ).getValue( )));
    DataLogManager.log(String.format("  ProcTemp ........................... %5s %5s", talonFX.getFault_ProcTemp( ).getValue( ),
        talonFX.getStickyFault_ProcTemp( ).getValue( )));
    DataLogManager.log(String.format("  DeviceTemp ......................... %5s %5s", talonFX.getFault_DeviceTemp( ).getValue( ),
        talonFX.getStickyFault_DeviceTemp( ).getValue( )));
    DataLogManager.log(String.format("  Undervoltage ....................... %5s %5s",
        talonFX.getFault_Undervoltage( ).getValue( ), talonFX.getStickyFault_Undervoltage( ).getValue( )));
    DataLogManager.log(String.format("  BootDuringEnable ................... %5s %5s",
        talonFX.getFault_BootDuringEnable( ).getValue( ), talonFX.getStickyFault_BootDuringEnable( ).getValue( )));
    DataLogManager.log(String.format("  UnlicensedFeatureInUse ............. %5s %5s",
        talonFX.getFault_UnlicensedFeatureInUse( ).getValue( ), talonFX.getStickyFault_UnlicensedFeatureInUse( ).getValue( )));
    DataLogManager.log(String.format("  BridgeBrownout ..................... %5s %5s",
        talonFX.getFault_BridgeBrownout( ).getValue( ), talonFX.getStickyFault_BridgeBrownout( ).getValue( )));
    DataLogManager.log(String.format("  RemoteSensorReset .................. %5s %5s",
        talonFX.getFault_RemoteSensorReset( ).getValue( ), talonFX.getStickyFault_RemoteSensorReset( ).getValue( )));
    DataLogManager.log(String.format("  MissingDifferentialFX .............. %5s %5s",
        talonFX.getFault_MissingDifferentialFX( ).getValue( ), talonFX.getStickyFault_MissingDifferentialFX( ).getValue( )));
    DataLogManager.log(String.format("  RemoteSensorPosOverflow ............ %5s %5s",
        talonFX.getFault_RemoteSensorPosOverflow( ).getValue( ), talonFX.getStickyFault_RemoteSensorPosOverflow( )));
    DataLogManager.log(String.format("  OverSupplyV ........................ %5s %5s",
        talonFX.getFault_OverSupplyV( ).getValue( ), talonFX.getStickyFault_OverSupplyV( ).getValue( )));
    DataLogManager.log(String.format("  UnstableSupplyV .................... %5s %5s",
        talonFX.getFault_UnstableSupplyV( ).getValue( ), talonFX.getStickyFault_UnstableSupplyV( ).getValue( )));
    DataLogManager.log(String.format("  ReverseHardLimit ................... %5s %5s",
        talonFX.getFault_ReverseHardLimit( ).getValue( ), talonFX.getStickyFault_ReverseHardLimit( ).getValue( )));
    DataLogManager.log(String.format("  ForwardHardLimit ................... %5s %5s",
        talonFX.getFault_ForwardHardLimit( ).getValue( ), talonFX.getStickyFault_ForwardHardLimit( ).getValue( )));
    DataLogManager.log(String.format("  ReverseSoftLimit ................... %5s %5s",
        talonFX.getFault_ReverseSoftLimit( ).getValue( ), talonFX.getStickyFault_ReverseSoftLimit( ).getValue( )));
    DataLogManager.log(String.format("  ForwardSoftLimit ................... %5s %5s",
        talonFX.getFault_ForwardSoftLimit( ).getValue( ), talonFX.getStickyFault_ForwardSoftLimit( ).getValue( )));
    DataLogManager.log(String.format("  RemoteSensorDataInvalid ............ %5s %5s",
        talonFX.getFault_RemoteSensorDataInvalid( ).getValue( ), talonFX.getStickyFault_RemoteSensorDataInvalid( )));
    DataLogManager.log(String.format("  FusedSensorOutOfSync ............... %5s %5s",
        talonFX.getFault_FusedSensorOutOfSync( ).getValue( ), talonFX.getStickyFault_FusedSensorOutOfSync( ).getValue( )));
    DataLogManager.log(String.format("  StatorCurrLimit .................... %5s %5s",
        talonFX.getFault_StatorCurrLimit( ).getValue( ), talonFX.getStickyFault_StatorCurrLimit( ).getValue( )));
    DataLogManager.log(String.format("  SupplyCurrLimit .................... %5s %5s",
        talonFX.getFault_SupplyCurrLimit( ).getValue( ), talonFX.getStickyFault_SupplyCurrLimit( ).getValue( )));
    DataLogManager.log(String.format("  UsingFusedCANcoderWhileUnlicensed .. %5s %5s",
        talonFX.getFault_UsingFusedCANcoderWhileUnlicensed( ).getValue( ),
        talonFX.getStickyFault_UsingFusedCANcoderWhileUnlicensed( ).getValue( )));
  }

  /****************************************************************************
   * 
   * Print fault flags for a CANcoder
   * 
   * @param canCoder
   *          reference to a CANcoder
   * @param name
   *          descriptive name of the CANcoder
   */
  public void canCoderPrintFaults(CANcoder canCoder, String name)
  {
    DataLogManager.log(String.format("%s: ------------------ DUMP FAULTS ------------------", name));
    DataLogManager.log(String.format("  FaultField ......................... 0x%08x 0x%08x",
        canCoder.getFaultField( ).getValue( ), canCoder.getStickyFaultField( ).getValue( )));
    DataLogManager.log(String.format("  Hardware ........................... %5s %5s", canCoder.getFault_Hardware( ).getValue( ),
        canCoder.getStickyFault_Hardware( ).getValue( )));
    DataLogManager.log(String.format("  Undervoltage ....................... %5s %5s",
        canCoder.getFault_Undervoltage( ).getValue( ), canCoder.getStickyFault_Undervoltage( ).getValue( )));
    DataLogManager.log(String.format("  BootDuringEnable ................... %5s %5s",
        canCoder.getFault_BootDuringEnable( ).getValue( ), canCoder.getStickyFault_BootDuringEnable( ).getValue( )));
    DataLogManager.log(String.format("  UnlicensedFeatureInUse ............. %5s %5s",
        canCoder.getFault_UnlicensedFeatureInUse( ).getValue( ), canCoder.getStickyFault_UnlicensedFeatureInUse( ).getValue( )));
    DataLogManager.log(String.format("  BadMagnet .......................... %5s %5s", canCoder.getFault_BadMagnet( ).getValue( ),
        canCoder.getStickyFault_BadMagnet( ).getValue( )));
  }

  /****************************************************************************
   * 
   * Print fault flags for a CANrange
   * 
   * @param canrange
   *          reference to a CANrange
   * @param name
   *          descriptive name of the CANrange
   */
  public void canRangePrintFaults(CANrange canRange, String name)
  {
    DataLogManager.log(String.format("%s: ------------------ DUMP FAULTS ------------------", name));
    DataLogManager.log(String.format("  FaultField ......................... 0x%08x 0x%08x",
        canRange.getFaultField( ).getValue( ), canRange.getStickyFaultField( ).getValue( )));
    DataLogManager.log(String.format("  Hardware ........................... %5s %5s", canRange.getFault_Hardware( ).getValue( ),
        canRange.getStickyFault_Hardware( ).getValue( )));
    DataLogManager.log(String.format("  Undervoltage ....................... %5s %5s",
        canRange.getFault_Undervoltage( ).getValue( ), canRange.getStickyFault_Undervoltage( ).getValue( )));
    DataLogManager.log(String.format("  BootDuringEnable ................... %5s %5s",
        canRange.getFault_BootDuringEnable( ).getValue( ), canRange.getStickyFault_BootDuringEnable( ).getValue( )));
    DataLogManager.log(String.format("  UnlicensedFeatureInUse ............. %5s %5s",
        canRange.getFault_UnlicensedFeatureInUse( ).getValue( ), canRange.getStickyFault_UnlicensedFeatureInUse( ).getValue( )));
  }

  /****************************************************************************
   * 
   * Print fault flags for a CANdle
   * 
   * @param candle
   *          reference to a CANdle LED controller
   * @param name
   *          descriptive name of the CANdle
   */
  public void candlePrintFaults(CANdle candle, String name)
  {
    DataLogManager.log(String.format("%s: ------------------ DUMP FAULTS ------------------", name));
    DataLogManager.log(String.format("  5VTooHigh ......... %5s %5s", candle.getFault_5VTooHigh( ).toString( ),
        candle.getStickyFault_5VTooHigh( ).toString( )));
    DataLogManager.log(String.format("  5VTooLow .......... %5s %5s", candle.getFault_5VTooLow( ).toString( ),
        candle.getStickyFault_5VTooLow( ).toString( )));
    DataLogManager.log(String.format("  BootDuringEnable .. %5s %5s", candle.getFault_BootDuringEnable( ).toString( ),
        candle.getStickyFault_BootDuringEnable( ).toString( )));
    DataLogManager.log(String.format("  Hardware .......... %5s %5s", candle.getFault_Hardware( ).toString( ),
        candle.getStickyFault_BootDuringEnable( ).toString( )));
    DataLogManager.log(String.format("  Overvoltage ....... %5s %5s", candle.getFault_Overvoltage( ).toString( ),
        candle.getStickyFault_Overvoltage( ).toString( )));
    DataLogManager.log(String.format("  ShortCircuit ...... %5s %5s", candle.getFault_ShortCircuit( ).toString( ),
        candle.getStickyFault_ShortCircuit( ).toString( )));
    DataLogManager.log(String.format("  SoftwareFuse ...... %5s %5s", candle.getFault_SoftwareFuse( ).toString( ),
        candle.getStickyFault_SoftwareFuse( ).toString( )));
    DataLogManager.log(String.format("  Thermal ........... %5s %5s", candle.getFault_Thermal( ).toString( ),
        candle.getStickyFault_Thermal( ).toString( )));
    DataLogManager.log(String.format("  Undervoltage ...... %5s %5s", candle.getFault_Undervoltage( ).toString( ),
        candle.getStickyFault_Undervoltage( ).toString( )));
    DataLogManager.log(String.format("  UnlicensedFeatureInUse .. %5s %5s", candle.getFault_UnlicensedFeatureInUse( ).toString( ),
        candle.getStickyFault_UnlicensedFeatureInUse( ).toString( )));
  }

}
