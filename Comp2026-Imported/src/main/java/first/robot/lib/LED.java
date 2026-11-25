//
// LED class - LED feedback on robot
//
package first.robot.lib;

import org.wpilib.hardware.bus.CANPort;
import org.wpilib.system.DataLogManager;
import org.wpilib.tunable.Selectable;
import org.wpilib.tunable.Tunables;
import org.wpilib.util.Color;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.ColorFlowAnimation;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.EmptyAnimation;
import com.ctre.phoenix6.controls.FireAnimation;
import com.ctre.phoenix6.controls.LarsonAnimation;
import com.ctre.phoenix6.controls.RainbowAnimation;
import com.ctre.phoenix6.controls.RgbFadeAnimation;
import com.ctre.phoenix6.controls.SingleFadeAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.controls.TwinkleAnimation;
import com.ctre.phoenix6.controls.TwinkleOffAnimation;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.AnimationDirectionValue;
import com.ctre.phoenix6.signals.Enable5VRailValue;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StatusLedWhenActiveValue;
import com.ctre.phoenix6.signals.StripTypeValue;
import com.ctre.phoenix6.signals.VBatOutputModeValue;
import com.ctre.phoenix6.sim.CANdleSimState;

import first.robot.Constants.Ports;
import first.robot.Constants.LEDConsts.ANIMATION;
import first.robot.Constants.LEDConsts.COLOR;
import first.robot.lib.phoenix.PhoenixUtil6;

/****************************************************************************
 * 
 * LED class to control LED changes and provide command factory
 */
public class LED
{
  // Constants

  /*
   * Slot 0: LEDs 0 to 7 (0-7 are onboard, 8-399 are an external strip)
   * - CANdle supports 8 animation slots (0-7).
   */
  private static final int                     kSlot          = 0;    // Animation slot (0 - 7)
  private static final int                     kSlot0StartIdx = 0;    // LEDs controled by animation slot 0
  private static final int                     kSlot0EndIdx   = 12;

  private static final double                  kFrameRate     = 2.0;  // Animation speed in Hz 2-1000
  private static final double                  kBrightness    = 1.0;  // Brightness level 0.0 - 1.0
  private static final AnimationDirectionValue kDirection     = AnimationDirectionValue.Forward;
  private static final int                     kSize          = 4;    // Number of LEDs in animations that use partial string
  private static final double                  kSparking      = 0.4;  // Sparking level 0.0 - 1.0
  private static final double                  kCooling       = 0.5;  // Cooling level 0.0 - 1.0
  private static final double                  kMaxLEDsOn     = 0.5;  // Number of LEDs on at once 

  /*
   * LED object for managing control requests
   */
  private class LEDRequest
  {
    COLOR     color     = COLOR.OFF;
    ANIMATION animation = ANIMATION.SOLID;
    double    rate      = kFrameRate;

    LEDRequest(COLOR reqColor, ANIMATION reqAnimation, double reqRate)
    {
      color = reqColor;
      animation = reqAnimation;
      rate = reqRate;
    }
  }

  /* Color can be constructed from RGBW, a WPILib Color/Color8Bit, HSV, or hex */
  private static final RGBWColor      kWhite             = new RGBWColor(Color.WHITE).scaleBrightness(kBrightness);
  private static final RGBWColor      kRed               = new RGBWColor(Color.RED).scaleBrightness(kBrightness);
  private static final RGBWColor      kOrange            = new RGBWColor(Color.ORANGE).scaleBrightness(kBrightness);
  private static final RGBWColor      kYellow            = new RGBWColor(Color.YELLOW).scaleBrightness(kBrightness);
  private static final RGBWColor      kGreen             = new RGBWColor(Color.GREEN).scaleBrightness(kBrightness);
  private static final RGBWColor      kBlue              = new RGBWColor(Color.BLUE).scaleBrightness(kBrightness);
  private static final RGBWColor      kViolet            = new RGBWColor(Color.VIOLET).scaleBrightness(kBrightness);
  private static final RGBWColor      kOff               = new RGBWColor(0, 0, 0).scaleBrightness(kBrightness);

  // Member objects
  private final CANdle                m_candle           = new CANdle(Ports.kCANID_CANdle, new CANBus(CANPort.CAN_S0));
  private final CANdleSimState        m_candleSim        = new CANdleSimState(m_candle);
  private final Selectable<COLOR>     m_colorChooser     = new Selectable<COLOR>( );
  private final Selectable<ANIMATION> m_animationChooser = new Selectable<ANIMATION>( );

  private String                      m_name             = "";
  private boolean                     m_candleValid      = false;
  private LEDRequest                  m_request          = new LEDRequest(COLOR.OFF, ANIMATION.SOLID, kFrameRate);
  private LEDRequest                  m_active           = new LEDRequest(COLOR.OFF, ANIMATION.SOLID, kFrameRate);

  /****************************************************************************
   * 
   * Constructor
   */
  public LED( )
  {
    setName("LED");

    /* Configure CANdle */
    var cfg = new CANdleConfiguration( );

    /* set the LED strip type and brightness */
    cfg.LED.StripType = StripTypeValue.GRB;
    cfg.LED.BrightnessScalar = 0.5;

    /* disable status LED when being controlled */
    cfg.CANdleFeatures.StatusLedWhenActive = StatusLedWhenActiveValue.Disabled;
    cfg.CANdleFeatures.VBatOutputMode = VBatOutputModeValue.Off;
    cfg.CANdleFeatures.Enable5VRail = Enable5VRailValue.Enabled;

    m_candleValid = PhoenixUtil6.getInstance( ).candleInitialize6(m_candle, cfg);

    /* clear all previous animations */
    if (m_candleValid)
    {
      for (int i = 0; i < 8; ++i)
      {
        m_candle.setControl(new EmptyAnimation(i));
      }
    }

    initDashboard( );
    initialize( );
  }

  /****************************************************************************
   * 
   * Getter and setter for managing the class name
   */
  private void setName(String name)
  {
    m_name = name;
  }

  public String getName( )
  {
    return m_name;
  }

  /****************************************************************************
   * 
   * Periodic actions that run every scheduler loop time (20 msec)
   */
  public void periodic( )
  {
    // This method will be called once per scheduler run

    checkLEDRequests( );
  }

  /****************************************************************************
   * 
   * Periodic actions that run every scheduler loop time (20 msec) during simulation
   */
  public void simulationPeriodic( )
  {
    // This method will be called once per scheduler run during simulation

    if (m_candleValid)
    {
      @SuppressWarnings("unused")
      StatusCode m_status = m_candleSim.getLastStatusCode( );
    }
  }

  /****************************************************************************
   * 
   * Initialize dashboard widgets
   */
  private void initDashboard( )
  {
    // Initialize dashboard widgets

    // Add options for colors in Dashboard
    m_colorChooser.addDefault("OFF", COLOR.OFF);
    m_colorChooser.add("WHITE", COLOR.WHITE);
    m_colorChooser.add("RED", COLOR.RED);
    m_colorChooser.add("ORANGE", COLOR.ORANGE);
    m_colorChooser.add("YELLOW", COLOR.YELLOW);
    m_colorChooser.add("GREEN", COLOR.GREEN);
    m_colorChooser.add("BLUE", COLOR.BLUE);
    m_colorChooser.add("PURPLE", COLOR.PURPLE);
    Tunables.publish("color", m_colorChooser);
    m_colorChooser.onChange(this::updateColorChooserCallback);

    // Animation options in Dashboard
    m_animationChooser.addDefault("SOLID", ANIMATION.SOLID);
    m_animationChooser.add("COLORFLOW", ANIMATION.COLORFLOW);
    m_animationChooser.add("FIRE", ANIMATION.FIRE);
    m_animationChooser.add("LARSON", ANIMATION.LARSON);
    m_animationChooser.add("RAINBOW", ANIMATION.RAINBOW);
    m_animationChooser.add("RGBFADE", ANIMATION.RGBFADE);
    m_animationChooser.add("SINGLEFADE", ANIMATION.SINGLEFADE);
    m_animationChooser.add("STROBE", ANIMATION.STROBE);
    m_animationChooser.add("TWINKLE", ANIMATION.TWINKLE);
    m_animationChooser.add("TWINKLEOFF", ANIMATION.TWINKLEOFF);
    Tunables.publish("animation", m_animationChooser);
    m_animationChooser.onChange(this::updateAnimationChooserCallback);
  }

  /****************************************************************************
   * 
   * Detect chooser changes and request new color and animation
   */
  private void updateColorChooserCallback(COLOR option)
  {
    COLOR newColor = (option != null) ? option : COLOR.OFF;
    ANIMATION animation = m_animationChooser.getSelected( );
    if (animation == null)
    {
      animation = ANIMATION.SOLID;
    }
    setLEDs(newColor, animation, kFrameRate);
  }

  private void updateAnimationChooserCallback(ANIMATION option)
  {
    COLOR color = m_colorChooser.getSelected( );
    if (color == null)
    {
      color = COLOR.OFF;
    }
    ANIMATION newAnimation = (option != null) ? option : ANIMATION.SOLID;
    setLEDs(color, newAnimation, kFrameRate);
  }

  // Put methods for controlling this subsystem here. Call these from Commands.

  /****************************************************************************
   * 
   * Initialize subsystem during robot mode changes
   */
  public void initialize( )
  {
    DataLogManager.log(String.format("%s: Subsystem initialized!", getName( )));
    setLEDs(COLOR.OFF, ANIMATION.SOLID, 0.0);
  }

  /****************************************************************************
   * 
   * Write out hardware faults and reset sticky faults
   */
  public void printFaults( )
  {
    if (m_candleValid)
    {
      PhoenixUtil6.getInstance( ).candlePrintFaults(m_candle, "CANdle");
      m_candle.clearStickyFaults( );
    }
    else
    {
      DataLogManager.log(String.format("%s: CANdle invalid, skipping fault dump", getName( )));
    }
  }

  //////////////////////////////////////////////////////////////////////////// //////////////////////
  /// PRIVATE HELPERS //////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////

  /****************************************************************************
   * 
   * Set LEDs based on the requested color and animation
   */
  private void checkLEDRequests( )
  {
    RGBWColor ledColor;
    ControlRequest animation;

    if (m_request.color != m_active.color || m_request.animation != m_active.animation || m_request.rate != m_active.rate)
    {
      m_active.color = m_request.color;
      m_active.animation = m_request.animation;
      m_active.rate = m_request.rate;

      // Convert color request to a control request
      switch (m_active.color)
      {
        default :
        case OFF :
          ledColor = kOff;
          break;
        case WHITE :
          ledColor = kWhite;
          break;
        case RED :
          ledColor = kRed;
          break;
        case ORANGE :
          ledColor = kOrange;
          break;
        case YELLOW :
          ledColor = kYellow;
          break;
        case GREEN :
          ledColor = kGreen;
          break;
        case BLUE :
          ledColor = kBlue;
          break;
        case PURPLE :
          ledColor = kViolet;
          break;
      }

      // Convert animation request to a control request
      switch (m_active.animation)
      {
        default :
        case SOLID :
          m_candle.setControl(new EmptyAnimation(0)); // Animation must be cleared fro solid colors to work
          animation = new SolidColor(kSlot0StartIdx, kSlot0EndIdx).withColor(ledColor);
          break;
        case COLORFLOW :
          animation = new ColorFlowAnimation(kSlot0StartIdx, kSlot0EndIdx).withSlot(kSlot).withFrameRate(m_active.rate)
              .withDirection(kDirection).withColor(ledColor);
          break;
        case FIRE :
          animation = new FireAnimation(kSlot0StartIdx, kSlot0EndIdx).withSlot(kSlot).withFrameRate(m_active.rate)
              .withDirection(kDirection).withBrightness(kBrightness).withCooling(kCooling).withSparking(kSparking);
          break;
        case LARSON :
          animation = new LarsonAnimation(kSlot0StartIdx, kSlot0EndIdx).withSlot(kSlot).withFrameRate(m_active.rate)
              .withSize(kSize).withColor(ledColor);
          break;
        case RAINBOW :
          animation = new RainbowAnimation(kSlot0StartIdx, kSlot0EndIdx).withSlot(kSlot).withFrameRate(m_active.rate)
              .withDirection(kDirection).withBrightness(kBrightness);
          break;
        case RGBFADE :
          animation = new RgbFadeAnimation(kSlot0StartIdx, kSlot0EndIdx).withSlot(kSlot).withFrameRate(m_active.rate)
              .withBrightness(kBrightness);
          break;
        case SINGLEFADE :
          animation = new SingleFadeAnimation(kSlot0StartIdx, kSlot0EndIdx).withSlot(kSlot).withFrameRate(m_active.rate)
              .withColor(ledColor);
          break;
        case STROBE :
          animation =
              new StrobeAnimation(kSlot0StartIdx, kSlot0EndIdx).withSlot(kSlot).withFrameRate(m_active.rate).withColor(ledColor);
          break;
        case TWINKLE :
          animation = new TwinkleAnimation(kSlot0StartIdx, kSlot0EndIdx).withSlot(kSlot).withFrameRate(m_active.rate)
              .withMaxLEDsOnProportion(kMaxLEDsOn).withColor(ledColor);
          break;
        case TWINKLEOFF :
          animation = new TwinkleOffAnimation(kSlot0StartIdx, kSlot0EndIdx).withSlot(kSlot).withFrameRate(m_active.rate)
              .withMaxLEDsOnProportion(kMaxLEDsOn).withColor(ledColor);
          break;
      }

      DataLogManager.log(String.format("%s: CANdle active now %s, %s", getName( ), m_active.color, m_active.animation));
      if (m_candleValid)
      {
        m_candle.setControl(animation);
      }

    }
  }

  /****************************************************************************
   * 
   * Set LED requests based on the requested color and animation
   */
  public void setLEDs(COLOR color, ANIMATION animation, double rate)
  {
    m_request.color = color;
    m_request.animation = animation;
    m_request.rate = rate;
    DataLogManager.log(String.format("%s: CANdle request is %s, %s, %.3f", getName( ), color, animation, rate));
  }

  //////////////////////////////////////////////////////////////////////////// //////////////////////
  /// COMMAND FACTORIES ////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////

  /****************************************************************************
   * 
   * Create LED set command
   * 
   * @param color
   *          LED color to display
   * @param animation
   *          LED animation pattern to use
   * @return instant command that changes LEDs
   */

  // public Command getLEDCommand(COLOR color, ANIMATION animation, double rate)
  // {
  //   return new InstantCommand(            // Command that runs exactly once
  //       ( ) -> setLEDs(color, animation, rate) // Method to call
  //   )                                     //
  //       .withName("LEDSet")          //
  //       .ignoringDisable(true);
  // }

}
