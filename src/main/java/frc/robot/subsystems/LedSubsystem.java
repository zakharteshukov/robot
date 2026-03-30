package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LedSubsystem extends SubsystemBase {
  private static final AddressableLED led = new AddressableLED(0);
  private static final AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(120);
  private static Color currentColor = Color.kGreen;

  public enum LedMode {
    OFF,
    SOLID,
    BLINK,
    SCROLL,
    BREATHE,
    PROCESSOR
  }

  public static LedMode currentLedMode = LedMode.OFF;

  public LedSubsystem() {
    led.setLength(ledBuffer.getLength());
    led.start();
  }

  public void setCurrentColor(Color color) {
    currentColor = color;
  }

  public static Color getCurrentColor() {
    return currentColor;
  }

  public void setSolidLED() {
    LEDPattern pattern = LEDPattern.solid(currentColor);
    pattern.applyTo(ledBuffer);
    led.setData(ledBuffer);
  }

  public void setGradientLED(Color color1, Color color2) {
    LEDPattern gradient =
      LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous, color1, color2);
    gradient.applyTo(ledBuffer);
    led.setData(ledBuffer);
  }

  public void setScrollLED(Color color) {
    Distance ledSpacing = Meters.of(1 / 120.0);
    LEDPattern base = LEDPattern.gradient(LEDPattern.GradientType.kContinuous, color, Color.kBlack);
    LEDPattern pattern = base.scrollAtRelativeSpeed(Percent.per(Second).of(150));
    pattern.applyTo(ledBuffer);
    led.setData(ledBuffer);
  }

  public void setScrollLED(Color color, Color color2) {
    Distance ledSpacing = Meters.of(1 / 120.0);
    LEDPattern base = LEDPattern.gradient(LEDPattern.GradientType.kContinuous, color, color2);
    LEDPattern pattern = base.scrollAtRelativeSpeed(Percent.per(Second).of(100));
    pattern.applyTo(ledBuffer);
    led.setData(ledBuffer);
  }

  public void BreatheSERPENTHEIM() {
    // Create an LED pattern that displays a red-to-blue gradient, breathing at a 2 second period
    // (0.5 Hz)
    LEDPattern base =
      LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous, Color.kGreen, Color.kBlack);
    LEDPattern pattern = base.breathe(Seconds.of(2));

    pattern.applyTo(ledBuffer);

    led.setData(ledBuffer);
  }

  public static void setBlinkLED(Color color) {
    // Create an LED pattern that displays a red-to-blue gradient, blinking at various rates.
    LEDPattern base =
      LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous, color, Color.kBlack);

    // 1.5 seconds on, 1.5 seconds off, for a total period of 3 seconds
    LEDPattern pattern = base.blink(Seconds.of(0.1));

    // 2 seconds on, 1 second off, for a total period of 3 seconds
    LEDPattern asymmetric = base.blink(Seconds.of(2), Seconds.of(1));

    // Turn the base pattern on when the RSL is on, and off when the RSL is off
    LEDPattern sycned = base.synchronizedBlink(RobotController::getRSLState);

    // Apply the LED pattern to the data buffer
    pattern.applyTo(ledBuffer);

    // Write the data to the LED strip
    led.setData(ledBuffer);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    switch (currentLedMode) {
      case OFF:
      default:
        setCurrentColor(Color.kBlack);
        setSolidLED();
        break;
      case SOLID:
        setSolidLED();
        break;
      case BLINK:
        setBlinkLED(currentColor);
        break;
      case BREATHE:
        BreatheSERPENTHEIM();
        break;
      case SCROLL:
        setScrollLED(currentColor);
        break;
      case PROCESSOR:
        setScrollLED(Color.kPurple, Color.kDarkBlue);
        break;
    }
  }
}
