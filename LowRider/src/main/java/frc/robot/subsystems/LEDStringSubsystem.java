/*-------------------------------------------------------*/
/* LED SUBSYSTEM 2023                                    */
/* Code for controlling WS2812B LED RGB Strip using      */
/* RoboRio as a controller                               */
/*-------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.OpConstants;
import frc.robot.Constants.OpConstants.LedOption;

public class LEDStringSubsystem extends SubsystemBase implements ToggleableSubsystem {
  // WITH NEW LED STRIP, COLORS ARE IN RBG not RGB
  private static final int[] YELLOW = { 255, 0, 120 };
  private static final int[] PURPLE = { 120, 150, 0 };
  private static final int[] WHITE = { 250, 250, 250 };
  private static final int[] BLUE = { 0, 255, 0 };
  private static final int[] BLACK = { 0, 0, 0 };
  private static final int[] RED = { 255, 0, 0 };
  private static final int[] GREEN = { 0, 0, 128 };
  private Timer mTimer;
  private AddressableLED m_led;
  private AddressableLEDBuffer m_ledBuffer;
  private int length;
  private boolean blink;
  private double startBlink;
  private LedOption currentColor;
  private boolean enabled;

  public LEDStringSubsystem(boolean enabled) {
    this.enabled = enabled;
    if (enabled) {
      // PWM port 9
      // Must be a PWM header, not MXP or DIO
      m_led = new AddressableLED(OpConstants.kPWM_LedString);
      mTimer = new Timer();
      // Reuse buffer
      // Default to a length of 60, start empty output
      // Length is expensive to set, so only set it once, then just update data
      m_ledBuffer = new AddressableLEDBuffer(OpConstants.kLedStringLength);
      length = m_ledBuffer.getLength();
      m_led.setLength(length);

      // Set the data
      m_led.setData(m_ledBuffer);
      m_led.start();
      this.currentColor = LedOption.BLACK;  // set to Off
      System.out.println("ledSubsystem initailized to: " + this.currentColor.toString());
    }
  }

  @Override
  public boolean isEnabled() {
    return enabled;
  }

  @Override
  public void periodic() {
  }

  public void init() {
    if (enabled) {
      // initialization stuff
      _setSingleColor(OpConstants.LedOption.BLACK);
      mTimer.start();
    }
  }

  public void setBlink(boolean blink) {
    if (enabled) {
      this.blink = blink;
      this.startBlink = mTimer.get();
      if (!blink) {
        _setCurrentColor(); // make sure if blink ended in BLACK, then turn ON
      }
    }
  }

  public void setColor(OpConstants.LedOption color) {
    if (enabled) {
      if (currentColor == color) {
        return;
      }
      SmartDashboard.putString("Color", color.toString());
      System.out.println("\n\n\nsetting color to " + color + "\n\n\n");
      currentColor = color;
      _setCurrentColor();
    }
  }

  private void _setCurrentColor() {
    if (currentColor == LedOption.INIT) {
      _setTeamColors();
    } else {
      _setSingleColor(currentColor);
    }
  }

  private void _setSingleColor(OpConstants.LedOption color) {
    int r = 0;
    int g = 0;
    int b = 0;
    switch (color) {
      case INIT:    return; // this should not be an option here
      case WHITE:   r = WHITE[0]; g = WHITE[1]; b = WHITE[2]; break;
      case YELLOW:  r = YELLOW[0]; g = YELLOW[1]; b = YELLOW[2]; break;
      case PURPLE:  r = PURPLE[0]; g = PURPLE[1]; b = PURPLE[2]; break;
      case BLUE:    r = BLUE[0]; g = BLUE[1]; b = BLUE[2]; break;
      case RED:     r = RED[0]; g = RED[1]; b = RED[2]; break;
      case GREEN:   r = GREEN[0]; g = GREEN[1]; b = GREEN[2]; break;
      case BLACK:   r = BLACK[0]; g = BLACK[1]; b = BLACK[2]; break;
    }
    for (int i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setRGB(i, r, g, b);
    }
    m_led.setData(m_ledBuffer);
    // System.out.println("Color = " + r + ", "+ g + ", "+ b);
  }

  /*
   * 5 LED blocks of Yellow/Blue for team colors
   */
  private void _setTeamColors() {
    for (int i = 0; i < m_ledBuffer.getLength(); i++) {
      if (i % 2 == 0) {
        m_ledBuffer.setRGB(i, BLUE[0], BLUE[1], BLUE[2]);
      } else {
        m_ledBuffer.setRGB(i, YELLOW[0], YELLOW[1], YELLOW[2]);
      }
    }
    m_led.setData(m_ledBuffer);
  }

}