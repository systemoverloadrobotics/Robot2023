// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.logging.Logger;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class Led extends SubsystemBase {
    private final Logger logger;
    public AddressableLEDBuffer ledBuffer;
    public AddressableLED led;

    /** Creates a new ExampleSubsystem. */
    public Led() {
        logger = Logger.getLogger(Led.class.getName());

        led = new AddressableLED(9);

        // Default to a length of 60, start empty output
        // Length is expensive to set, so only set it once, then just update data
        ledBuffer = new AddressableLEDBuffer(150);
        led.setLength(ledBuffer.getLength());

        // Set the data
        led.setData(ledBuffer);
        led.start();

    }

    private void led(Color8Bit color8Bit) {}

    public void setLEDColor(Color color) {
        for (var i = 0; i < ledBuffer.getLength(); i++) {
            led(new Color8Bit(color));
            // ledBuffer.setRGB(i, red, green, blue);
        }


        led.setData(ledBuffer);
    }


    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}
