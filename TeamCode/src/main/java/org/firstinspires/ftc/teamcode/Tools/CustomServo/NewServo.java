package org.firstinspires.ftc.teamcode.Tools.CustomServo;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;

public class NewServo {
    private CRServo crServo;

    public NewServo(HardwareMap hardwareMap, String name, double minRange, double maxRange) {
        // Initialize the CRServo from the hardware map
        crServo = hardwareMap.get(CRServo.class, name);

        // Check if the CRServo supports PwmControl
        if (crServo instanceof PwmControl) {
            PwmControl pwmControl = (PwmControl) crServo;

            // Set the custom PWM range
            pwmControl.setPwmRange(new PwmControl.PwmRange(minRange, maxRange));
        } else {
            throw new UnsupportedOperationException("CRServo does not support PwmControl");
        }
    }

    /**
     * Sets the power for the CRServo.
     * @param power The power to set (-1.0 to 1.0).
     */
    public void setPower(double power) {
        crServo.setPower(power);
    }

    /**
     * Disables the PWM for the CRServo.
     */
    public void disablePwm() {
        if (crServo instanceof PwmControl) {
            PwmControl pwmControl = (PwmControl) crServo;
            pwmControl.setPwmDisable();
        }
    }

    /**
     * Enables the PWM for the CRServo.
     */
    public void enablePwm() {
        if (crServo instanceof PwmControl) {
            PwmControl pwmControl = (PwmControl) crServo;
            pwmControl.setPwmEnable();
        }
    }
}
