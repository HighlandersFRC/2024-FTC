package org.firstinspires.ftc.teamcode.Tools;

public class ColorConverter {

    public static int[] hsvToRgb(float h, float s, float v) {
        int r, g, b;

        // Ensure the inputs are in the correct range
        h = h % 360; // Hue should be in [0, 360)
        s = Math.max(0, Math.min(1, s)); // Saturation should be in [0, 1]
        v = Math.max(0, Math.min(1, v)); // Value should be in [0, 1]

        if (s == 0) {
            // Achromatic (gray)
            r = g = b = Math.round(v * 255);
        } else {
            float c = v * s; // Chroma
            float x = c * (1 - Math.abs((h / 60) % 2 - 1));
            float m = v - c;

            float rPrime, gPrime, bPrime;

            if (h < 60) {
                rPrime = c; gPrime = x; bPrime = 0;
            } else if (h < 120) {
                rPrime = x; gPrime = c; bPrime = 0;
            } else if (h < 180) {
                rPrime = 0; gPrime = c; bPrime = x;
            } else if (h < 240) {
                rPrime = 0; gPrime = x; bPrime = c;
            } else if (h < 300) {
                rPrime = x; gPrime = 0; bPrime = c;
            } else {
                rPrime = c; gPrime = 0; bPrime = x;
            }

            r = Math.round((rPrime + m) * 255);
            g = Math.round((gPrime + m) * 255);
            b = Math.round((bPrime + m) * 255);
        }

        return new int[]{r, g, b};
    }
}
