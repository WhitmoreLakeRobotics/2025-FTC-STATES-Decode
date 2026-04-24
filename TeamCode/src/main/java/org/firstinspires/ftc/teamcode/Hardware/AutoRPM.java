package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

// Auto calculates RPM for launcher
public class AutoRPM {
    
    public enum Mode {
        MANUAL,
        AUTO
    }

    public Mode mode = Mode.MANUAL;

    private boolean debug = true;

    public boolean Measure = false;
    private double Distance = 0;
    private double[] rpms = {0,0};
    HardwareMap hardwareMap;
    Telemetry telemetry;

    public void init() {
        telemetry.addData("AutoRPM init", true);
    }

    public void init_loop() {}

    public void start() {}

    public void stop() {}

    public void loop() {
        update();
    }

    public void update() {

        if (mode != Mode.AUTO) return;

        if (!Measure) return;

        rpms = calculateRPMs(Distance);

        if (debug) {
            telemetry.addData("In AutoRPM Mode", mode);
            telemetry.addData("In AutoRPM Measure is", Measure);
            telemetry.addData("rpms 0 = ", rpms[0]);
            telemetry.addData("rpms 1 = ", rpms[1]);
        }
    }

    public double[] calculateRPMs(double Distance) {

        // Top motor interpolation
        double d1 = 0.5;       // meters
        double r1top = 1600;

        double d2 = 2.4;       // meters
        double r2top = 3500;

        double m_top = (r2top - r1top) / (d2 - d1);
        double b_top = r1top - m_top * d1;

        double targetTopRPM = m_top * Distance + b_top;

        // Bottom motor interpolation
        double r1bottom = 3500;
        double r2bottom = 4500;

        double m_bottom = (r2bottom - r1bottom) / (d2 - d1);
        double b_bottom = r1bottom - m_bottom * d1;

        double targetBottomRPM = m_bottom * Distance + b_bottom;

        return new double[]{targetTopRPM, targetBottomRPM};
    }

    public void setDistance(double dist) {
        Distance = dist;
    }

    public double[] getRPMs() {
        return rpms;
    }
}
