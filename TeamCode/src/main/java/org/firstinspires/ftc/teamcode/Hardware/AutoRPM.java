package org.firstinspires.ftc.teamcode.Hardware;

public class AutoRPM {

    public boolean Measure = false;

    private final Limey limey;
    private final Launcher launcher;

    public AutoRPM(Limey limey, Launcher launcher) {
        this.limey = limey;
        this.launcher = launcher;
    }

    public void loop() {
        update();
    }

    public void update() {

        if (!Measure) return;

        if (limey == null || launcher == null) return;

        // If no tag, do nothing
        if (limey.getTagID() < 0) return;

        double distance = limey.getTagDistance();   // meters

        double[] rpms = calculateRPMs(distance);

        launcher.setTargetRPMs(rpms[0], rpms[1]);
    }

    public double[] calculateRPMs(double distance) {

        // Top motor interpolation
        double d1 = 0.5;       // meters
        double r1top = 1900;

        double d2 = 2.9;       // meters
        double r2top = 3600;

        double m_top = (r2top - r1top) / (d2 - d1);
        double b_top = r1top - m_top * d1;

        double targetTopRPM = m_top * distance + b_top;

        // Bottom motor interpolation
        double r1bottom = 4000;
        double r2bottom = 5500;

        double m_bottom = (r2bottom - r1bottom) / (d2 - d1);
        double b_bottom = r1bottom - m_bottom * d1;

        double targetBottomRPM = m_bottom * distance + b_bottom;

        return new double[]{targetTopRPM, targetBottomRPM};
    }
}
