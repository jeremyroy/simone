package com.github.jeremyroy.simone.phone_imu;

public class Quat
{
    public double w;
    public double x;
    public double y;
    public double z;

    public Quat() {}
    public Quat(double a, double b, double c, double d)
    {
        w = a;
        x = b;
        y = c;
        z = d;
    }
}