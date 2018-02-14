package com.github.jeremyroy.simone.phone_imu;

import com.stormbots.MiniPID;

class RateController
{
    private MiniPID m_roll_rate_pid;
    private MiniPID m_pitch_rate_pid;
    private MiniPID m_yaw_rate_pid;

    public RateController(Vect3F roll, Vect3F pitch, Vect3F yaw)
    {
        m_roll_rate_pid = new MiniPID(roll.x, roll.y, roll.z);
        m_pitch_rate_pid = new MiniPID(pitch.x, pitch.y, pitch.z);
        m_yaw_rate_pid = new MiniPID(yaw.x, yaw.y, yaw.z);
    }
    
    public void setDesiredRates(Vect3F rates)
    {
        m_pitch_rate_pid.setSetpoint(rates.x);
        m_roll_rate_pid.setSetpoint(rates.y);
        m_yaw_rate_pid.setSetpoint(rates.z);
    }

    public Vect3F getOutput(Vect3F sensor_rates)
    {
        double sensor_roll_rate = sensor_rates.y;
        double sensor_pitch_rate = sensor_rates.x;
        double sensor_yaw_rate = sensor_rates.z;

        // TODO: verify that the measure and desired reference frames are alligned
        // 
        // Run controller 
        // Assume both angular velocities are in radians per second
        //
        // Calculate thrust adjustments (in % of full scale)
        double roll_thrust_adj = 0.0;
        double pitch_thrust_adj = 0.0;
        double yaw_thrust_adj = 0.0;
        
        roll_thrust_adj  = m_roll_rate_pid.getOutput(sensor_roll_rate);
        pitch_thrust_adj = m_pitch_rate_pid.getOutput(sensor_pitch_rate);
        yaw_thrust_adj   = m_yaw_rate_pid.getOutput(sensor_yaw_rate);

        // Format and return thrust adjustments
        Vect3F thrust_adj = new Vect3F();
        thrust_adj.x = roll_thrust_adj;
        thrust_adj.y = pitch_thrust_adj;
        thrust_adj.z = yaw_thrust_adj;

        return thrust_adj;
    }

    public void reset(Vect3F roll, Vect3F pitch, Vect3F yaw)
    {
        // Update PID values
        m_roll_rate_pid.setPID(roll.x, roll.y, roll.z);
        m_pitch_rate_pid.setPID(pitch.x, pitch.y, pitch.z);
        m_yaw_rate_pid.setPID(yaw.x, yaw.y, yaw.z);

        // Reset PIDs
        m_roll_rate_pid.reset();
        m_pitch_rate_pid.reset();
        m_yaw_rate_pid.reset();
    }
}