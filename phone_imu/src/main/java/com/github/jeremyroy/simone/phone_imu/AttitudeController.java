package com.github.jeremyroy.simone.phone_imu;

import com.stormbots.MiniPID;

public class AttitudeController
{
    private MiniPID m_roll_att_pid;
    private MiniPID m_pitch_att_pid;

    public AttitudeController(Vect3F roll, Vect3F pitch)
    {
        m_roll_att_pid = new MiniPID(roll.x, roll.y, roll.z);
        m_pitch_att_pid = new MiniPID(pitch.x, pitch.y, pitch.z);
    }

    public void setDesiredAtt(Vect3F orientation)
    {
        m_pitch_att_pid.setSetpoint(orientation.x);
        m_roll_att_pid.setSetpoint(orientation.y);
    }

    public Vect3F getOutput(Vect3F sensor_orientation)
    {
        double sensor_roll_att = sensor_orientation.y;
        double sensor_pitch_att = sensor_orientation.x;

        // TODO: verify that the measure and desired reference frames are alligned
        // 
        // Run controller 
        // Assume both angular velocities are in radians per second
        //
        // Calculate angular rate adjustments (in % of full scale)
        double roll_rate_adj = 0.0;
        double pitch_rate_adj = 0.0;
        
        roll_rate_adj  = m_roll_att_pid.getOutput(sensor_roll_att);
        pitch_rate_adj = m_pitch_att_pid.getOutput(sensor_pitch_att);

        // Format and return angular rate adjustments
        Vect3F rate_adj = new Vect3F();
        rate_adj.x = roll_rate_adj;
        rate_adj.y = pitch_rate_adj;

        return rate_adj;
    }

    public void reset(Vect3F roll, Vect3F pitch)
    {
        // Update PID values
        m_roll_att_pid.setPID(roll.x, roll.y, roll.z);
        m_pitch_att_pid.setPID(pitch.x, pitch.y, pitch.z);

        // Reset PIDs
        m_roll_att_pid.reset();
        m_pitch_att_pid.reset();
    }

}