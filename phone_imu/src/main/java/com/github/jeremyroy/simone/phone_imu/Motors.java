package com.github.jeremyroy.simone.phone_imu;

import android.media.AudioFormat;
import android.media.AudioManager;
import android.media.AudioTrack;

// Motors API for Jeremy
public class Motors {
    public static final int MOTOR_1 = 0;
    public static final int MOTOR_2 = 1;
    public static final int MOTOR_3 = 2;
    public static final int MOTOR_4 = 3;

    private Audio audio = null;

    private double duty0 = 0;
    private double duty1 = 0;
    private double duty2 = 0;
    private double duty3 = 0;
    private int sampleRate;

    // Parameters for mapping motor duty cycles
    private int upper_bound = 10;
    private int lower_bound = 5;
    private int resolution = 2; //Number of decimal places to keep for duty cycle, which is stored as a percent

    private boolean enabled;

    public Motors(int sampleRate) {
        this.sampleRate = sampleRate;
        audio = new Audio();

        setMotorDuty(MOTOR_1, 0);
        setMotorDuty(MOTOR_2, 0);
        setMotorDuty(MOTOR_3, 0);
        setMotorDuty(MOTOR_4, 0);

        enabled = false;
    }

    public void initializeESCs() {
        // Do something
    }

    private double truncate(double val, double min, double max)
    {
        if (val < min)
            val = min;
        else if (val > max)
            val = max;
        return val;
    }

    // Expects duty to be an int and passed as a percent
    public void setMotorDuty(int motor, double duty) {
        // Fix if duty if out of range
        duty = truncate(duty, 0.0, 100.0);

        // Map a 0-100% duty cycle range to 2-8%
        float mapped_duty = (float)(duty / 100.0 * (upper_bound - lower_bound) + lower_bound);
        mapped_duty = (float)(Math.floor(mapped_duty * Math.pow(10, resolution)) / Math.pow(10, resolution));

        switch (motor) {
            case MOTOR_1:
                duty0 = mapped_duty;
                break;
            case MOTOR_2:
                duty1 = mapped_duty;
                break;
            case MOTOR_3:
                duty2 = mapped_duty;
                break;
            case MOTOR_4:
                duty3 = mapped_duty;
                break;
            default:
                throw new IndexOutOfBoundsException("Motor number must be between 0-3");
        }
    }

    public void start_motors() {
        audio.start();
    }

    public void stop_motors() {
        audio.stop();
    }

    public void pause_motors() {
        setMotorDuty(MOTOR_1, 0);
        setMotorDuty(MOTOR_2, 0);
        setMotorDuty(MOTOR_3, 0);
        setMotorDuty(MOTOR_4, 0);

        enabled = false;
    }

    public void resume_motors() {
        enabled = true;
    }

    public boolean is_enabled(){
        return enabled;
    }

    protected class Audio implements Runnable {
        private AudioTrack at = null;
        protected Thread thread = null;

        private int buff_size = AudioTrack.getMinBufferSize(sampleRate,
                AudioFormat.CHANNEL_OUT_STEREO,
                AudioFormat.ENCODING_PCM_16BIT);
        private int freq = 50;

        // Amplitude percentages for first and second motor on each channel
        private int amplitude1 = (int)(8 / 100.0 * 32767);
        private int amplitude2 = (int)(2 / 100.0 * 32767);

        public void run() {
            processAudio();


        }

        protected void start() {
            if (thread == null) {
                thread = new Thread(this, "Audio");
                thread.start();
                thread.setPriority(Thread.MAX_PRIORITY);
            }
        }

        protected void stop()
        {
            Thread t = thread;
            thread = null;

            // Wait for the thread to exit
            while (t != null && t.isAlive())
                Thread.yield();
        }

        public void processAudio() {
            at = new AudioTrack(AudioManager.STREAM_MUSIC,
                    sampleRate,
                    AudioFormat.CHANNEL_OUT_STEREO,
                    AudioFormat.ENCODING_PCM_16BIT,
                    buff_size,
                    AudioTrack.MODE_STREAM);

            double K = 2.0 * Math.PI / sampleRate;
            double phaseL = 0;
            double phaseR = 0;

            at.play();

            while (thread != null) {
                short samples[] = new short[buff_size];
                for (int i = 0; i < buff_size; i++) {
                    // Left sample
                    if (i % 2 == 0) {
                        // Motor 1
                        if (phaseL < Math.PI) {
                            double cut_off_phase = duty0 / 100.0 * 2.0 * Math.PI;
                            samples[i] = (phaseL > cut_off_phase) ? (short) 0 : (short) amplitude1;
                        }
                        // Motor 2
                        else {
                            // For motor 2, the cut off phase is offset by PI since the second
                            // half of each period is reserved for motor 2, while the first half of
                            // each period is reserved for motor 1. Note that this limits the duty
                            // cycle to 50%.
                            double cut_off_phase = (duty1 / 100.0 * 2.0 * Math.PI) + Math.PI;
                            samples[i] = (phaseL > cut_off_phase) ? (short) 0 : (short) amplitude2;
                        }

                        phaseL += (phaseL < 2.0 * Math.PI) ? freq * K : (freq * K) - (2.0 * Math.PI);
                    }
                    // Right sample
                    else {
                        // Motor 3
                        if (phaseR < Math.PI) {
                            double cut_off_phase = duty2 / 100.0 * 2.0 * Math.PI;
                            samples[i] = (phaseR > cut_off_phase) ? (short) 0 : (short) amplitude1;
                        }
                        // Motor 4
                        else {
                            // For motor 4, the cut off phase is offset by PI since the second
                            // half of each period is reserved for motor 4, while the first half of
                            // each period is reserved for motor 3. Note that this limits the duty
                            // cycle to 50%.
                            double cut_off_phase = (duty3 / 100.0 * 2.0 * Math.PI) + Math.PI;
                            samples[i] = (phaseR > cut_off_phase) ? (short) 0 : (short) amplitude2;
                        }

                        phaseR += (phaseR < 2.0 * Math.PI) ? freq * K : (freq * K) - (2.0 * Math.PI);
                    }
                }

                at.write(samples, 0, buff_size);
            }

            at.stop();
            at.release();

        }
    }
}
