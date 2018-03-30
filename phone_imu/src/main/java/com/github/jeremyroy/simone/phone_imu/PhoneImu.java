package com.github.jeremyroy.simone.phone_imu;

import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;

import org.ros.concurrent.CancellableLoop;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;

import std_msgs.Header;
import sensor_msgs.Imu;
import org.ros.message.Time;

public class PhoneImu extends AbstractNodeMain implements SensorEventListener
{
    private final SensorManager mSensorManager;
    private final Sensor mAccSensor;
    private final Sensor mGyroSensor;
    private final Sensor mQuatSensor;

    private int mSensorPeriodUs;

    private String topic_name;
    private double[] mOrientation;
    private double[] mAngularVelocity;
    private double[] mLinearAcceleration;

    private double[] mCovOrientation;
    private double[] mCovAngularVelocity;
    private double[] mCovLinearAcceleration;

    public PhoneImu(SensorManager sensorManager) {
        this(sensorManager, "phone_imu", 20000);
    }

    public PhoneImu(SensorManager sensorManager, String topic)
    {
        this(sensorManager, topic, 20000);
    }

    public PhoneImu(SensorManager sensorManager, String topic, int sensor_period_us) {
        // Set up topic
        this.topic_name = topic;

        // Initialize the sampling period
        mSensorPeriodUs = sensor_period_us; // Default 50 Hz

        // Set up sensors
        mSensorManager = sensorManager;
        mQuatSensor = mSensorManager.getDefaultSensor(Sensor.TYPE_ROTATION_VECTOR);
        mGyroSensor = mSensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE);
        mAccSensor = mSensorManager.getDefaultSensor(Sensor.TYPE_LINEAR_ACCELERATION);

        mOrientation = new double[4];
        mAngularVelocity = new double[3];
        mLinearAcceleration = new double[3];

        mCovOrientation = new double[9]; // Automatically instantiated to zero
        mCovAngularVelocity = new double[9];
        mCovLinearAcceleration = new double[9];
    }

    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of("simone/phone_imu");
    }

    public void onStart(ConnectedNode connectedNode) {
        final Publisher<sensor_msgs.Imu> publisher = connectedNode.newPublisher(this.topic_name, "sensor_msgs/Imu");

        mSensorManager.registerListener(this, mQuatSensor, mSensorPeriodUs);
        mSensorManager.registerListener(this, mGyroSensor, mSensorPeriodUs);
        mSensorManager.registerListener(this, mAccSensor, mSensorPeriodUs);

        // Start sensor loop
        connectedNode.executeCancellableLoop(new CancellableLoop() {
            private int sequenceNumber;

            protected void setup() {
                this.sequenceNumber = 0;
            }

            protected void loop() throws InterruptedException {
                sensor_msgs.Imu data = publisher.newMessage();
                Time current_time = new Time();

                long time = System.currentTimeMillis();

                current_time.secs = (int) (time / 1000);
                current_time.nsecs = (int) ((time % 1000) * 1000000);

                data.getHeader().setSeq(sequenceNumber);
                data.getHeader().setStamp(current_time);
                data.getHeader().setFrameId("phone");

                data.getOrientation().setW(mOrientation[3]);
                data.getOrientation().setX(mOrientation[0]);
                data.getOrientation().setY(mOrientation[1]);
                data.getOrientation().setZ(mOrientation[2]);

                data.getAngularVelocity().setX(mAngularVelocity[0]);
                data.getAngularVelocity().setY(mAngularVelocity[1]);
                data.getAngularVelocity().setZ(mAngularVelocity[2]);

                data.getLinearAcceleration().setX(mLinearAcceleration[0]);
                data.getLinearAcceleration().setY(mLinearAcceleration[1]);
                data.getLinearAcceleration().setZ(mLinearAcceleration[2]);

                data.setOrientationCovariance(mCovOrientation);
                data.setAngularVelocityCovariance(mCovAngularVelocity);
                data.setLinearAccelerationCovariance(mCovLinearAcceleration);

                publisher.publish(data);
                ++this.sequenceNumber;
                Thread.sleep(mSensorPeriodUs / 1000);
            }
        });
    }

    @Override
    public void onSensorChanged(SensorEvent event) {
        if (event.sensor.getType() == Sensor.TYPE_ROTATION_VECTOR)
        {
            // Set orientation data
            float[] quat = new float[4];
            SensorManager.getQuaternionFromVector(quat, event.values);
            this.setOrientation(quat[0], quat[1], quat[2], quat[3]);
        }
        else if (event.sensor.getType() == Sensor.TYPE_GYROSCOPE)
        {
            float[] ang_vel = event.values;
            this.setAngularVelocity(ang_vel[0], ang_vel[1], ang_vel[2]);
        }
        else if (event.sensor.getType() == Sensor.TYPE_LINEAR_ACCELERATION)
        {
            float[] acc = event.values;
            this.setLinearAcceleration(acc[0], acc[1], acc[2]);
        }
    }

    @Override
    public void onAccuracyChanged(Sensor sensor, int i) {
        // Do nothing (for now)
    }

    private void setOrientation(double x, double y, double z, double w) {
        this.mOrientation[0] = x;
        this.mOrientation[1] = y;
        this.mOrientation[2] = z;
        this.mOrientation[3] = w;
    }

    private void setAngularVelocity(double x, double y, double z) {
        this.mAngularVelocity[0] = x;
        this.mAngularVelocity[1] = y;
        this.mAngularVelocity[2] = z;
    }

    private void setLinearAcceleration(double x, double y, double z) {
        this.mLinearAcceleration[0] = x;
        this.mLinearAcceleration[1] = y;
        this.mLinearAcceleration[2] = z;
    }
}
