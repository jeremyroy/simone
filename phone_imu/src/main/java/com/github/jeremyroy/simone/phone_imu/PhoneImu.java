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

import geometry_msgs.Quaternion;
import geometry_msgs.Vector3;
import std_msgs.Header;
import org.ros.message.Time;

public class PhoneImu extends AbstractNodeMain implements SensorEventListener
{
    private String topic_name;
    private Quaternion orientation;
    private Vector3 angular_velocity;
    private Vector3 linear_accerelation;

    private double[] cov_orientation;
    private double[] cov_angular_velocity;
    private double[] cov_linear_accerelation;

    public PhoneImu() {
        this.topic_name = "phone_imu";
        setOrientation(0,0,0,0);
        setAngluarVelocity(0,0,0);
        setLinearAcceleration(0,0,0);

        cov_orientation = new double[9]; // Automatically instantiated to zero
        cov_angular_velocity = new double[9];
        cov_linear_accerelation = new double[9];
    }

    public PhoneImu(String topic) {
        this.topic_name = topic;
        setOrientation(0,0,0,0);
        setAngluarVelocity(0,0,0);
        setLinearAcceleration(0,0,0);

        cov_orientation = new double[9];
        cov_angular_velocity = new double[9];
        cov_linear_accerelation = new double[9];
    }

    private void setOrientation(double x, double y, double z, double w) {
        this.orientation.setX(x);
        this.orientation.setY(y);
        this.orientation.setZ(z);
        this.orientation.setW(w);
    }

    private void setAngluarVelocity(double x, double y, double z) {
        this.angular_velocity.setX(x);
        this.angular_velocity.setY(y);
        this.angular_velocity.setZ(z);
    }

    private void setLinearAcceleration(double x, double y, double z) {
        this.linear_accerelation.setX(x);
        this.linear_accerelation.setY(y);
        this.linear_accerelation.setZ(z);
    }

    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of("simone/phone_imu");
    }

    public void onStart(ConnectedNode connectedNode) {
        final Publisher publisher = connectedNode.newPublisher(this.topic_name, "sensor_msgs/Imu");
        connectedNode.executeCancellableLoop(new CancellableLoop() {
            private int sequenceNumber;

            protected void setup() {
                this.sequenceNumber = 0;
            }

            protected void loop() throws InterruptedException {
                sensor_msgs.Imu data = (sensor_msgs.Imu)publisher.newMessage();
                Header new_header = (std_msgs.Header)publisher.newMessage();
                Time current_time = new Time();

                current_time.secs = (int) System.currentTimeMillis() / 1000;
                current_time.nsecs = (int) (System.currentTimeMillis() % 1000) * 1000;

                new_header.setSeq(sequenceNumber);
                new_header.setStamp(current_time);
                new_header.setFrameId("phone");

                data.setHeader(new_header);
                data.setOrientation(orientation);
                data.setAngularVelocity(angular_velocity);
                data.setLinearAcceleration(linear_accerelation);

                data.setOrientationCovariance(cov_orientation);
                data.setAngularVelocityCovariance(cov_angular_velocity);
                data.setLinearAccelerationCovariance(cov_linear_accerelation);

                publisher.publish(data);
                ++this.sequenceNumber;
                Thread.sleep(1000L);
            }
        });
    }

    @Override
    public void onSensorChanged(SensorEvent event) {
        if (event.sensor.getType() == Sensor.TYPE_ROTATION_VECTOR)
        {
            float[] quat = new float[4];

            SensorManager.getQuaternionFromVector(quat, event.values);

            this.setOrientation(quat[0], quat[1], quat[2], quat[3]);
        }
        else if (event.sensor.getType() == Sensor.TYPE_GYROSCOPE)
        {
            float[] ang_vel = event.values;
            this.setAngluarVelocity(ang_vel[0], ang_vel[1], ang_vel[2]);
        }
        else if (event.sensor.getType() == Sensor.TYPE_LINEAR_ACCELERATION)
        {
            float[] acc = event.values;
            this.setLinearAcceleration(acc[0], acc[1], acc[2]);
        }
    }

    @Override
    public void onAccuracyChanged(Sensor sensor, int i) {

    }
}