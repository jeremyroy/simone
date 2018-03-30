package com.github.jeremyroy.simone.phone_imu;

import android.hardware.SensorManager;
import android.media.AudioManager;
import android.content.Context;
import android.os.Bundle;
import android.widget.TextView;

import org.ros.android.RosActivity;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMainExecutor;

/**
 * @author jeremy.roy@queensu.com (Jeremy Roy)
 */
public class MainActivity extends RosActivity {

    private TextView textViewIn;
    private TextView textViewOut;

    public MainActivity() {
        // The RosActivity constructor configures the notification title and ticker
        // messages.
        super("Phone IMU", "Phone IMU");
    }

    @SuppressWarnings("unchecked")
    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.main);

        long time = System.currentTimeMillis();
        textViewIn = (TextView) findViewById(R.id.textTimeIn);
        textViewOut = (TextView) findViewById(R.id.textTimeOut);
    }

    @Override
    protected void init(NodeMainExecutor nodeMainExecutor) {
        SensorManager sensorManager = (SensorManager)getSystemService(SENSOR_SERVICE);
        AudioManager audioManager = (AudioManager)getSystemService(Context.AUDIO_SERVICE);
        PhoneImu imu = new PhoneImu(sensorManager);
        MotorNode motor = new MotorNode(audioManager, textViewOut);
        FlightController controller = new FlightController(textViewIn);//getApplicationContext());

        // At this point, the user has already been prompted to either enter the URI
        // of a master to use or to start a master locally.

        // The user can easily use the selected ROS Hostname in the master chooser
        // activity.
        NodeConfiguration nodeConfiguration = NodeConfiguration.newPublic(getRosHostname());
        nodeConfiguration.setMasterUri(getMasterUri());
        nodeMainExecutor.execute(imu, nodeConfiguration);
        nodeMainExecutor.execute(motor, nodeConfiguration);
        nodeMainExecutor.execute(controller, nodeConfiguration);
    }
}