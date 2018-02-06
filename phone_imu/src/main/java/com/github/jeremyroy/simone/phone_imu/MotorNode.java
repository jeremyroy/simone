package com.github.jeremyroy.simone.phone_imu;

import android.media.AudioManager;

import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.message.MessageListener;
import org.ros.node.topic.Subscriber;

import simone_msgs.MotorCTRL;

public class MotorNode extends AbstractNodeMain
{
    private final AudioManager m_audio_manager;

    private String topic_name;

    private Motors m_motors;

    public MotorNode(AudioManager am) {
        this(am, "phone_imu");
    }

    public MotorNode(AudioManager am, String topic) {
        /* Set up topic */
        this.topic_name = topic;

        /* Set up the motors object */
        // Get the recommended sample rate from the phone
        m_audio_manager = am;
        String sampleRateStr = m_audio_manager.getProperty(AudioManager.PROPERTY_OUTPUT_SAMPLE_RATE);
        int sampleRate = Integer.parseInt(sampleRateStr);

        // Initialize motor object
        m_motors = new Motors(sampleRate);
    }

    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of("simone/motor_node");
    }

    @Override
    public void onStart(ConnectedNode connectedNode) {
        Subscriber<simone_msgs.MotorCTRL> subscriber = connectedNode.newSubscriber("motor_node", simone_msgs.MotorCTRL._TYPE);
        subscriber.addMessageListener(new MessageListener<simone_msgs.MotorCTRL>() {
            @Override
            public void onNewMessage(simone_msgs.MotorCTRL message) {
                m_motors.setMotorDuty(Motors.MOTOR_1, message.getM1());
                m_motors.setMotorDuty(Motors.MOTOR_2, message.getM2());
                m_motors.setMotorDuty(Motors.MOTOR_3, message.getM3());
                m_motors.setMotorDuty(Motors.MOTOR_4, message.getM4());
            }
        });
    }

} // End class

