package com.github.jeremyroy.simone.phone_imu;

import android.media.AudioManager;

import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.Node;
import org.ros.message.MessageListener;
import org.ros.node.topic.Subscriber;
import org.ros.node.service.ServiceServer;
import org.ros.node.service.ServiceResponseBuilder;

import simone_msgs.MotorCTRL;
import hector_uav_msgs.EnableMotors;
import hector_uav_msgs.EnableMotorsRequest;
import hector_uav_msgs.EnableMotorsResponse;

public class MotorNode extends AbstractNodeMain
{
    private final AudioManager m_audio_manager;

    private String topic_name;
    private String motor_service_name;

    private Motors m_motors;

    public MotorNode(AudioManager am) {
        this(am, "motor_ctrl", "enable_motors");
    }

    public MotorNode(AudioManager am, String topic, String motor_service) {
        /* Set up topic */
        this.topic_name = topic;
        this.motor_service_name = motor_service;

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

        // Start motors
        m_motors.start_motors();

        // Set up motor command subscriber
        Subscriber<simone_msgs.MotorCTRL> subscriber =
                connectedNode.newSubscriber(this.topic_name, simone_msgs.MotorCTRL._TYPE);
        subscriber.addMessageListener(new MessageListener<simone_msgs.MotorCTRL>() {
            @Override
            public void onNewMessage(simone_msgs.MotorCTRL message){
                if (m_motors.is_enabled()) {
                    m_motors.setMotorDuty(Motors.MOTOR_1, message.getM1());
                    m_motors.setMotorDuty(Motors.MOTOR_2, message.getM2());
                    m_motors.setMotorDuty(Motors.MOTOR_3, message.getM3());
                    m_motors.setMotorDuty(Motors.MOTOR_4, message.getM4() + 2.0); // Plus two to account for different ESC
                }
            }
        });

        // Set up motor command
        ServiceServer<EnableMotorsRequest, EnableMotorsResponse> server =
                connectedNode.newServiceServer(this.motor_service_name, EnableMotors._TYPE,
                        new ServiceResponseBuilder<EnableMotorsRequest, EnableMotorsResponse>() {
                            @Override
                            public void build(EnableMotorsRequest request, EnableMotorsResponse response)
                            {
                                if (request.getEnable() && !m_motors.is_enabled())
                                {
                                    m_motors.resume_motors();
                                }
                                else if (m_motors.is_enabled())
                                {
                                    m_motors.pause_motors();
                                }

                                response.setSuccess(true);
                            }
                        });

    }

    @Override
    public void onShutdown(Node node)
    {
        m_motors.stop_motors();
    }

} // End class

