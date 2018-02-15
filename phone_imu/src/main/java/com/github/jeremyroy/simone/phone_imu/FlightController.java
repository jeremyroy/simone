package com.github.jeremyroy.simone.phone_imu;

import java.lang.Math;

import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.Node;
import org.ros.message.MessageListener;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;
import org.ros.node.service.ServiceServer;
import org.ros.node.service.ServiceResponseBuilder;

import sensor_msgs.Imu;

import simone_msgs.MotorCTRL;
import simone_msgs.UpdatePIDs;
import simone_msgs.UpdatePIDsRequest;
import simone_msgs.UpdatePIDsResponse;

import hector_uav_msgs.ThrustCommand;
import hector_uav_msgs.YawrateCommand;
import hector_uav_msgs.AttitudeCommand;

public class FlightController extends AbstractNodeMain
{
    Publisher<simone_msgs.MotorCTRL> m_motor_publisher;

    // Topics
    private String m_imu_topic;
    private String m_thrust_topic;
    private String m_yawrate_topic;
    private String m_att_topic;
    private String m_motor_ctrl_topic;
    private String m_pid_service;

    // State attributes
    private double m_thrust;
    private double m_pitch;
    private double m_roll;
    private double m_yaw;

    // Controllers
    private RateController m_rate_controller;
    private AttitudeController m_att_controller;

    public FlightController() {
        this("phone_imu", "command/thrust", "command/yawrate", "command/attitude_adjusted",
           "motor_ctrl", "update_pids");
    }

    public FlightController(String imu_topic, String thrust_topic, String yawrate_topic, 
        String attitude_topic, String motor_ctrl_topic, String pid_service) {
            /* Save topic names */
            this.m_imu_topic = imu_topic;
            this.m_thrust_topic = thrust_topic;
            this.m_yawrate_topic = yawrate_topic;
            this.m_att_topic = attitude_topic;
            this.m_motor_ctrl_topic = motor_ctrl_topic;
            this.m_pid_service = pid_service;

            /* Initialize the rate and attitude controllers */
            // Set default controller gains
            Vect3F roll_rate_pid_terms = new Vect3F(0.7, 0.0, 0.0);
            Vect3F pitch_rate_pid_terms = new Vect3F(0.7, 0.0, 0.0);
            Vect3F yaw_rate_pid_terms = new Vect3F(2.5, 0.0, 0.0);
            Vect3F roll_att_pid_terms = new Vect3F(4.5, 0.0, 0.0);
            Vect3F pitch_att_pid_terms = new Vect3F(4.5, 0.0, 0.0);

            // Initialize controllers
            m_rate_controller = new RateController(roll_rate_pid_terms, 
                    pitch_rate_pid_terms, yaw_rate_pid_terms);
            m_att_controller = new AttitudeController(roll_att_pid_terms,
                    pitch_att_pid_terms);
    }

    private double truncate(double value, double low, double high)
    {
        if (value < low)
            value = low;
        else if (value > high)
            value = high;
        return value;
    }

    private void applyThrustAdjustments(Vect3F thrust_adjustments)
    {
        double roll_thrust_adj, pitch_thrust_adj, yaw_thrust_adj;
        double motor1_thrust, motor2_thrust, motor3_thrust, motor4_thrust;

        simone_msgs.MotorCTRL motor_ctrl_msg = m_motor_publisher.newMessage();
        
        // Re-assign input vector to make it's values more comprehensive
        roll_thrust_adj  = thrust_adjustments.x;
        pitch_thrust_adj = thrust_adjustments.y;
        yaw_thrust_adj   = thrust_adjustments.z;
        
        // Calculate new thrusts
        motor4_thrust = m_thrust - roll_thrust_adj + pitch_thrust_adj + yaw_thrust_adj;
        motor2_thrust = m_thrust + roll_thrust_adj - pitch_thrust_adj + yaw_thrust_adj;
        motor3_thrust = m_thrust + roll_thrust_adj + pitch_thrust_adj - yaw_thrust_adj;
        motor1_thrust = m_thrust - roll_thrust_adj - pitch_thrust_adj - yaw_thrust_adj;

        // Make sure values do not exceed limits
        motor1_thrust = truncate(motor1_thrust, 0.0, 100.0);
        motor2_thrust = truncate(motor2_thrust, 0.0, 100.0);
        motor3_thrust = truncate(motor3_thrust, 0.0, 100.0);
        motor4_thrust = truncate(motor4_thrust, 0.0, 100.0);

        // Publish motor control message
        motor_ctrl_msg.setM1(motor1_thrust);
        motor_ctrl_msg.setM2(motor2_thrust);
        motor_ctrl_msg.setM3(motor3_thrust);
        motor_ctrl_msg.setM4(motor4_thrust);

        m_motor_publisher.publish(motor_ctrl_msg);
    }

    // This function is from wikipedia.
    private Vect3F quat2Euler(Quat q)
    {
        Vect3F rpy = new Vect3F();

        // Yaw (z-axis rotation)
        double siny = +2.0 * (-q.y * q.z + q.w * q.x);
        double cosy = -q.z * q.z + q.y * q.y - q.x * q.x + q.w * q.w; 
        rpy.z = -Math.atan2(siny, -cosy);

        // Roll (x-axis rotation)
        double sinr = +2.0 * (q.x * q.y + q.w * q.z);
        rpy.x = -Math.asin(sinr);

        // Pitch (y-axis rotation)
        double sinp = +2.0 * (-q.x * q.z + q.w * q.y);
        double cosp = - q.z * q.z - q.y * q.y + q.x * q.x + q.w * q.w;  
        rpy.y = -Math.atan2(sinp, cosp);

        return rpy;
    }

    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of("simone/flight_controller");
    }

    @Override
    public void onStart(ConnectedNode connectedNode) {
        // Subscribe to IMU topic
        Subscriber<sensor_msgs.Imu> imu_subscriber =
                connectedNode.newSubscriber(this.m_imu_topic, sensor_msgs.Imu._TYPE);
        // Create IMU topic callback
        imu_subscriber.addMessageListener(new MessageListener<sensor_msgs.Imu>() {
            @Override
            public void onNewMessage(sensor_msgs.Imu message){
                // Organize incomming sensor data in less verbose structures
                Vect3F rates = new Vect3F();
                rates.x = message.getAngularVelocity().getX();
                rates.y = message.getAngularVelocity().getY();
                rates.z = message.getAngularVelocity().getZ();
                
                Quat quat = new Quat();
                quat.w = message.getOrientation().getW();
                quat.x = message.getOrientation().getX();
                quat.y = message.getOrientation().getY();
                quat.z = message.getOrientation().getZ();

                Vect3F orientation = quat2Euler(quat);
                
                // Calculate stabalize angular velocity adjustments
                Vect3F rate_adjustments;
                rate_adjustments = m_att_controller.getOutput(orientation);
                rate_adjustments.z = m_yaw;

                // Calculate stabalize thrust adjustments
                Vect3F thrust_adjustments;
                thrust_adjustments = m_rate_controller.getOutput(rate_adjustments);

                // Send thrust adjustments to motors
                applyThrustAdjustments(thrust_adjustments);
            }
        });

        // Subscribe to thrust topic
        Subscriber<hector_uav_msgs.ThrustCommand> thrust_subscriber =
                connectedNode.newSubscriber(this.m_thrust_topic, hector_uav_msgs.ThrustCommand._TYPE);
        // Create thrust topic callback
        thrust_subscriber.addMessageListener(new MessageListener<hector_uav_msgs.ThrustCommand>() {
            @Override
            public void onNewMessage(hector_uav_msgs.ThrustCommand message){
                m_thrust = 60 - (message.getThrust() * 3);
            }
        });

        // Subscribe to yawrate topic
        Subscriber<hector_uav_msgs.YawrateCommand> yawrate_subscriber =
                connectedNode.newSubscriber(this.m_yawrate_topic, hector_uav_msgs.YawrateCommand._TYPE);
        // Create yawrate topic callback
        yawrate_subscriber.addMessageListener(new MessageListener<hector_uav_msgs.YawrateCommand>() {
            @Override
            public void onNewMessage(hector_uav_msgs.YawrateCommand message){
                // Get commanded value
                m_yaw = message.getTurnrate();

                // Build control inputs from current state
                Vect3F commanded_values = new Vect3F(m_pitch, m_roll, m_yaw); // formatted as phone's x, y, z axis

                // Update the controller with the new desired state
                m_att_controller.setDesiredAtt(commanded_values);
            }
        });

        // Subscribe to attitude topic
        Subscriber<hector_uav_msgs.AttitudeCommand> attitude_subscriber =
                connectedNode.newSubscriber(this.m_att_topic, hector_uav_msgs.AttitudeCommand._TYPE);
        // Create attitude topic callback
        attitude_subscriber.addMessageListener(new MessageListener<hector_uav_msgs.AttitudeCommand>() {
            @Override
            public void onNewMessage(hector_uav_msgs.AttitudeCommand message){
                // Get commanded value
                m_roll = message.getRoll();
                m_pitch = message.getPitch();

                // Build control inputs from current state
                Vect3F commanded_values = new Vect3F(m_pitch, m_roll, m_yaw); // formatted as phone's x, y, z axis

                // Update the controller with the new desired state
                m_att_controller.setDesiredAtt(commanded_values);
            }
        });

        // Set up motor control publisher
        m_motor_publisher = connectedNode.newPublisher(this.m_motor_ctrl_topic, simone_msgs.MotorCTRL._TYPE);

        // Set up update_pids server
        ServiceServer<UpdatePIDsRequest, UpdatePIDsResponse> server =
                connectedNode.newServiceServer(this.m_pid_service, UpdatePIDs._TYPE,
                        new ServiceResponseBuilder<UpdatePIDsRequest, UpdatePIDsResponse>() {
                            @Override
                            public void build(UpdatePIDsRequest request, UpdatePIDsResponse response)
                            {
                                Vect3F roll_rate_pid_terms = new Vect3F();
                                roll_rate_pid_terms.x = request.getRollRateKP();
                                roll_rate_pid_terms.y = request.getRollRateKI();
                                roll_rate_pid_terms.z = request.getRollRateKD();

                                Vect3F pitch_rate_pid_terms = new Vect3F();
                                pitch_rate_pid_terms.x = request.getPitchRateKP();
                                pitch_rate_pid_terms.y = request.getPitchRateKI();
                                pitch_rate_pid_terms.z = request.getPitchRateKD();

                                Vect3F yaw_rate_pid_terms = new Vect3F();
                                yaw_rate_pid_terms.x = request.getYawRateKP();
                                yaw_rate_pid_terms.y = request.getYawRateKI();
                                yaw_rate_pid_terms.z = request.getYawRateKD();
                                
                                // Re-load parameters for attitude PIDs
                                Vect3F roll_att_pid_terms = new Vect3F();
                                roll_att_pid_terms.x = request.getRollAttKP();
                                roll_att_pid_terms.y = request.getRollAttKI();
                                roll_att_pid_terms.z = request.getRollAttKD();

                                Vect3F pitch_att_pid_terms = new Vect3F();
                                pitch_att_pid_terms.x = request.getPitchAttKP();
                                pitch_att_pid_terms.y = request.getPitchAttKI();
                                pitch_att_pid_terms.z = request.getPitchAttKD();

                                // Re-start PID controllers
                                m_rate_controller.reset(roll_rate_pid_terms, pitch_rate_pid_terms, yaw_rate_pid_terms);
                                m_att_controller.reset(roll_att_pid_terms, pitch_att_pid_terms);

                                response.setSuccess(true);
                            }
                        });

    }

} // End class

