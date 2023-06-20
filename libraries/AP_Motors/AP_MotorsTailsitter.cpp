/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 *       AP_MotorsTailsitter.cpp - ArduCopter motors library for tailsitters and bicopters
 *
 */

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include "AP_MotorsTailsitter.h"
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

#define SERVO_OUTPUT_RANGE  4500

// init
void AP_MotorsTailsitter::init(motor_frame_class frame_class, motor_frame_type frame_type)  //初始化尾椎式飞行器的电机和舵机
{
    // setup default motor and servo mappings
    uint8_t chan;

    // right throttle defaults to servo output 1
    SRV_Channels::set_aux_channel_default(SRV_Channel::k_throttleRight, CH_1);  //设置右侧油门电机的默认辅助通道功能为k_throttleRight（右侧油门），通道号为CH_1
    if (SRV_Channels::find_channel(SRV_Channel::k_throttleRight, chan)) {       //查找已分配给右侧油门功能的通道号，并将其保存在变量chan中
        motor_enabled[chan] = true;                                             //将找到的右侧油门通道对应的motor_enabled数组元素设置为true，表示该电机已启用
    }

    // left throttle defaults to servo output 2
    SRV_Channels::set_aux_channel_default(SRV_Channel::k_throttleLeft, CH_2);  //设置左侧油门电机的默认辅助通道功能为k_throttleLeft（左侧油门），通道号为CH_2
    if (SRV_Channels::find_channel(SRV_Channel::k_throttleLeft, chan)) {       //查找已分配给左侧油门功能的通道号，并将其保存在变量chan中
        motor_enabled[chan] = true;                                            //将找到的左侧油门通道对应的motor_enabled数组元素设置为true，表示该电机已启用
    }

    // right servo defaults to servo output 3
    SRV_Channels::set_aux_channel_default(SRV_Channel::k_tiltMotorRight, CH_3);  //设置右侧俯仰伺服舵机的默认辅助通道功能为k_tiltMotorRight（右侧俯仰伺服舵机）。通道号为 CH_3，表示将其连接到舵机输出通道3上。
    SRV_Channels::set_angle(SRV_Channel::k_tiltMotorRight, SERVO_OUTPUT_RANGE);  //将右侧俯仰伺服舵机的角度范围设置为SERVO_OUTPUT_RANGE。这个函数用于设置伺服舵机的角度范围，确保其适应特定的飞行器需求。

    // left servo defaults to servo output 4
    SRV_Channels::set_aux_channel_default(SRV_Channel::k_tiltMotorLeft, CH_4);  //设置左侧俯仰伺服舵机的默认辅助通道功能为k_tiltMotorLeft（左侧俯仰伺服舵机）。通道号为 CH_4，表示将其连接到舵机输出通道4上。
    SRV_Channels::set_angle(SRV_Channel::k_tiltMotorLeft, SERVO_OUTPUT_RANGE);  //将左侧俯仰伺服舵机的角度范围设置为SERVO_OUTPUT_RANGE。这个函数用于设置伺服舵机的角度范围，确保其适应特定的飞行器需求。

    //rightjoint servo defaults to servo output 5
    SRV_Channels::set_aux_channel_default(SRV_Channel::k_tiltMotorRightJoint, CH_5);  //设置右侧关节俯仰伺服舵机的默认辅助通道功能为k_tiltMotorRightJoint（右侧关节俯仰伺服舵机）。通道号为 CH_5，表示将其连接到舵机输出通道5上。
    SRV_Channels::set_angle(SRV_Channel::k_tiltMotorRightJoint, SERVO_OUTPUT_RANGE);  //将右侧关节俯仰伺服舵机的角度范围设置为SERVO_OUTPUT_RANGE。这个函数用于设置伺服舵机的角度范围，确保其适应特定的飞行器需求。

    //leftjoint servo defaults to servo output 6
    SRV_Channels::set_aux_channel_default(SRV_Channel::k_tiltMotorLeftJoint,CH_6);  //设置左侧关节俯仰伺服舵机的默认辅助通道功能为k_tiltMotorLeftJoint（左侧关节俯仰伺服舵机）。通道号为 CH_6，表示将其连接到舵机输出通道6上。
    SRV_Channels::set_angle(SRV_Channel::k_tiltMotorLeftJoint,SERVO_OUTPUT_RANGE);  //将左侧关节俯仰伺服舵机的角度范围设置为SERVO_OUTPUT_RANGE。这个函数用于设置伺服舵机的角度范围，确保其适应特定的飞行器需求。

    //rightwheel servo defaults to servo output 7
    SRV_Channels::set_aux_channel_default(SRV_Channel::k_speedMotorRightWheel, CH_7);  //设置右侧足部电机的默认辅助通道功能为k_speedMotorRightWheel（右侧足部电机）。通道号为 CH_7，表示将其连接到舵机输出通道7上。
    SRV_Channels::set_angle(SRV_Channel::k_speedMotorRightWheel, SERVO_OUTPUT_RANGE);  //将右侧足部电机的角度范围设置为SERVO_OUTPUT_RANGE。这个函数用于设置电机的角度范围，确保其适应特定的飞行器需求。

    //leftwheel servo defaults to servo output 8
    SRV_Channels::set_aux_channel_default(SRV_Channel::k_speedMotorLeftWheel, CH_8);  //设置左侧足部电机的默认辅助通道功能为k_speedMotorLeftWheel（右侧足部电机）。通道号为 CH_8，表示将其连接到舵机输出通道8上。
    SRV_Channels::set_angle(SRV_Channel::k_speedMotorLeftWheel, SERVO_OUTPUT_RANGE);  //将左侧足部电机的角度范围设置为SERVO_OUTPUT_RANGE。这个函数用于设置电机的角度范围，确保其适应特定的飞行器需求。
    
    _mav_type = MAV_TYPE_COAXIAL;

    // record successful initialisation if what we setup was the desired frame_class
    set_initialised_ok(frame_class == MOTOR_FRAME_TAILSITTER);
}


/// Constructor
AP_MotorsTailsitter::AP_MotorsTailsitter(uint16_t loop_rate, uint16_t speed_hz) :
    AP_MotorsMulticopter(loop_rate, speed_hz)
{
    set_update_rate(speed_hz);
}


// set update rate to motors - a value in hertz
void AP_MotorsTailsitter::set_update_rate(uint16_t speed_hz)
{
    // record requested speed
    _speed_hz = speed_hz;

    SRV_Channels::set_rc_frequency(SRV_Channel::k_throttleLeft, speed_hz);
    SRV_Channels::set_rc_frequency(SRV_Channel::k_throttleRight, speed_hz);
}

void AP_MotorsTailsitter::output_to_motors()
{
    if (!initialised_ok()) {
        return;
    }

    switch (_spool_state) {
        case SpoolState::SHUT_DOWN:
            _actuator[0] = 0.0f;
            _actuator[1] = 0.0f;
            _actuator[2] = 0.0f;
            break;
        case SpoolState::GROUND_IDLE:
            set_actuator_with_slew(_actuator[0], actuator_spin_up_to_ground_idle());
            set_actuator_with_slew(_actuator[1], actuator_spin_up_to_ground_idle());
            set_actuator_with_slew(_actuator[2], actuator_spin_up_to_ground_idle());
            break;
        case SpoolState::SPOOLING_UP:
        case SpoolState::THROTTLE_UNLIMITED:
        case SpoolState::SPOOLING_DOWN:
            set_actuator_with_slew(_actuator[0], thrust_to_actuator(_thrust_left));
            set_actuator_with_slew(_actuator[1], thrust_to_actuator(_thrust_right));
            set_actuator_with_slew(_actuator[2], thrust_to_actuator(_throttle));
            break;
    }

    SRV_Channels::set_output_pwm(SRV_Channel::k_throttleLeft, output_to_pwm(_actuator[0]));   //将_actuator[0]的输出值转换为PWM信号，并将其设置为左侧油门的输出值
    SRV_Channels::set_output_pwm(SRV_Channel::k_throttleRight, output_to_pwm(_actuator[1]));  //将_actuator[1]的输出值转换为PWM信号，并将其设置为右侧油门的输出值

    // use set scaled to allow a different PWM range on plane forward throttle, throttle range is 0 to 100
    SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, _actuator[2]*100);

    SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorLeft, _tilt_left*SERVO_OUTPUT_RANGE);
    SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRight, _tilt_right*SERVO_OUTPUT_RANGE);
    
    SRV_Channels::set_output_scaled(SRV_Channel::k_speedMotorLeftWheel, _speed_leftwheel*SERVO_OUTPUT_RANGE);    //设置左侧轮速电机的输出值
    SRV_Channels::set_output_scaled(SRV_Channel::k_speedMotorRightWheel, _speed_rightwheel*SERVO_OUTPUT_RANGE);  //设置右侧轮速电机的输出值

    SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorLeftJoint,_tilt_leftjoint*SERVO_OUTPUT_RANGE);    //设置左侧关节俯仰伺服舵机通道的输出值
    SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRightJoint,_tilt_rightjoint*SERVO_OUTPUT_RANGE);  //设置右侧关节俯仰伺服舵机通道的输出值

}

// get_motor_mask - returns a bitmask of which outputs are being used for motors (1 means being used)
//  this can be used to ensure other pwm outputs (i.e. for servos) do not conflict
uint16_t AP_MotorsTailsitter::get_motor_mask()
{
    uint32_t motor_mask = 0;
    uint8_t chan;
    if (SRV_Channels::find_channel(SRV_Channel::k_throttleLeft, chan)) {
        motor_mask |= 1U << chan;
    }
    if (SRV_Channels::find_channel(SRV_Channel::k_throttleRight, chan)) {
        motor_mask |= 1U << chan;
    }

    // add parent's mask
    motor_mask |= AP_MotorsMulticopter::get_motor_mask();

    return motor_mask;
}

// calculate outputs to the motors
void AP_MotorsTailsitter::output_armed_stabilizing()
{
    float   roll_thrust;                // roll thrust input value, +/- 1.0
    float   pitch_thrust;               // pitch thrust input value, +/- 1.0
    float   yaw_thrust;                 // yaw thrust input value, +/- 1.0
    float   throttle_thrust;            // throttle thrust input value, 0.0 - 1.0
    float   thrust_max;                 // highest motor value
    float   thr_adj = 0.0f;             // the difference between the pilot's desired throttle and throttle_thrust_best_rpy

    // apply voltage and air pressure compensation
    const float compensation_gain = get_compensation_gain();
    roll_thrust = (_roll_in + _roll_in_ff) * compensation_gain;
    pitch_thrust = _pitch_in + _pitch_in_ff;
    yaw_thrust = _yaw_in + _yaw_in_ff;
    throttle_thrust = get_throttle() * compensation_gain;

    // sanity check throttle is above zero and below current limited throttle
    if (throttle_thrust <= 0.0f) {
        throttle_thrust = 0.0f;
        limit.throttle_lower = true;
    }
    if (throttle_thrust >= _throttle_thrust_max) {
        throttle_thrust = _throttle_thrust_max;
        limit.throttle_upper = true;
    }

    // calculate left and right throttle outputs
    _thrust_left  = throttle_thrust + roll_thrust * 0.5f;
    _thrust_right = throttle_thrust - roll_thrust * 0.5f;

    // if max thrust is more than one reduce average throttle
    thrust_max = MAX(_thrust_right,_thrust_left);
    if (thrust_max > 1.0f) {
        thr_adj = 1.0f - thrust_max;
        limit.throttle_upper = true;
        limit.roll = true;
        limit.pitch = true;
    }

    // Add adjustment to reduce average throttle
    _thrust_left  = constrain_float(_thrust_left  + thr_adj, 0.0f, 1.0f);
    _thrust_right = constrain_float(_thrust_right + thr_adj, 0.0f, 1.0f);
    _throttle = throttle_thrust + thr_adj;
    // compensation_gain can never be zero
    _throttle_out = _throttle / compensation_gain;

    // thrust vectoring
    _tilt_left  = pitch_thrust - yaw_thrust;
    _tilt_right = pitch_thrust + yaw_thrust;
    _speed_leftwheel = pitch_thrust * 0.5f - yaw_thrust * 0.5f;  //添加左右两个足部电机
    _speed_rightwheel = pitch_thrust * 0.5f + yaw_thrust * 0.5f;
    _tilt_leftjoint = pitch_thrust * 0.5f - yaw_thrust * 0.5f;  //添加左右两个关节舵机
    _tilt_rightjoint = pitch_thrust * 0.5f + yaw_thrust * 0.5f;

}

// output_test_seq - spin a motor at the pwm value specified
//  motor_seq is the motor's sequence number from 1 to the number of motors on the frame
//  pwm value is an actual pwm value that will be output, normally in the range of 1000 ~ 2000
void AP_MotorsTailsitter::output_test_seq(uint8_t motor_seq, int16_t pwm)
{
    // exit immediately if not armed
    if (!armed()) {
        return;
    }

    // output to motors and servos
    switch (motor_seq) {
        case 1:
            // right throttle
            SRV_Channels::set_output_pwm(SRV_Channel::k_throttleRight, pwm);
            break;
        case 2:
            // right tilt servo
            SRV_Channels::set_output_pwm(SRV_Channel::k_tiltMotorRight, pwm);
            break;
        case 3:
            // left throttle
            SRV_Channels::set_output_pwm(SRV_Channel::k_throttleLeft, pwm);
            break;
        case 4:
            // left tilt servo
            SRV_Channels::set_output_pwm(SRV_Channel::k_tiltMotorLeft, pwm);
            break;
        case 5:
            //right joint tilt servo
            SRV_Channels::set_output_pwm(SRV_Channel::k_tiltMotorRightJoint, pwm);  //将pwm设置为右侧关节俯仰伺服舵机通道的输出值
            break;
        case 6:
            //right wheel speed servo
            SRV_Channels::set_output_pwm(SRV_Channel::k_speedMotorRightWheel, pwm);  //将pwm设置为右侧轮速电机通道的输出值
            break;
        case 7:
            //left joint tilt servo
            SRV_Channels::set_output_pwm(SRV_Channel::k_tiltMotorLeftJoint, pwm);  ////将pwm设置为左侧关节俯仰伺服舵机通道的输出值
            break; 
        case 8:
            //left wheel speed servo
            SRV_Channels::set_output_pwm(SRV_Channel::k_speedMotorLeftWheel, pwm);  //将pwm设置为左侧轮速电机通道的输出值
            break;
        default:
            // do nothing
            break;
    }
}
