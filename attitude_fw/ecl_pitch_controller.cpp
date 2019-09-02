/****************************************************************************
 *
 *   Copyright (c) 2013 Estimation and Control Library (ECL). All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name ECL nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file ecl_pitch_controller.cpp
 * Implementation of a simple orthogonal pitch PID controller.
 *
 * Authors and acknowledgements in header.
 */
#include <ecl/ecl.h>
#include <geo/geo.h>

#include "ecl_pitch_controller.h"
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <float.h>
#include <geo/geo.h>
#include <ecl/ecl.h>
#include <mathlib/mathlib.h>
#include <systemlib/err.h>
#include <uORB/topics/ADRC.h>

float ADRC_data[5] = {0.0f};

ECL_PitchController::ECL_PitchController() :
	ECL_Controller("pitch"),
	_max_rate_neg(0.0f),
	_roll_ff(0.0f)
{
}

float ECL_PitchController::control_attitude(const struct ECL_ControlData &ctl_data)
{

	/* Do not calculate control signal with bad inputs */
	if (!(PX4_ISFINITE(ctl_data.pitch_setpoint) &&
	      PX4_ISFINITE(ctl_data.roll) &&
	      PX4_ISFINITE(ctl_data.pitch) &&
	      PX4_ISFINITE(ctl_data.airspeed))) {
		warnx("not controlling pitch");
		return _rate_setpoint;
	}

	/* Calculate the error */
	float pitch_error = ctl_data.pitch_setpoint - ctl_data.pitch;

	/*  Apply P controller: rate setpoint from current error and time constant */
	_rate_setpoint =  pitch_error / _tc;

	/* limit the rate */
	if (_max_rate > 0.01f && _max_rate_neg > 0.01f) {
		if (_rate_setpoint > 0.0f) {
			_rate_setpoint = (_rate_setpoint > _max_rate) ? _max_rate : _rate_setpoint;

		} else {
			_rate_setpoint = (_rate_setpoint < -_max_rate_neg) ? -_max_rate_neg : _rate_setpoint;
		}

	}

	return _rate_setpoint;
}

float ECL_PitchController::control_bodyrate(const struct ECL_ControlData &ctl_data)
{
	/* Do not calculate control signal with bad inputs */
	if (!(PX4_ISFINITE(ctl_data.roll) &&
	      PX4_ISFINITE(ctl_data.pitch) &&
	      PX4_ISFINITE(ctl_data.body_y_rate) &&
	      PX4_ISFINITE(ctl_data.body_z_rate) &&
	      PX4_ISFINITE(ctl_data.yaw_rate_setpoint) &&
	      PX4_ISFINITE(ctl_data.airspeed_min) &&
	      PX4_ISFINITE(ctl_data.airspeed_max) &&
	      PX4_ISFINITE(ctl_data.scaler))) {
		return math::constrain(_last_output, -1.0f, 1.0f);
	}

	/* get the usual dt estimate */
	uint64_t dt_micros = ecl_elapsed_time(&_last_run);
	_last_run = ecl_absolute_time();
	float dt = (float)dt_micros * 1e-6f;

	/* lock integral for long intervals */
	bool lock_integrator = ctl_data.lock_integrator;

	if (dt_micros > 500000) {
		lock_integrator = true;
	}

	_rate_error = _bodyrate_setpoint - ctl_data.body_y_rate;

	if (!lock_integrator && _k_i > 0.0f) {

		float id = _rate_error * dt * ctl_data.scaler;

		/*
		 * anti-windup: do not allow integrator to increase if actuator is at limit
		 */
		if (_last_output < -1.0f) {
			/* only allow motion to center: increase value */
			id = math::max(id, 0.0f);

		} else if (_last_output > 1.0f) {
			/* only allow motion to center: decrease value */
			id = math::min(id, 0.0f);
		}

		_integrator += id * _k_i;
	}

	/* integrator limit */
	//xxx: until start detection is available: integral part in control signal is limited here
	float integrator_constrained = math::constrain(_integrator, -_integrator_max, _integrator_max);

	/* Apply PI rate controller and store non-limited output */
	_last_output = _bodyrate_setpoint * _k_ff * ctl_data.scaler +
		       _rate_error * _k_p * ctl_data.scaler * ctl_data.scaler
		       + integrator_constrained;  //scaler is proportional to 1/airspeed

	return math::constrain(_last_output, -1.0f, 1.0f);
}

float ECL_PitchController::control_euler_rate(const struct ECL_ControlData &ctl_data)
{
	/* Transform setpoint to body angular rates (jacobian) */
	_bodyrate_setpoint = cosf(ctl_data.roll) * _rate_setpoint +
			     cosf(ctl_data.pitch) * sinf(ctl_data.roll) * ctl_data.yaw_rate_setpoint;

	return control_bodyrate(ctl_data);
}

float ECL_PitchController::ADRC_control(const struct ECL_ControlData &ctl_data, float P_U)
{
	const hrt_abstime now = hrt_absolute_time();
	float u = 0.0f;
	struct ADRC_s ADRC_log;
	static uint64_t last_run = 0;
	float deltaT = (hrt_absolute_time() - last_run) / 1000000.0f;

	last_run = hrt_absolute_time();

	//if(deltaT>0.5f)
	//	deltaT = 0.0125f;
	//sleep(2);
	//printf(" deltaT = %8.6f\n",(double)deltaT );
	//不接遥控器时，指令值为0,,there is no outer loop
	//printf(" pitch = %8.6f\n",(double)ctl_data.pitch);

	P_TD(ctl_data, deltaT);   //得到目标指令的值和微分值   存储到ADRC_data[0]和[1]中
	//P_LESO(ctl_data,P_U, deltaT);  //得到系统输出的值和微分值   存储到ADRC_data[2]和[3]中
	P_NESO(ctl_data,P_U, deltaT);

	//u = P_ADRC_PD();           //直接进行PD控制
	u = P_ADRC_NLF(deltaT);            //进行非线性反馈控制
	//printf("%f\n",(double)ADRC_data[0] );
	//printf("u = %f\n",(double)deltaT );

	memset(&ADRC_log, 0, sizeof(ADRC_log));
	orb_advert_t ADRC_sub = orb_advertise(ORB_ID(ADRC), &ADRC_log);   //公告主题 test_pub为指针
	//ADRC_log.x1 = 0.88;
	ADRC_log.timestamp = now;

	ADRC_log.x1 = ADRC_data[0];
	ADRC_log.x2 = ADRC_data[1];
	ADRC_log.z1 = ADRC_data[2];
	ADRC_log.z2 = ADRC_data[3];
	ADRC_log.z3 = ADRC_data[4];
	//ADRC_log.z1_roll = 10.0f;
	orb_publish(ORB_ID(ADRC), ADRC_sub, &ADRC_log);      //发布数据
	
	//warnx("ADRC_data[0] = %.4f,ADRC_data[2] = %.4f",(double)ADRC_log.x1,(double)ADRC_log.z1);

	return math::constrain(u, -1.0f, 1.0f);
}
float ECL_PitchController::P_ADRC_PD()
{
	//printf("%f\n",(double)_p);
	//printf("%f\n",(double)_d);
	float u = 0.0f;
	u = _p*(ADRC_data[0]-ADRC_data[2])+_d*(ADRC_data[1]-ADRC_data[3]) - ADRC_data[4]/_adrc_b0;
	//u = _p*(ADRC_data[0]-ADRC_data[2])+_d*(ADRC_data[1]-ADRC_data[3]) ;
	return u;
	//return math::constrain(u, -1.0f, 1.0f);
}

float ECL_PitchController::P_ADRC_NLF(float dtime)
{
	float nlf_u0 = 0.0f;    //非线性反馈的输出控制量
	float nlf_u = 0.0f;

	//nlf_u = (_adrc_lamda * (ADRC_data[0] - ADRC_data[2]) + _adrc_alpha * P_fal(ADRC_data[1] - ADRC_data[3]) - ADRC_data[4]) / _adrc_b0;
	//nlf_u = (_adrc_lamda * (ADRC_data[0] - ADRC_data[2]) + _adrc_alpha * P_fal(ADRC_data[1] - ADRC_data[3]) - ADRC_data[4]/100 ) / _adrc_b0;
	//nlf_u = (_adrc_lamda * (ADRC_data[0] - ADRC_data[2]) + _adrc_alpha * (ADRC_data[1] - ADRC_data[3]) - ADRC_data[4]) / _adrc_b0;
	float err1 = ADRC_data[0] - ADRC_data[2];
	float err2 = ADRC_data[1] - ADRC_data[3];
	nlf_u0 = _adrc_lamda * P_fal(err1,0.5,dtime) + _adrc_alpha * P_fal(err2,1.2,dtime);
	nlf_u = nlf_u0 - ADRC_data[4]/_adrc_b0;
	//nlf_u = nlf_u0 ;
	return nlf_u;
}

void ECL_PitchController::P_LESO(const struct ECL_ControlData &ctl_data,float P_U, float dtime)
{
	//printf("aaaaaaa");
	static float z1 = 0.0f;
	static float z2 = 0.0f;
	static float z3 = 0.0f;
	float L1 = 0.0f;
	float L2 = 0.0f;
	float L3 = 0.0f;
	float e1 = 0.0f;

	L1 = _bd*3;
	L2 = _bd*_bd*3;
	L3 = _bd*_bd*_bd*1;

	e1 = z1 - ctl_data.pitch;
	z1 = z1+dtime*(z2-L1*e1);
	z2 = z2+dtime*(z3-L2*e1+_adrc_b0*P_U);
	z3 = z3+dtime*(-L3*e1);
	ADRC_data[2] = z1;
	ADRC_data[3] = z2;
	ADRC_data[4] = z3;

}
void ECL_PitchController::P_NESO(const struct ECL_ControlData &ctl_data, float P_U, float dtime)
{	
	static float N_z1 = 0.0f;
	static float N_z2 = 0.0f;
	static float N_z3 = 0.0f;

	float err = 0.0f;
	float fe = 0.0f;
	float fe1 = 0.0f;

	err = N_z1 - ctl_data.pitch;
	fe = P_fal(err,0.5,dtime);
	fe1 = P_fal(err,0.25,dtime);

	N_z1 = N_z1 + dtime*(N_z2 - _adrc_bt1 * err );
	N_z2 = N_z2 + dtime*(N_z3 - _adrc_bt2 * fe + _adrc_b0 * P_U);
	N_z3 = N_z3 + dtime*(-_adrc_bt3 * fe1);
	ADRC_data[2] = N_z1;
	ADRC_data[3] = N_z2;
	ADRC_data[4] = N_z3;

}
void ECL_PitchController::P_TD(const struct ECL_ControlData &ctl_data, float dtime)
{
	static float x1 = 0.0f;   //实际值的估计值
	static float x2 = 0.0f;   //实际值的微分

	float fh = 0.0f;
	fh = P_fhan(x1-ctl_data.pitch_setpoint, x2, _r, _s*dtime);
	x1 = x1+dtime*x2;
	x2 = x2+dtime*fh;
	ADRC_data[0] = x1;
	ADRC_data[1] = x2;
	//warnx("ADRC_data[0] = %.8f\n", (double)ADRC_data[0]);


}

float ECL_PitchController::signal_PID_control(const struct ECL_ControlData &ctl_data)
{
	
	static float err = 0.0f;

	float outputsignal = 0.0f;
	//float _signalkp = 0.1;
	//float _signalkd = 0.1;

	/* Do not calculate control signal with bad inputs */
	if (!(PX4_ISFINITE(ctl_data.pitch_setpoint) &&
	      PX4_ISFINITE(ctl_data.roll) &&
	      PX4_ISFINITE(ctl_data.pitch) &&
	      PX4_ISFINITE(ctl_data.airspeed))) {
		warnx("not controlling pitch");
		return _rate_setpoint;
	}

	/* get the usual dt estimate */
	uint64_t dt_micros = ecl_elapsed_time(&_last_run);
	_last_run = ecl_absolute_time();
	float dt = (float)dt_micros * 1e-6f;

	/* Calculate the error */
	float pitch_error = ctl_data.pitch_setpoint - ctl_data.pitch;
	float id = pitch_error*dt;
	//还没有写积分

	outputsignal = _signal_p*pitch_error +  _signal_d*(err - pitch_error) + 0*id;
	err = pitch_error;

	return math::constrain(outputsignal, -1.0f, 1.0f);

}




float ECL_PitchController::P_fhan(float x1,float x2,float r,float h)
{
	float d = 0.0f;
	float d0 = 0.0f;
	float y = 0.0f;
	float a0 = 0.0f;
	float a = 0.0f;
	float u = 0.0f;

	d = r*h;
	d0 = h*d;
	y = x1+h*x2;
	a0 = sqrt(d*d+8*r*abs(y));
	if(abs(y)>d0)
		a = x2+(a0-d)/2*sign(y);
	else
		a = x2+y/h;

	if(abs(a)>d)
		u = -r*sign(a);
	else
		u = -r*a/d;

	return u;
}

float ECL_PitchController::P_fal(float e,float alpha,float delta)
{	//默认fal函数
	float out = 0.0f;
	
	if(abs(e)>delta)
		out = (float)pow(abs(e),alpha)*sign(e);
		
	else out = (float)e/(float)(pow(delta,(1-alpha)));

	return out;
}
float ECL_PitchController::sign(float x)
{
	float a = 0.0f;
	if(abs(x) < 0.00001)
		a  = 0.0f;
	else if(x>0)
		a = 1.0f;
	else if(x<0)
		a = -1.0f;
	
	return a;

}