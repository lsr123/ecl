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
 * @file ecl_pitch_controller.h
 * Definition of a simple orthogonal pitch PID controller.
 *
 * @author Lorenz Meier <lm@inf.ethz.ch>
 * @author Thomas Gubler <thomasgubler@gmail.com>
 *
 * Acknowledgements:
 *
 *   The control design is based on a design
 *   by Paul Riseborough and Andrew Tridgell, 2013,
 *   which in turn is based on initial work of
 *   Jonathan Challinger, 2012.
 */

#ifndef ECL_PITCH_CONTROLLER_H
#define ECL_PITCH_CONTROLLER_H

#include <stdbool.h>
#include <stdint.h>

#include "ecl_controller.h"

class __EXPORT ECL_PitchController :
	public ECL_Controller
{
public:
	ECL_PitchController();
	~ECL_PitchController() = default;

	float control_attitude(const struct ECL_ControlData &ctl_data);
	float control_euler_rate(const struct ECL_ControlData &ctl_data);
	float control_bodyrate(const struct ECL_ControlData &ctl_data);
	float ADRC_control(const struct ECL_ControlData &ctl_data, float a);
	float signal_PID_control(const struct ECL_ControlData &ctl_data);
	

	void P_LESO(const struct ECL_ControlData &ctl_data, float P_U, float dtime);
	void P_NESO(const struct ECL_ControlData &ctl_data, float P_U, float dtime);
	void P_TD(const struct ECL_ControlData &ctl_data, float dtime);
	float P_ADRC_PD(void);
	float P_ADRC_NLF(float dtime);
	float P_fhan(float x1,float x2,float r,float h);
	float P_fal(float e,float alpha,float delta);
	float sign(float x);

	/* Additional Setters */
	void set_max_rate_pos(float max_rate_pos)
	{
		_max_rate = max_rate_pos;
	}

	void set_max_rate_neg(float max_rate_neg)
	{
		_max_rate_neg = max_rate_neg;
	}

	void set_roll_ff(float roll_ff)
	{
		_roll_ff = roll_ff;
	}

protected:
	float _max_rate_neg;
	float _roll_ff;
};

#endif // ECL_PITCH_CONTROLLER_H
