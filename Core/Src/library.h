//Funtion Cascade Position And Velocity Controller
float PositionController(float r,float y) //r == trajectory, y==feedback
{
	e1 = r - y;
	s1 = s1 + e1;
	u1 = kp_1*e1 + ki_1*s1 + kd_1*(e1-p1);
	p1 = e1;
	return u1;
}

float VelocityController(float r,float y,float uP)
{
	e2 = uP + r;
	if(e2 >= w_max){
		e2 = w_max;
	}
	e2 = e2 - y;
	s2 = s2 + e2;
	u2 = kp_2*e2 + ki_2*s2;
	return u2;
}

float Cascade(float Pd,float P,float Vd,float V){
	static float u;
	float add = 2;
	u = PositionController(Pd, P);
	u = VelocityController(Vd, V, u);
	PID_dir = 1;
	if(u > 24){
		u = 24;
	}
	else if (u < 0){
		u = -(u);
		PID_dir = 0;
	}

	if(t >= t5)
	{
		add = 0;
	}
    
	return u+add;
}

//Funtion End Effector
void OpenEndEffector(&FlagOpen_EndEffector, &EndEffector_State) {
	if (hi2c1.State == HAL_I2C_STATE_READY && FlagOpen_EndEffector == 1)
	{
		static uint8_t addr_open = 0x45;
		HAL_I2C_Master_Transmit(&hi2c1, ENDEFF_ADDR, &addr_open, 1,100);
		FlagOpen_EndEffector = 0;
		FlagRead_EndEffector = 1;
		EndEffector_State = State_start;
		EndEffector_timestamp = HAL_GetTick();
	}

	if (FlagRead_EndEffector == 1)
	{
		if (HAL_GetTick() - EndEffector_timestamp > 250)
		{
			EndEffector_timestamp = HAL_GetTick();
			CheckEndEffector();
		}

		switch(EndEffector_State)
		{
		case State_start:
			if (EndEffector_Status == 0x12)
			{
				EndEffector_State = State_open;
			}
			break;
		case State_open:
			if (EndEffector_Status == 0x34)
			{
				EndEffector_State = State_shoot;
			}
			break;
		case State_shoot:
			if (EndEffector_Status == 0x56)
			{
				EndEffector_State = State_close;
			}
			break;
		case State_close:
			if (EndEffector_Status == 0x78)
			{
				EndEffector_State = State_wait;
			}
			break;
		case State_wait:
			FlagRead_EndEffector = 0;
			break;
		}
	}
}

void CheckEndEffector(EndEffector_Status)
{
	static uint8_t addr = 0x23;
	HAL_I2C_Master_Seq_Transmit_IT(&hi2c1, ENDEFF_ADDR, &addr, 1, I2C_FIRST_FRAME);
	if(hi2c1.State == HAL_I2C_STATE_READY)
	{
		HAL_I2C_Master_Seq_Receive_IT(&hi2c1, ENDEFF_ADDR, &EndEffector_Status, 1, I2C_LAST_FRAME);
	}
}