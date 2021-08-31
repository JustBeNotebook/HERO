





		在初始化里面先调用  BMI 初始化函数     
			BMI_Init()

		在任务系统里面调用函数读取加速度 角速度  姿态结算 
			BMI_Get_ACC(&accx,&accy,&accz);  //读取加速度
			BMI_Get_GRO(&gyrox,&gyroy,&gyroz);//读取角速度
			BMI_Get_data(&roll,&pitch,&yaw,&gyrox,&gyroy,&gyroz,&accx,&accy,&accz);//读取欧拉角




















