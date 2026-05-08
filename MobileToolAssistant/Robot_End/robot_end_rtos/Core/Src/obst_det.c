///*
// * obst_det.c
// *
// *  Created on: Feb 17, 2026
// *      Author: Chidubem Emeka-Nwuba
// */
//


#include "obst_det.h"
#include "cmsis_os.h"
#include "motor_hw.h"


/* ================= Sensor Storage ================= */

ToF_Data_t ToF_DATA[TOF_SENSOR_COUNT];


/* ================= Address Array ================= */

const uint16_t tofAddresses[TOF_SENSOR_COUNT] =
{
    TOF_L_ADDR,
    TOF_C_ADDR,
    TOF_R_ADDR
};


/* ================= XSHUT Arrays ================= */

static GPIO_TypeDef* xshutPorts[TOF_SENSOR_COUNT] =
{
    XSHUT_L_PORT,
    XSHUT_C_PORT,
    XSHUT_R_PORT
};

static const uint16_t xshutPins[TOF_SENSOR_COUNT] =
{
    XSHUT_L_PIN,
    XSHUT_C_PIN,
    XSHUT_R_PIN
};

/* ================= Dodge Command Variables ================= */

static uint16_t filtered_left = 0;
static uint16_t filtered_center = 0;
static uint16_t filtered_right = 0;

/* ================= Reset Data ================= */

void ToF_ResetData(ToF_Data_t *sensor)
{
    sensor->distance = 0;
    sensor->sensorState = 0;
    sensor->dataReady = 0;
    sensor->rangeStatus = 0;
}


/* ================= Assign Addresses ================= */

void ToF_AssignAddresses(void)
{
    /* Reset all sensors */

    for(int i = 0; i < TOF_SENSOR_COUNT; i++)
        HAL_GPIO_WritePin(xshutPorts[i], xshutPins[i], GPIO_PIN_RESET);

    osDelay(50);

    /* Enable and assign address sequentially */

    for(int i = 0; i < TOF_SENSOR_COUNT; i++)
    {
        HAL_GPIO_WritePin(xshutPorts[i], xshutPins[i], GPIO_PIN_SET);

        osDelay(50);

        VL53L1X_SetI2CAddress(TOF_DEFAULT_ADDR, tofAddresses[i]);
    }
}



/* ================= Sensor Init ================= */

int ToF_InitSensor(uint16_t dev, ToF_Data_t *sensor)
{
    int status;

    sensor->sensorState = 0;

    while(sensor->sensorState == 0)
    {
        status = VL53L1X_BootState(dev, &sensor->sensorState);
        osDelay(2);
    }

    status = VL53L1X_SensorInit(dev);
    status = VL53L1X_SetDistanceMode(dev, 2); // Long range sensing
    status = VL53L1X_SetTimingBudgetInMs(dev, 200);
    status = VL53L1X_SetInterMeasurementInMs(dev, 200);
    status = VL53L1X_StartRanging(dev);

    return status;
}



/* ================= Update All Sensors ================= */

void ToF_UpdateSensors(void)
{
    for(int i = 0; i < TOF_SENSOR_COUNT; i++)
    {
        VL53L1X_CheckForDataReady(tofAddresses[i], &ToF_DATA[i].dataReady);

        if(ToF_DATA[i].dataReady)
        {
            VL53L1X_GetDistance(tofAddresses[i], &ToF_DATA[i].distance);
            VL53L1X_GetRangeStatus(tofAddresses[i], &ToF_DATA[i].rangeStatus);
            VL53L1X_ClearInterrupt(tofAddresses[i]);

            // Ignore invalid readings
            if(ToF_DATA[i].rangeStatus != 0)
            	ToF_DATA[i].distance = 0;

            ToF_DATA[i].dataReady = 0;
        }
    }
}


/* ================= Distance Access ================= */

uint16_t ToF_GetLeftDistance(void)
{
    return ToF_DATA[TOF_LEFT].distance;
}

uint16_t ToF_GetCenterDistance(void)
{
    return ToF_DATA[TOF_CENTER].distance;
}

uint16_t ToF_GetRightDistance(void)
{
    return ToF_DATA[TOF_RIGHT].distance;
}


/* ================= Filter Helper Function ================= */

static uint16_t Filter(uint16_t previous, uint16_t current)
{
    return 0.2 * previous + 0.8 * current;
}

/* ================= Obstacle Avoiding Algorithm ================= */

void ToF_SetRobotCommand(void)
{
	uint16_t left   = ToF_GetLeftDistance();
	uint16_t center = ToF_GetCenterDistance();
	uint16_t right  = ToF_GetRightDistance();

	/* Treat distance = 0 as max distance */
	if(left == 0)   left = 4000;
	if(center == 0) center = 4000;
	if(right == 0)  right = 4000;

	/* Apply filtering */
	filtered_left   = Filter(filtered_left, left);
	filtered_center = Filter(filtered_center, center);
	filtered_right  = Filter(filtered_right, right);

	/* Default command */
	robotCommand = (RobotCommand_t){ .dodgeAngle = 90 };

	/* ================= FRONT OBSTACLE ================= */

	if(filtered_center < OBSTACLE_THRESHOLD_FAR)
	{
		int turn_left = (filtered_left > filtered_right);

		/* CLOSE obstacle */
		if(filtered_center < OBSTACLE_THRESHOLD_CLOSE)
		{
			if(turn_left)
				robotCommand = (RobotCommand_t){ .dodgeAngle = 150 }; // strong left
			else
				robotCommand = (RobotCommand_t){ .dodgeAngle = 30 };  // strong right
		}
		else
		{
			if(turn_left)
				robotCommand = (RobotCommand_t){ .dodgeAngle = 120 }; // gentle left
			else
				robotCommand = (RobotCommand_t){ .dodgeAngle = 60 };  // gentle right
		}
	}

	/* ================= LEFT OBSTACLE ================= */

	else if(filtered_left < SIDE_THRESHOLD_FAR)
	{
		if(filtered_left < SIDE_THRESHOLD_CLOSE)
			robotCommand = (RobotCommand_t){ .dodgeAngle = 30 }; // strong right
		else
			robotCommand = (RobotCommand_t){ .dodgeAngle = 60 }; // gentle right
	}

	/* ================= RIGHT OBSTACLE ================= */

	else if(filtered_right < SIDE_THRESHOLD_FAR)
	{
		if(filtered_right < SIDE_THRESHOLD_CLOSE)
			robotCommand = (RobotCommand_t){ .dodgeAngle = 150 }; // strong left
		else
			robotCommand = (RobotCommand_t){ .dodgeAngle = 120 }; // gentle left
	}

	/* ================= CLEAR PATH ================= */

	else
	{
		robotCommand = (RobotCommand_t){ .dodgeAngle = 90 }; // straight
	}
}






