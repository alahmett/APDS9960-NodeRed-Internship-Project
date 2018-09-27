#include "apds9960.h"
#include <Arduino.h>
#include <Wire.h>


APDS9960::APDS9960()
{
	gesture_ud_delta_ = 0;
	gesture_lr_delta_ = 0;

	gesture_ud_count_ = 0;
	gesture_lr_count_ = 0;

	gesture_near_count_ = 0;
	gesture_far_count_ = 0;

	gesture_state_ = 0;
	gesture_motion_ = DIR_NONE;
}

/**
* @brief Destructor
*/
APDS9960::~APDS9960()
{

}



bool APDS9960::apds9960_wireReadDataByte(uint8_t reg, uint8_t &val)
{

	/* Indicate which register we want to read from */
	if (!apds9960_wireWriteByte(reg)) {
		return false;
	}

	/* Read from register */
	Wire.requestFrom(APDS9960_I2C_ADDR, 1);
	while (Wire.available()) {
		val = Wire.read();
	}

	return true;
}

bool APDS9960::apds9960_wireWriteDataByte(uint8_t reg, uint8_t val)
{
	Wire.beginTransmission(APDS9960_I2C_ADDR);
	Wire.write(reg);
	Wire.write(val);
	if (Wire.endTransmission() != 0) {
		return false;
	}

	return true;
}


bool APDS9960::apds9960_setMode(uint8_t mode, uint8_t enable)
{
	uint8_t reg_val;

	/* Read current ENABLE register */
	reg_val = apds9960_getMode();
	if (reg_val == ERROR) {
		return false;
	}

	/* Change bit(s) in ENABLE register */
	enable = enable & 0x01;
	if (mode >= 0 && mode <= 6) {
		if (enable) {
			reg_val |= (1 << mode);
		}
		else {
			reg_val &= ~(1 << mode);
		}
	}
	else if (mode == ALL) {
		if (enable) {
			reg_val = 0x7F;
		}
		else {
			reg_val = 0x00;
		}
	}

	/* Write value back to ENABLE register */
	if (!apds9960_wireWriteDataByte(APDS9960_ENABLE, reg_val)) {
		return false;
	}

	return true;
}


uint8_t APDS9960::apds9960_getMode()
{
	uint8_t enable_value;

	/* Read current ENABLE register */
	if (!apds9960_wireReadDataByte(APDS9960_ENABLE, enable_value)) {
		return ERROR;
	}

	return enable_value;
}


bool APDS9960::apds9960_wireWriteByte(uint8_t val)
{
	Wire.beginTransmission(APDS9960_I2C_ADDR);
	Wire.write(val);
	if (Wire.endTransmission() != 0) {
		return false;
	}

	return true;
}

bool APDS9960::apds9960_setLEDDrive(uint8_t drive)
{
	uint8_t val;

	/* Read value from CONTROL register */
	if (!apds9960_wireReadDataByte(APDS9960_CONTROL, val)) {
		return false;
	}

	/* Set bits in register to given value */
	drive &= 0b00000011;
	drive = drive << 6;
	val &= 0b00111111;
	val |= drive;

	/* Write register value back into CONTROL register */
	if (!apds9960_wireWriteDataByte(APDS9960_CONTROL, val)) {
		return false;
	}

	return true;
}

bool APDS9960::apds9960_setProximityGain(uint8_t drive)
{
	uint8_t val;

	/* Read value from CONTROL register */
	if (!apds9960_wireReadDataByte(APDS9960_CONTROL, val)) {
		return false;
	}

	/* Set bits in register to given value */
	drive &= 0b00000011;
	drive = drive << 2;
	val &= 0b11110011;
	val |= drive;

	/* Write register value back into CONTROL register */
	if (!apds9960_wireWriteDataByte(APDS9960_CONTROL, val)) {
		return false;
	}

	return true;
}


bool APDS9960::apds9960_setProxIntLowThresh(uint8_t threshold)
{
	if (!apds9960_wireWriteDataByte(APDS9960_PILT, threshold)) {
		return false;
	}

	return true;
}
bool APDS9960::apds9960_setProxIntHighThresh(uint8_t threshold)
{
	if (!apds9960_wireWriteDataByte(APDS9960_PIHT, threshold)) {
		return false;
	}

	return true;
}


bool APDS9960::apds9960_setLightIntLowThreshold(uint16_t threshold)
{
	uint8_t val_low;
	uint8_t val_high;

	/* Break 16-bit threshold into 2 8-bit values */
	val_low = threshold & 0x00FF;
	val_high = (threshold & 0xFF00) >> 8;

	/* Write low byte */
	if (!apds9960_wireWriteDataByte(APDS9960_AILTL, val_low)) {
		return false;
	}

	/* Write high byte */
	if (!apds9960_wireWriteDataByte(APDS9960_AILTH, val_high)) {
		return false;
	}

	return true;
}



bool APDS9960::apds9960_setLightIntHighThreshold(uint16_t threshold)
{
	uint8_t val_low;
	uint8_t val_high;

	/* Break 16-bit threshold into 2 8-bit values */
	val_low = threshold & 0x00FF;
	val_high = (threshold & 0xFF00) >> 8;

	/* Write low byte */
	if (!apds9960_wireWriteDataByte(APDS9960_AIHTL, val_low)) {
		return false;
	}

	/* Write high byte */
	if (!apds9960_wireWriteDataByte(APDS9960_AIHTH, val_high)) {
		return false;
	}

	return true;
}

bool APDS9960::apds9960_setGestureEnterThresh(uint8_t threshold)
{
	if (!apds9960_wireWriteDataByte(APDS9960_GPENTH, threshold)) {
		return false;
	}

	return true;
}

bool APDS9960::apds9960_setGestureExitThresh(uint8_t threshold)
{
	if (!apds9960_wireWriteDataByte(APDS9960_GEXTH, threshold)) {
		return false;
	}

	return true;
}

bool APDS9960::apds9960_setGestureGain(uint8_t gain)
{
	uint8_t val;

	/* Read value from GCONF2 register */
	if (!apds9960_wireReadDataByte(APDS9960_GCONF2, val)) {
		return false;
	}

	/* Set bits in register to given value */
	gain &= 0b00000011;
	gain = gain << 5;
	val &= 0b10011111;
	val |= gain;

	/* Write register value back into GCONF2 register */
	if (!apds9960_wireWriteDataByte(APDS9960_GCONF2, val)) {
		return false;
	}

	return true;
}
bool APDS9960::apds9960_setGestureLEDDrive(uint8_t drive)
{
	uint8_t val;

	/* Read value from GCONF2 register */
	if (!apds9960_wireReadDataByte(APDS9960_GCONF2, val)) {
		return false;
	}

	/* Set bits in register to given value */
	drive &= 0b00000011;
	drive = drive << 3;
	val &= 0b11100111;
	val |= drive;

	/* Write register value back into GCONF2 register */
	if (!apds9960_wireWriteDataByte(APDS9960_GCONF2, val)) {
		return false;
	}

	return true;
}

bool APDS9960::apds9960_setGestureWaitTime(uint8_t time)
{
	uint8_t val;

	/* Read value from GCONF2 register */
	if (!apds9960_wireReadDataByte(APDS9960_GCONF2, val)) {
		return false;
	}

	/* Set bits in register to given value */
	time &= 0b00000111;
	val &= 0b11111000;
	val |= time;

	/* Write register value back into GCONF2 register */
	if (!apds9960_wireWriteDataByte(APDS9960_GCONF2, val)) {
		return false;
	}

	return true;
}

bool APDS9960::apds9960_setGestureIntEnable(uint8_t enable)
{
	uint8_t val;

	/* Read value from GCONF4 register */
	if (!apds9960_wireReadDataByte(APDS9960_GCONF4, val)) {
		return false;
	}

	/* Set bits in register to given value */
	enable &= 0b00000001;
	enable = enable << 1;
	val &= 0b11111101;
	val |= enable;

	/* Write register value back into GCONF4 register */
	if (!apds9960_wireWriteDataByte(APDS9960_GCONF4, val)) {
		return false;
	}

	return true;
}



























//-------------------------------------------------------------------
void APDS9960::resetGestureParameters()
{
	gesture_data_.index = 0;
	gesture_data_.total_gestures = 0;

	gesture_ud_delta_ = 0;
	gesture_lr_delta_ = 0;

	gesture_ud_count_ = 0;
	gesture_lr_count_ = 0;

	gesture_near_count_ = 0;
	gesture_far_count_ = 0;

	gesture_state_ = 0;
	gesture_motion_ = DIR_NONE;
}

bool APDS9960::apds9960_setLEDBoost(uint8_t boost)
{
	uint8_t val;

	/* Read value from CONFIG2 register */
	if (!apds9960_wireReadDataByte(APDS9960_CONFIG2, val)) {
		return false;
	}

	/* Set bits in register to given value */
	boost &= 0b00000011;
	boost = boost << 4;
	val &= 0b11001111;
	val |= boost;

	/* Write register value back into CONFIG2 register */
	if (!apds9960_wireWriteDataByte(APDS9960_CONFIG2, val)) {
		return false;
	}

	return true;
}



bool APDS9960::apds9960_enablePower()
{
	if (!apds9960_setMode(POWER, 1)) {
		return false;
	}

	return true;
}


bool APDS9960::apds9960_setProximityIntEnable(uint8_t enable)
{
	uint8_t val;

	/* Read value from ENABLE register */
	if (!apds9960_wireReadDataByte(APDS9960_ENABLE, val)) {
		return false;
	}

	/* Set bits in register to given value */
	enable &= 0b00000001;
	enable = enable << 5;
	val &= 0b11011111;
	val |= enable;

	/* Write register value back into ENABLE register */
	if (!apds9960_wireWriteDataByte(APDS9960_ENABLE, val)) {
		return false;
	}

	return true;
}


bool APDS9960::apds9960_setProximityIntLowThreshold(uint8_t threshold)
{

	/* Write threshold value to register */
	if (!apds9960_wireWriteDataByte(APDS9960_PILT, threshold)) {
		return false;
	}

	return true;
}
bool APDS9960::apds9960_setProximityIntHighThreshold(uint8_t threshold)
{

	/* Write threshold value to register */
	if (!apds9960_wireWriteDataByte(APDS9960_PIHT, threshold)) {
		return false;
	}

	return true;
}
//-------------------------------------------------------------------------------
/**
* @brief Determines if there is a gesture available for reading
*
* @return True if gesture available. False otherwise.
*/
bool APDS9960::apds9960_isGestureAvailable()
{
	uint8_t val;

	/* Read value from GSTATUS register */
	if (!apds9960_wireReadDataByte(APDS9960_GSTATUS, val)) {
		return ERROR;
	}

	/* Shift and mask out GVALID bit */
	val &= APDS9960_GVALID;

	/* Return true/false based on GVALID bit */
	if (val == 1) {
		return true;
	}
	else {
		return false;
	}
}

/**
* @brief Reads a block (array) of bytes from the I2C device and register
*
* @param[in] reg the register to read from
* @param[out] val pointer to the beginning of the data
* @param[in] len number of bytes to read
* @return Number of bytes read. -1 on read error.
*/
int APDS9960::apds9960_wireReadDataBlock(uint8_t reg,
	uint8_t *val,
	unsigned int len)
{
	unsigned char i = 0;

	/* Indicate which register we want to read from */
	if (!apds9960_wireWriteByte(reg)) {
		return -1;
	}

	/* Read block data */
	Wire.requestFrom(APDS9960_I2C_ADDR, len);
	while (Wire.available()) {
		if (i >= len) {
			return -1;
		}
		val[i] = Wire.read();
		i++;
	}

	return i;
}

bool APDS9960::apds9960_processGestureData()
{
	uint8_t u_first = 0;
	uint8_t d_first = 0;
	uint8_t l_first = 0;
	uint8_t r_first = 0;
	uint8_t u_last = 0;
	uint8_t d_last = 0;
	uint8_t l_last = 0;
	uint8_t r_last = 0;
	int ud_ratio_first;
	int lr_ratio_first;
	int ud_ratio_last;
	int lr_ratio_last;
	int ud_delta;
	int lr_delta;
	int i;

	/* If we have less than 4 total gestures, that's not enough */
	if (gesture_data_.total_gestures <= 4) {
		return false;
	}

	/* Check to make sure our data isn't out of bounds */
	if ((gesture_data_.total_gestures <= 32) && \
		(gesture_data_.total_gestures > 0)) {

		/* Find the first value in U/D/L/R above the threshold */
		for (i = 0; i < gesture_data_.total_gestures; i++) {
			if ((gesture_data_.u_data[i] > GESTURE_THRESHOLD_OUT) &&
				(gesture_data_.d_data[i] > GESTURE_THRESHOLD_OUT) &&
				(gesture_data_.l_data[i] > GESTURE_THRESHOLD_OUT) &&
				(gesture_data_.r_data[i] > GESTURE_THRESHOLD_OUT)) {

				u_first = gesture_data_.u_data[i];
				d_first = gesture_data_.d_data[i];
				l_first = gesture_data_.l_data[i];
				r_first = gesture_data_.r_data[i];
				break;
			}
		}

		/* If one of the _first values is 0, then there is no good data */
		if ((u_first == 0) || (d_first == 0) || \
			(l_first == 0) || (r_first == 0)) {

			return false;
		}
		/* Find the last value in U/D/L/R above the threshold */
		for (i = gesture_data_.total_gestures - 1; i >= 0; i--) {
			if ((gesture_data_.u_data[i] > GESTURE_THRESHOLD_OUT) &&
				(gesture_data_.d_data[i] > GESTURE_THRESHOLD_OUT) &&
				(gesture_data_.l_data[i] > GESTURE_THRESHOLD_OUT) &&
				(gesture_data_.r_data[i] > GESTURE_THRESHOLD_OUT)) {

				u_last = gesture_data_.u_data[i];
				d_last = gesture_data_.d_data[i];
				l_last = gesture_data_.l_data[i];
				r_last = gesture_data_.r_data[i];
				break;
			}
		}
	}

	/* Calculate the first vs. last ratio of up/down and left/right */
	ud_ratio_first = ((u_first - d_first) * 100) / (u_first + d_first);
	lr_ratio_first = ((l_first - r_first) * 100) / (l_first + r_first);
	ud_ratio_last = ((u_last - d_last) * 100) / (u_last + d_last);
	lr_ratio_last = ((l_last - r_last) * 100) / (l_last + r_last);

	/* Determine the difference between the first and last ratios */
	ud_delta = ud_ratio_last - ud_ratio_first;
	lr_delta = lr_ratio_last - lr_ratio_first;

	/* Accumulate the UD and LR delta values */
	gesture_ud_delta_ += ud_delta;
	gesture_lr_delta_ += lr_delta;

	/* Determine U/D gesture */
	if (gesture_ud_delta_ >= GESTURE_SENSITIVITY_1) {
		gesture_ud_count_ = 1;
	}
	else if (gesture_ud_delta_ <= -GESTURE_SENSITIVITY_1) {
		gesture_ud_count_ = -1;
	}
	else {
		gesture_ud_count_ = 0;
	}

	/* Determine L/R gesture */
	if (gesture_lr_delta_ >= GESTURE_SENSITIVITY_1) {
		gesture_lr_count_ = 1;
	}
	else if (gesture_lr_delta_ <= -GESTURE_SENSITIVITY_1) {
		gesture_lr_count_ = -1;
	}
	else {
		gesture_lr_count_ = 0;
	}

	/* Determine Near/Far gesture */
	if ((gesture_ud_count_ == 0) && (gesture_lr_count_ == 0)) {
		if ((abs(ud_delta) < GESTURE_SENSITIVITY_2) && \
			(abs(lr_delta) < GESTURE_SENSITIVITY_2)) {

			if ((ud_delta == 0) && (lr_delta == 0)) {
				gesture_near_count_++;
			}
			else if ((ud_delta != 0) || (lr_delta != 0)) {
				gesture_far_count_++;
			}

			if ((gesture_near_count_ >= 10) && (gesture_far_count_ >= 2)) {
				if ((ud_delta == 0) && (lr_delta == 0)) {
					gesture_state_ = NEAR_STATE;
				}
				else if ((ud_delta != 0) && (lr_delta != 0)) {
					gesture_state_ = FAR_STATE;
				}
				return true;
			}
		}
	}
	else {
		if ((abs(ud_delta) < GESTURE_SENSITIVITY_2) && \
			(abs(lr_delta) < GESTURE_SENSITIVITY_2)) {

			if ((ud_delta == 0) && (lr_delta == 0)) {
				gesture_near_count_++;
			}

			if (gesture_near_count_ >= 10) {
				gesture_ud_count_ = 0;
				gesture_lr_count_ = 0;
				gesture_ud_delta_ = 0;
				gesture_lr_delta_ = 0;
			}
		}
	}


	return false;
}

bool APDS9960::apds9960_decodeGesture()
{
	/* Return if near or far event is detected */
	if (gesture_state_ == NEAR_STATE) {
		gesture_motion_ = DIR_NEAR;
		return true;
	}
	else if (gesture_state_ == FAR_STATE) {
		gesture_motion_ = DIR_FAR;
		return true;
	}

	/* Determine swipe direction */
	if ((gesture_ud_count_ == -1) && (gesture_lr_count_ == 0)) {
		gesture_motion_ = DIR_UP;
	}
	else if ((gesture_ud_count_ == 1) && (gesture_lr_count_ == 0)) {
		gesture_motion_ = DIR_DOWN;
	}
	else if ((gesture_ud_count_ == 0) && (gesture_lr_count_ == 1)) {
		gesture_motion_ = DIR_RIGHT;
	}
	else if ((gesture_ud_count_ == 0) && (gesture_lr_count_ == -1)) {
		gesture_motion_ = DIR_LEFT;
	}
	else if ((gesture_ud_count_ == -1) && (gesture_lr_count_ == 1)) {
		if (abs(gesture_ud_delta_) > abs(gesture_lr_delta_)) {
			gesture_motion_ = DIR_UP;
		}
		else {
			gesture_motion_ = DIR_RIGHT;
		}
	}
	else if ((gesture_ud_count_ == 1) && (gesture_lr_count_ == -1)) {
		if (abs(gesture_ud_delta_) > abs(gesture_lr_delta_)) {
			gesture_motion_ = DIR_DOWN;
		}
		else {
			gesture_motion_ = DIR_LEFT;
		}
	}
	else if ((gesture_ud_count_ == -1) && (gesture_lr_count_ == -1)) {
		if (abs(gesture_ud_delta_) > abs(gesture_lr_delta_)) {
			gesture_motion_ = DIR_UP;
		}
		else {
			gesture_motion_ = DIR_LEFT;
		}
	}
	else if ((gesture_ud_count_ == 1) && (gesture_lr_count_ == 1)) {
		if (abs(gesture_ud_delta_) > abs(gesture_lr_delta_)) {
			gesture_motion_ = DIR_DOWN;
		}
		else {
			gesture_motion_ = DIR_RIGHT;
		}
	}
	else {
		return false;
	}

	return true;
}

/**
* @brief Resets all the parameters in the gesture data member
*/

//------------------------------------------------------------------------------

/**
* @brief Reads the proximity level as an 8-bit value
*
* @param[out] val value of the proximity sensor.
* @return True if operation successful. False otherwise.
*/
bool APDS9960::apds9960_readProximity(uint8_t &val)
{
	val = 0;

	/* Read value from proximity data register */
	if (!apds9960_wireReadDataByte(APDS9960_PDATA, val)) {
		return false;
	}

	return true;
}

//-------------------------------------------------------------------------------
/**
* @brief Processes a gesture event and returns best guessed gesture
*
* @return Number corresponding to gesture. -1 on error.
*/
int APDS9960::apds9960_readGesture()
{
	uint8_t fifo_level = 0;
	uint8_t bytes_read = 0;
	uint8_t fifo_data[128];
	uint8_t gstatus;
	int motion;
	int i;

	/**
	for (int i = 0; i < 128; i++)
	{
	Serial.print("test ");
	Serial.print(i);
	Serial.print(": ");
	Serial.println(fifo_data[i]);
	}*/

	/* Make sure that power and gesture is on and data is valid */
	if (!apds9960_isGestureAvailable() || !(apds9960_getMode() & 0b01000001)) {
		return DIR_NONE;
	}

	/* Keep looping as long as gesture data is valid */
	while (1) {

		/* Wait some time to collect next batch of FIFO data */
		delay(FIFO_PAUSE_TIME);

		/* Get the contents of the STATUS register. Is data still valid? */
		if (!apds9960_wireReadDataByte(APDS9960_GSTATUS, gstatus)) {
			return ERROR;
		}

		/* If we have valid data, read in FIFO */
		if ((gstatus & APDS9960_GVALID) == APDS9960_GVALID) {

			/* Read the current FIFO level */
			if (!apds9960_wireReadDataByte(APDS9960_GFLVL, fifo_level)) {
				return ERROR;
			}

			/* If there's stuff in the FIFO, read it into our data block */
			if (fifo_level > 0) {
				bytes_read = apds9960_wireReadDataBlock(APDS9960_GFIFO_U,
					(uint8_t*)fifo_data,
					(fifo_level * 4));
				if (bytes_read == -1) {
					return ERROR;
				}

        size_t n = sizeof(fifo_data) / sizeof(fifo_data[0]);
				/* If at least 1 set of data, sort the data into U/D/L/R */
				if (bytes_read >= 4) {
					for (i = 0; i < bytes_read; i += 4) {
						gesture_data_.u_data[gesture_data_.index] = \
							fifo_data[i + 0];
						gesture_data_.d_data[gesture_data_.index] = \
							fifo_data[i + 1];
						gesture_data_.l_data[gesture_data_.index] = \
							fifo_data[i + 2];
						gesture_data_.r_data[gesture_data_.index] = \
							fifo_data[i + 3];
						gesture_data_.index++;
						gesture_data_.total_gestures++;

						u_ptr = &gesture_data_.u_data[0];
						d_ptr = &gesture_data_.d_data[0];
						l_ptr = &gesture_data_.l_data[0];
						r_ptr = &gesture_data_.r_data[0];

						//		ptr = &gesture_data_;

					}
			

					/* Filter and process gesture data. Decode near/far state */
					if (apds9960_processGestureData()) {
						if (apds9960_decodeGesture()) {
							//***TODO: U-Turn Gestures



						}
					}

					tot_num_gest = n/4;

					/* Reset data */
					gesture_data_.index = 0;
					gesture_data_.total_gestures = 0;
				}
			}
		}
		else {

			/* Determine best guessed gesture and clean up */
			delay(FIFO_PAUSE_TIME);
			apds9960_decodeGesture();
			motion = gesture_motion_;



			resetGestureParameters();




			return motion;
		}
	}
}


/**
* @brief Tells the state machine to either enter or exit gesture state machine
*
* @param[in] mode 1 to enter gesture state machine, 0 to exit.
* @return True if operation successful. False otherwise.
*/
bool APDS9960::apds9960_setGestureMode(uint8_t mode)
{
	uint8_t val;

	/* Read value from GCONF4 register */
	if (!apds9960_wireReadDataByte(APDS9960_GCONF4, val)) {
		return false;
	}

	/* Set bits in register to given value */
	mode &= 0b00000001;
	val &= 0b11111110;
	val |= mode;

	/* Write register value back into GCONF4 register */
	if (!apds9960_wireWriteDataByte(APDS9960_GCONF4, val)) {
		return false;
	}

	return true;
}










//---------------------------------------------------------------------------
bool APDS9960::apds9960_readAmbientLight(uint16_t &val)
{
	uint8_t val_byte;
	val = 0;

	/* Read value from clear channel, low byte register */
	if (!apds9960_wireReadDataByte(APDS9960_CDATAL, val_byte)) {
		return false;
	}
	val = val_byte;

	/* Read value from clear channel, high byte register */
	if (!apds9960_wireReadDataByte(APDS9960_CDATAH, val_byte)) {
		return false;
	}
	val = val + ((uint16_t)val_byte << 8);
	//c == val;
	return true;
}

/**
* @brief Reads the red light level as a 16-bit value
*
* @param[out] val value of the light sensor.
* @return True if operation successful. False otherwise.
*/
bool APDS9960::apds9960_readRedLight(uint16_t &val)
{
	uint8_t val_byte;
	val = 0;

	/* Read value from clear channel, low byte register */
	if (!apds9960_wireReadDataByte(APDS9960_RDATAL, val_byte)) {
		return false;
	}
	val = val_byte;

	/* Read value from clear channel, high byte register */
	if (!apds9960_wireReadDataByte(APDS9960_RDATAH, val_byte)) {
		return false;
	}
	val = val + ((uint16_t)val_byte << 8);
	//r == val;

	return true;
}

/**
* @brief Reads the green light level as a 16-bit value
*
* @param[out] val value of the light sensor.
* @return True if operation successful. False otherwise.
*/
bool APDS9960::apds9960_readGreenLight(uint16_t &val)
{
	uint8_t val_byte;
	val = 0;

	/* Read value from clear channel, low byte register */
	/*if (!wireReadDataByte(APDS9960_GDATAL, val_byte)) {
	return false;
	}*/
	if (apds9960_wireReadDataByte(APDS9960_GDATAL, val_byte)) {
		val = val_byte;
		return val;
	}


	/* Read value from clear channel, high byte register */
	/*	if (!wireReadDataByte(APDS9960_GDATAH, val_byte)) {
	return false;
	}
	val = val + ((uint16_t)val_byte << 8);
	*/
	if (apds9960_wireReadDataByte(APDS9960_GDATAH, val_byte)) {
		val = val + ((uint16_t)val_byte << 8);
		return val;
	}


	//g == val;

	return true;
}

/**
* @brief Reads the red light level as a 16-bit value
*
* @param[out] val value of the light sensor.
* @return True if operation successful. False otherwise.
*/
bool APDS9960::apds9960_readBlueLight(uint16_t &val)
{
	uint8_t val_byte;
	val = 0;

	/* Read value from clear channel, low byte register */
	if (!apds9960_wireReadDataByte(APDS9960_BDATAL, val_byte)) {
		return false;
	}
	val = val_byte;

	/* Read value from clear channel, high byte register */
	if (!apds9960_wireReadDataByte(APDS9960_BDATAH, val_byte)) {
		return false;
	}
	val = val + ((uint16_t)val_byte << 8);
	//b == val;

	return true;
}







//--------------------------------------------------------------------------------
bool APDS9960::apds9960_setAmbientLightGain(uint8_t drive)
{
	uint8_t val;

	/* Read value from CONTROL register */
	if (!apds9960_wireReadDataByte(APDS9960_CONTROL, val)) {
		return false;
	}

	/* Set bits in register to given value */
	drive &= 0b00000011;
	val &= 0b11111100;
	val |= drive;

	/* Write register value back into CONTROL register */
	if (!apds9960_wireWriteDataByte(APDS9960_CONTROL, val)) {
		return false;
	}

	return true;
}

bool APDS9960::apds9960_setAmbientLightIntEnable(uint8_t enable)
{
	uint8_t val;

	/* Read value from ENABLE register */
	if (!apds9960_wireReadDataByte(APDS9960_ENABLE, val)) {
		return false;
	}

	/* Set bits in register to given value */
	enable &= 0b00000001;
	enable = enable << 4;
	val &= 0b11101111;
	val |= enable;

	/* Write register value back into ENABLE register */
	if (!apds9960_wireWriteDataByte(APDS9960_ENABLE, val)) {
		return false;
	}

	return true;
}


bool APDS9960::apds9960_enableLightSensor(bool interrupts)
{

	/* Set default gain, interrupts, enable power, and enable sensor */
	if (!apds9960_setAmbientLightGain(DEFAULT_AGAIN)) {
		return false;
	}
	if (interrupts) {
		if (!apds9960_setAmbientLightIntEnable(1)) {
			return false;
		}
	}
	else {
		if (!apds9960_setAmbientLightIntEnable(0)) {
			return false;
		}
	}
	if (!apds9960_enablePower()) {
		return false;
	}
	if (!apds9960_setMode(AMBIENT_LIGHT, 1)) {
		return false;
	}
	return true;
}


//----------------------------------------------------------------------------

bool APDS9960::apds9960_enableProximitySensor(bool interrupts)
{
	/* Set default gain, LED, interrupts, enable power, and enable sensor */
	if (!apds9960_setProximityGain(DEFAULT_PGAIN)) {
		return false;
	}
	if (!apds9960_setLEDDrive(DEFAULT_LDRIVE)) {
		return false;
	}
	if (interrupts) {
		if (!apds9960_setProximityIntEnable(1)) {
			return false;
		}
	}
	else {
		if (!apds9960_setProximityIntEnable(0)) {
			return false;
		}
	}
	if (!apds9960_enablePower()) {
		return false;
	}
	if (!apds9960_setMode(PROXIMITY, 1)) {
		return false;
	}

	return true;
}

//--------------------------------------------------------------
bool APDS9960::apds9960_enableGestureSensor(bool interrupts)
{

	/* Enable gesture mode
	Set ENABLE to 0 (power off)
	Set WTIME to 0xFF
	Set AUX to LED_BOOST_300
	Enable PON, WEN, PEN, GEN in ENABLE
	*/
	resetGestureParameters();
	if (!apds9960_wireWriteDataByte(APDS9960_WTIME, 0xFF)) {
		return false;
	}
	if (!apds9960_wireWriteDataByte(APDS9960_PPULSE, DEFAULT_GESTURE_PPULSE)) {
		return false;
	}
	if (!apds9960_setLEDBoost(LED_BOOST_300)) {
		return false;
	}
	if (interrupts) {
		if (!apds9960_setGestureIntEnable(1)) {
			return false;
		}
	}
	else {
		if (!apds9960_setGestureIntEnable(0)) {
			return false;
		}
	}
	if (!apds9960_setGestureMode(1)) {
		return false;
	}
	if (!apds9960_enablePower()) {
		return false;
	}
	if (!apds9960_setMode(WAIT, 1)) {
		return false;
	}
	if (!apds9960_setMode(PROXIMITY, 1)) {
		return false;
	}
	if (!apds9960_setMode(GESTURE, 1)) {
		return false;
	}

	return true;
}

//----------------------------------------------------------------------------

bool APDS9960::apds9960_init()
{
	uint8_t id;

	/* Initialize I2C */
	Wire.begin();

	/* Read ID register and check against known values for APDS-9960 */
	if (!apds9960_wireReadDataByte(APDS9960_ID, id)) {
		return false;
	}
	if (!(id == APDS9960_ID_1 || id == APDS9960_ID_2)) {
		return false;
	}

	/* Set ENABLE register to 0 (disable all features) */
	if (!apds9960_setMode(ALL, OFF)) {
		return false;
	}

	/* Set default values for ambient light and proximity registers */
	if (!apds9960_wireWriteDataByte(APDS9960_ATIME, DEFAULT_ATIME)) {
		return false;
	}
	if (!apds9960_wireWriteDataByte(APDS9960_WTIME, DEFAULT_WTIME)) {
		return false;
	}
	if (!apds9960_wireWriteDataByte(APDS9960_PPULSE, DEFAULT_PROX_PPULSE)) {
		return false;
	}
	if (!apds9960_wireWriteDataByte(APDS9960_POFFSET_UR, DEFAULT_POFFSET_UR)) {
		return false;
	}
	if (!apds9960_wireWriteDataByte(APDS9960_POFFSET_DL, DEFAULT_POFFSET_DL)) {
		return false;
	}
	if (!apds9960_wireWriteDataByte(APDS9960_CONFIG1, DEFAULT_CONFIG1)) {
		return false;
	}
	if (!apds9960_setLEDDrive(DEFAULT_LDRIVE)) {
		return false;
	}
	if (!apds9960_setProximityGain(DEFAULT_PGAIN)) {
		return false;
	}
	if (!apds9960_setAmbientLightGain(DEFAULT_AGAIN)) {
		return false;
	}
	if (!apds9960_setProxIntLowThresh(DEFAULT_PILT)) {
		return false;
	}
	if (!apds9960_setProxIntHighThresh(DEFAULT_PIHT)) {
		return false;
	}
	if (!apds9960_setLightIntLowThreshold(DEFAULT_AILT)) {
		return false;
	}
	if (!apds9960_setLightIntHighThreshold(DEFAULT_AIHT)) {
		return false;
	}
	if (!apds9960_wireWriteDataByte(APDS9960_PERS, DEFAULT_PERS)) {
		return false;
	}
	if (!apds9960_wireWriteDataByte(APDS9960_CONFIG2, DEFAULT_CONFIG2)) {
		return false;
	}
	if (!apds9960_wireWriteDataByte(APDS9960_CONFIG3, DEFAULT_CONFIG3)) {
		return false;
	}

	/* Set default values for gesture sense registers */
	if (!apds9960_setGestureEnterThresh(DEFAULT_GPENTH)) {
		return false;
	}
	//if (!setGestureExitThresh(DEFAULT_GEXTH)) {
	//	return false;
	//}
	if (!apds9960_wireWriteDataByte(APDS9960_GCONF1, DEFAULT_GCONF1)) {
		return false;
	}
	if (!apds9960_setGestureGain(DEFAULT_GGAIN)) {
		return false;
	}
	if (!apds9960_setGestureLEDDrive(DEFAULT_GLDRIVE)) {
		return false;
	}
	if (!apds9960_setGestureWaitTime(DEFAULT_GWTIME)) {
		return false;
	}
	if (!apds9960_wireWriteDataByte(APDS9960_GOFFSET_U, DEFAULT_GOFFSET)) {
		return false;
	}
	if (!apds9960_wireWriteDataByte(APDS9960_GOFFSET_D, DEFAULT_GOFFSET)) {
		return false;
	}
	if (!apds9960_wireWriteDataByte(APDS9960_GOFFSET_L, DEFAULT_GOFFSET)) {
		return false;
	}
	if (!apds9960_wireWriteDataByte(APDS9960_GOFFSET_R, DEFAULT_GOFFSET)) {
		return false;
	}
	if (!apds9960_wireWriteDataByte(APDS9960_GPULSE, DEFAULT_GPULSE)) {
		return false;
	}
	if (!apds9960_wireWriteDataByte(APDS9960_GCONF3, DEFAULT_GCONF3)) {
		return false;
	}
	if (!apds9960_setGestureIntEnable(DEFAULT_GIEN)) {
		return false;
	}
	return true;
}
//-----------------------------------------------------------------
bool APDS9960::apds9960_getGesture()
{
	uint8_t fifo_level = 0;
	uint8_t bytes_read = 0;
	uint8_t fifo_data[128];
	uint8_t gstatus;
	int motion;
	int i;

	/* Make sure that power and gesture is on and data is valid */
	if (!apds9960_isGestureAvailable() || !(apds9960_getMode() & 0b01000001)) {
		return DIR_NONE;
	}

	/* Keep looping as long as gesture data is valid */
	while (1) {

		/* Wait some time to collect next batch of FIFO data */
		delay(FIFO_PAUSE_TIME);

		/* Get the contents of the STATUS register. Is data still valid? */
		if (!apds9960_wireReadDataByte(APDS9960_GSTATUS, gstatus)) {
			return ERROR;
		}

		/* If we have valid data, read in FIFO */
		if ((gstatus & APDS9960_GVALID) == APDS9960_GVALID) {

			/* Read the current FIFO level */
			if (!apds9960_wireReadDataByte(APDS9960_GFLVL, fifo_level)) {
				return ERROR;
			}

			/* If there's stuff in the FIFO, read it into our data block */
			if (fifo_level > 0) {
				bytes_read = apds9960_wireReadDataBlock(APDS9960_GFIFO_U,
					(uint8_t*)fifo_data,
					(fifo_level * 4));
				if (bytes_read == -1) {
					return ERROR;
				}


				/* If at least 1 set of data, sort the data into U/D/L/R */
				if (bytes_read >= 4) {
					for (i = 0; i < bytes_read; i += 4) {
						gesture_data_.u_data[gesture_data_.index] = \
							fifo_data[i + 0];
						gesture_data_.d_data[gesture_data_.index] = \
							fifo_data[i + 1];
						gesture_data_.l_data[gesture_data_.index] = \
							fifo_data[i + 2];
						gesture_data_.r_data[gesture_data_.index] = \
							fifo_data[i + 3];
						gesture_data_.index++;
						gesture_data_.total_gestures++;

				/*		u_ptr = &gesture_data_.u_data[0];
						d_ptr = &gesture_data_.d_data[0];
						l_ptr = &gesture_data_.l_data[0];
						r_ptr = &gesture_data_.r_data[0];
						*/
						//		ptr = &gesture_data_;

					}

				//	tot_num_gest = gesture_data_.total_gestures;

					/* Reset data */
					gesture_data_.index = 0;
					gesture_data_.total_gestures = 0;
				}
		}
	}
		else {

			/* Determine best guessed gesture and clean up */
			delay(FIFO_PAUSE_TIME);
			resetGestureParameters();


	//		return motion;
		}
}





	return true;
}
