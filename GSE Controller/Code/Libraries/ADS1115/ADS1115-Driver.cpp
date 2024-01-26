#include <ADS1115-Driver.h>

//MSB - oddly, the Wire.begin in the address function was crashing, so moved it out to main



static uint8_t currentPGA = 255;

ADS1115::ADS1115(i2c_addr_t i2cAddr) {
	i2cAddress = i2cAddr;
	
	//Wire.begin();
}

ADS1115::~ADS1115() {}

/**
 * Public methods
 */

void ADS1115::reset() {
	writeRegister(ADS1115_CONFIG_REG_ADDR, ADS1115_CONFIG_REG_DEF);
	writeRegister(ADS1115_LOW_TRESH_REG_ADDR, ADS1115_LOW_TRESH_REG_DEF);
	writeRegister(ADS1115_HIGH_TRESH_REG_ADDR, ADS1115_HIGH_TRESH_REG_DEF);
}

reg_val_t ADS1115::readRegister(reg_addr_t dataAddress) {
	Wire.beginTransmission(i2cAddress);
	Wire.write(dataAddress);
	Wire.endTransmission();
	Wire.requestFrom(i2cAddress, (uint8_t) 2);

	if (Wire.available()) {
		uint8_t firstByte = Wire.read();
		uint8_t secondByte = Wire.read();

		return (firstByte << 8) + secondByte;
	}

	return -1;
}

void ADS1115::writeRegister(reg_addr_t dataAddress, uint16_t data) {
	uint8_t hByte = data >> 8;
	uint8_t lByte = data & 255;

	Wire.beginTransmission(i2cAddress);
	Wire.write(dataAddress);
	Wire.write(hByte);
	Wire.write(lByte);
	Wire.endTransmission();
}

reg_val_t ADS1115::healthTest() {
	//return readRegister(ADS1115_CONVERSION_REG_ADDR); zzz                                                             
	return readRegister(0b00000001);
}

reg_val_t ADS1115::readRawValue() {
	//return readRegister(ADS1115_CONVERSION_REG_ADDR); zzz                                                             
	return readRegister(0b00000000);
}

float ADS1115::readConvertedValue() {
    //uint16_t testVal = readRawValue(); //zzz
	//float rawValue = (float) testVal; //zzz
	float rawValue = (float) readRawValue();

	if (currentPGA == 255) {
		currentPGA = getPga();
	}
	

	float multiplier;

	switch (currentPGA) {
		case ADS1115_PGA_0_256:
			multiplier = ADS1115_PGA_0_256_MULT;
			break;
		case ADS1115_PGA_0_512:
			multiplier = ADS1115_PGA_0_512_MULT;
			break;
		case ADS1115_PGA_1_024:
			multiplier = ADS1115_PGA_1_024_MULT;
			break;
		case ADS1115_PGA_2_048:
			multiplier = ADS1115_PGA_2_048_MULT;
			break;
		case ADS1115_PGA_4_096:
			multiplier = ADS1115_PGA_4_096_MULT;
			break;
		case ADS1115_PGA_6_144:
			multiplier = ADS1115_PGA_6_144_MULT;
			break;
		default:
			multiplier = 1.0;
			break;
	}
    if((multiplier * rawValue)>6500.0) rawValue = 0;
	return multiplier * rawValue;
}

void ADS1115::startSingleConvertion() {
	writeFlag(ADS1115_CONFIG_REG_ADDR, ADS1115_OS_FLAG_POS, 1);
}

uint8_t ADS1115::getOperationalStatus() {
	return readFlag(ADS1115_CONFIG_REG_ADDR, ADS1115_OS_FLAG_POS);
}

void ADS1115::setMultiplexer(uint8_t mux) {
	writeFlag(ADS1115_CONFIG_REG_ADDR, ADS1115_MUX2_DAT_POS, getFlag(mux, 2));
	writeFlag(ADS1115_CONFIG_REG_ADDR, ADS1115_MUX1_DAT_POS, getFlag(mux, 1));
	writeFlag(ADS1115_CONFIG_REG_ADDR, ADS1115_MUX0_DAT_POS, getFlag(mux, 0));
}

uint8_t ADS1115::getMultiplexer() {
	uint8_t mux = readFlag(ADS1115_CONFIG_REG_ADDR, ADS1115_MUX0_DAT_POS);

	mux |= readFlag(ADS1115_CONFIG_REG_ADDR, ADS1115_MUX1_DAT_POS) << 1;
	mux |= readFlag(ADS1115_CONFIG_REG_ADDR, ADS1115_MUX2_DAT_POS) << 2;

	return mux;
}

void ADS1115::setPga(uint8_t pga) {
	currentPGA = pga;

	writeFlag(ADS1115_CONFIG_REG_ADDR, ADS1115_PGA2_DAT_POS, getFlag(pga, 2));
	writeFlag(ADS1115_CONFIG_REG_ADDR, ADS1115_PGA1_DAT_POS, getFlag(pga, 1));
	writeFlag(ADS1115_CONFIG_REG_ADDR, ADS1115_PGA0_DAT_POS, getFlag(pga, 0));
}

uint8_t ADS1115::getPga() {
	uint8_t pga = readFlag(ADS1115_CONFIG_REG_ADDR, ADS1115_PGA0_DAT_POS);

	pga |= readFlag(ADS1115_CONFIG_REG_ADDR, ADS1115_PGA1_DAT_POS) << 1;
	pga |= readFlag(ADS1115_CONFIG_REG_ADDR, ADS1115_PGA2_DAT_POS) << 2;

	return pga;
}

void ADS1115::setDeviceMode(uint8_t mode) {
	writeFlag(ADS1115_CONFIG_REG_ADDR, ADS1115_MODE_FLAG_POS, mode);
}

uint8_t ADS1115::getDeviceMode() {
	return readFlag(ADS1115_CONFIG_REG_ADDR, ADS1115_MODE_FLAG_POS);
}

void ADS1115::setDataRate(uint8_t dataRate) {
	writeFlag(ADS1115_CONFIG_REG_ADDR, ADS1115_DR2_DAT_POS, getFlag(dataRate, 2));
	writeFlag(ADS1115_CONFIG_REG_ADDR, ADS1115_DR1_DAT_POS, getFlag(dataRate, 1));
	writeFlag(ADS1115_CONFIG_REG_ADDR, ADS1115_DR0_DAT_POS, getFlag(dataRate, 0));
}

uint8_t ADS1115::getDataRate() {
	uint8_t dataRate = readFlag(ADS1115_CONFIG_REG_ADDR, ADS1115_DR0_DAT_POS);

	dataRate |= readFlag(ADS1115_CONFIG_REG_ADDR, ADS1115_DR1_DAT_POS) << 1;
	dataRate |= readFlag(ADS1115_CONFIG_REG_ADDR, ADS1115_DR2_DAT_POS) << 2;

	return dataRate;
}

void ADS1115::setComparatorMode(uint8_t mode) {
	writeFlag(ADS1115_CONFIG_REG_ADDR, ADS1115_COMP_MODE_FLAG_POS, mode);
}

uint8_t ADS1115::getComparatorMode() {
	return readFlag(ADS1115_CONFIG_REG_ADDR, ADS1115_COMP_MODE_FLAG_POS);
}

void ADS1115::setComparatorPolarity(uint8_t polarity) {
	writeFlag(ADS1115_CONFIG_REG_ADDR, ADS1115_COMP_POL_FLAG_POS, polarity);
}

uint8_t ADS1115::getComparatorPolarity() {
	return readFlag(ADS1115_CONFIG_REG_ADDR, ADS1115_COMP_POL_FLAG_POS);
}

void ADS1115::setLatching(bool latching) {
	writeFlag(ADS1115_CONFIG_REG_ADDR, ADS1115_COMP_LAT_FLAG_POS, latching);
}

bool ADS1115::isLatching() {
	return readFlag(ADS1115_CONFIG_REG_ADDR, ADS1115_COMP_LAT_FLAG_POS);
}

void ADS1115::setComparatorQueue(uint8_t queue) {
	writeFlag(ADS1115_CONFIG_REG_ADDR, ADS1115_COMP_QUE1_DAT_POS, getFlag(queue, 1));
	writeFlag(ADS1115_CONFIG_REG_ADDR, ADS1115_COMP_QUE0_DAT_POS, getFlag(queue, 0));
}

uint8_t ADS1115::getComparatorQueue() {
	uint8_t queue = readFlag(ADS1115_CONFIG_REG_ADDR, ADS1115_COMP_QUE0_DAT_POS);

	queue |= readFlag(ADS1115_CONFIG_REG_ADDR, ADS1115_COMP_QUE1_DAT_POS) << 1;

	return queue;
}

void ADS1115::setLowTreshold(reg_val_t lowTreshold) {
	writeRegister(ADS1115_LOW_TRESH_REG_ADDR, lowTreshold);
}

reg_val_t ADS1115::getLowTreshold() {
	return readRegister(ADS1115_LOW_TRESH_REG_ADDR);
}

void ADS1115::setHighTreshold(reg_val_t highTreshold) {
	writeRegister(ADS1115_HIGH_TRESH_REG_ADDR, highTreshold);
}

reg_val_t ADS1115::getHighTreshold() {
	return readRegister(ADS1115_HIGH_TRESH_REG_ADDR);
}

/**
 * Private methods
 */

flag_val_t ADS1115::readFlag(reg_addr_t dataAddress, flag_pos_t flagPos) {
	reg_val_t registerValue = readRegister(dataAddress);

	return getFlag(registerValue, flagPos);
}

void ADS1115::writeFlag(reg_addr_t dataAddress, flag_pos_t flagPos, flag_val_t flagValue) {
	reg_val_t registerValue = readRegister(dataAddress);
	registerValue = setFlag(registerValue, flagPos, flagValue);
	writeRegister(dataAddress, registerValue);
}

flag_val_t ADS1115::getFlag(reg_val_t registerValue, flag_pos_t flagPos) {
	return getFlag(registerValue, flagPos, 1);
}

flag_val_t ADS1115::getFlag(reg_val_t registerValue, flag_pos_t flagPos, flag_val_t customVal) {
	return ((registerValue >> flagPos) & 1) == 1 ? customVal : 0;
}

reg_val_t ADS1115::setFlag(reg_val_t registerValue, flag_pos_t flagPos, flag_val_t flagVal) {
	if (flagVal == 1) {
		return registerValue | (1 << flagPos);
	}

	return registerValue & (~(1 << flagPos));
}