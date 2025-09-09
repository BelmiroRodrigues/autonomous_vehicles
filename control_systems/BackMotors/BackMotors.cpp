#include "BackMotors.hpp"

BackMotors::BackMotors(){
	i2c_device = "/dev/i2c-1";
	_fdMotor = open(i2c_device.c_str(), O_RDWR);
	if (_fdMotor < 0)
		throw std::runtime_error("Error open I2C");
	if ( ioctl(_fdMotor, I2C_SLAVE, _motorAddr) < 0){
		close (_fdMotor);
		throw std::runtime_error("Erro ao configurar endereço I2C do motor.");
	}
	std::cout << "JetCar inicializado com sucesso!" << std::endl;
}

BackMotors::~BackMotors(){
	setSpeed(0);
	close(_fdMotor);
	std::cout << "destructor call\n";
}

bool BackMotors::init_motors(){
	try{
		// Configure motor controller
		writeByteData(_fdMotor, 0x00, 0x20);

		// Set frequency to 60Hz
		int 	preScale;
		uint8_t oldMode, newMode;

		preScale = static_cast<int>(std::floor(25000000.0 / 4096.0 / 240 - 1));
		oldMode = readByteData(_fdMotor, 0x00);
		newMode = (oldMode & 0x7F) | 0x10;

		 // Configurar o novo modo e frequência
		writeByteData(_fdMotor, 0x00, newMode);
		writeByteData(_fdMotor, 0xFE, preScale);
		writeByteData(_fdMotor, 0x00, oldMode);

		usleep(5000);

		// Ativar auto-incremento
		writeByteData(_fdMotor, 0x00, oldMode | 0xa1);
		return true;
	}
	catch(const std::exception &e){
		std::cerr << "Erro na inicialização dos motores: " << e.what() << std::endl;
		return false;
	}
}

bool BackMotors::setMotorPwm(const int channel, int value){
	value = std::min(std::max(value, 0), 4095);
	try{
		writeByteData(_fdMotor, 0x06 + 4 * channel, 0);
		writeByteData(_fdMotor, 0x07 + 4 * channel, 0);
		writeByteData(_fdMotor, 0x08 + 4 * channel, value & 0xFF);
		writeByteData(_fdMotor, 0x09 + 4 * channel, value >> 8);
		return true;

	}
	catch(const std::exception &e){
		std::cerr << "Erro PWM dos motores: " << e.what() << std::endl;
		return false;
	}
}

void BackMotors::setSpeed(int speed) {
    int leftSpeed = std::max(-100, std::min(100, speed));
    int rightSpeed = std::max(-100, std::min(100, speed));

    // Converte para PWM (0-4095) e aplica compensação
    int pwmLeft  = static_cast<int>((std::abs(leftSpeed)  / 100.0 * 4095));
    int pwmRight = static_cast<int>((std::abs(rightSpeed) / 100.0 * 4095));

	pwmLeft  = std::min(pwmLeft,  4095) * _compLeft;
    pwmRight = std::min(pwmRight, 4095) * _compRight;

    if (leftSpeed > 0) { // forward
        setMotorPwm(0, pwmLeft); // IN1
        setMotorPwm(1, 0);       // IN2
        setMotorPwm(2, pwmLeft); // ENA
    } else if (leftSpeed < 0) { // backward
        setMotorPwm(0, pwmLeft);
        setMotorPwm(1, pwmLeft);
        setMotorPwm(2, 0);
    } else {
        setMotorPwm(0, 0);
        setMotorPwm(1, 0);
        setMotorPwm(2, 0);
    }

    if (rightSpeed > 0) { // forward
        setMotorPwm(5, pwmRight); // IN3
        setMotorPwm(6, 0);        // IN4
        setMotorPwm(7, pwmRight); // ENB
    } else if (rightSpeed < 0) { // backward
        setMotorPwm(5, 0);
        setMotorPwm(6, pwmRight);
        setMotorPwm(7, pwmRight);
    } else {
        setMotorPwm(5, 0);
        setMotorPwm(6, 0);
        setMotorPwm(7, 0);
    }
}


void BackMotors::writeByteData(int fd, uint8_t reg, uint8_t value) {
    uint8_t buffer[2] = {reg, value};
    if (write(fd, buffer, 2) != 2) {
        throw std::runtime_error("Erro ao escrever no dispositivo I2C.");
    }
}

uint8_t BackMotors::readByteData(int fd, uint8_t reg){
	if(write(fd, &reg, 1) != 1)
		throw std::runtime_error("Erro ao enviar o registrador ao dispositivo I2C.");
	uint8_t value;
	if (read(fd, &value, 1) != 1)
		throw std::runtime_error("Erro ao ler o registrador ao dispositivo I2C.");
	return value;
}