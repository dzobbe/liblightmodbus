/*
	liblightmodbus - a lightweight, multiplatform Modbus library
	Copyright (C) 2016	Jacek Wieczorek <mrjjot@gmail.com>

	This file is part of liblightmodbus.

	Liblightmodbus is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	Liblightmodbus is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef LIGHTMODBUS_SLAVE_BASE_H
#define LIGHTMODBUS_SLAVE_BASE_H

#include <inttypes.h>

#include "core.h"
#include "parser.h"

typedef struct
{
	uint8_t address; //Slave address

	uint16_t *registers; //Slave holding registers
	uint16_t registerCount; //Slave register count

	uint8_t *coils; //Slave coils
	uint16_t coilCount; //Slave coil count

	uint8_t *discreteInputs; //Slave discrete input
	uint16_t discreteInputCount; //Slave discrete input count

	uint8_t *registerMask; //Masks for register write protection (bit of value 1 - write protection)
	uint16_t registerMaskLength; //Masks length (each byte covers 8 registers)
	uint8_t *coilMask; //Masks for coil write protection (bit of value 1 - write protection)
	uint16_t coilMaskLength; //Masks length (each byte covers 8 coils)

	uint16_t *inputRegisters; //Slave input registers
	uint16_t inputRegisterCount; //Slave input count

	struct //Slave response formatting status
	{
		uint8_t *frame;
		uint8_t length;
	} response;

	struct //Request from master should be put here
	{
		uint8_t *frame;
		uint8_t length;
	} request;

} ModbusSlave; //Type containing slave device configuration data

//Enabling modules in compilation process (use makefile to automate this process)
#ifndef LIGHTMODBUS_SLAVE_F0102
#define LIGHTMODBUS_SLAVE_F0102 0
#endif
#ifndef LIGHTMODBUS_SLAVE_F01
#define LIGHTMODBUS_SLAVE_F01 0
#endif
#ifndef LIGHTMODBUS_SLAVE_F02
#define LIGHTMODBUS_SLAVE_F02 0
#endif
#ifndef LIGHTMODBUS_SLAVE_F0304
#define LIGHTMODBUS_SLAVE_F0304 0
#endif
#ifndef LIGHTMODBUS_SLAVE_F03
#define LIGHTMODBUS_SLAVE_F03 0
#endif
#ifndef LIGHTMODBUS_SLAVE_F04
#define LIGHTMODBUS_SLAVE_F04 0
#endif
#ifndef LIGHTMODBUS_SLAVE_F05
#define LIGHTMODBUS_SLAVE_F05 0
#endif
#ifndef LIGHTMODBUS_SLAVE_F06
#define LIGHTMODBUS_SLAVE_F06 0
#endif
#ifndef LIGHTMODBUS_SLAVE_F15
#define LIGHTMODBUS_SLAVE_F15 0
#endif
#ifndef LIGHTMODBUS_SLAVE_F16
#define LIGHTMODBUS_SLAVE_F16 0
#endif
#ifndef LIGHTMODBUS_SLAVE_F22
#define LIGHTMODBUS_SLAVE_F22 0
#endif

//Function prototypes
extern uint8_t modbusParseRequest( ModbusSlave *status ); //Parse and interpret given modbus frame on slave-side
extern uint8_t modbusSlaveInit( ModbusSlave *status ); //Very basic init of slave side
extern uint8_t modbusSlaveEnd( ModbusSlave *status ); //Free memory used by slave

#endif
