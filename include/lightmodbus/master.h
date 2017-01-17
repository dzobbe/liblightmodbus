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

#ifndef LIGHTMODBUS_MASTER_BASE_H
#define LIGHTMODBUS_MASTER_BASE_H

#include <inttypes.h>

#include "core.h"

typedef struct
{
	uint8_t predictedResponseLength; //If everything goes fine, slave will return this amout of data

	struct //Formatted request for slave
	{
		uint8_t *frame;
		uint8_t length;
	} request;

	struct //Response from slave should be put here
	{
		uint8_t *frame;
		uint8_t length;
	} response;

	struct //Data read from slave
	{
		uint8_t address; //Addres of slave
		uint16_t index; //Address of the first element (in slave device)
		uint16_t count; //Count of data units (coils, registers, etc.)
		uint8_t length; //Length of data in bytes
		uint8_t type; //Type of data
		uint8_t function; //Function that accessed the data
		//Two separate pointers are used in case pointer size differed between types (possible on some weird architectures)
		uint8_t *coils; //Received data
		uint16_t *regs; //And the same received data, but converted to uint16_t pointer for convenience
	} data;

	struct //Exceptions read are stored in this structure
	{
		uint8_t address; //Device address
		uint8_t function; //In which function exception occured
		uint8_t code; //Exception code
	} exception;

} ModbusMaster; //Type containing master device configuration data

#define MODBUS_HOLDING_REGISTER 1
#define MODBUS_INPUT_REGISTER 2
#define MODBUS_COIL 4
#define MODBUS_DISCRETE_INPUT 8

#ifndef LIGHTMODBUS_MASTER_F0102
#define LIGHTMODBUS_MASTER_F0102 0
#endif
#ifndef LIGHTMODBUS_MASTER_F01
#define LIGHTMODBUS_MASTER_F01 0
#endif
#ifndef LIGHTMODBUS_MASTER_F02
#define LIGHTMODBUS_MASTER_F02 0
#endif
#ifndef LIGHTMODBUS_MASTER_F0304
#define LIGHTMODBUS_MASTER_F0304 0
#endif
#ifndef LIGHTMODBUS_MASTER_F03
#define LIGHTMODBUS_MASTER_F03 0
#endif
#ifndef LIGHTMODBUS_MASTER_F04
#define LIGHTMODBUS_MASTER_F04 0
#endif
#ifndef LIGHTMODBUS_MASTER_F05
#define LIGHTMODBUS_MASTER_F05 0
#endif
#ifndef LIGHTMODBUS_MASTER_F06
#define LIGHTMODBUS_MASTER_F06 0
#endif
#ifndef LIGHTMODBUS_MASTER_F15
#define LIGHTMODBUS_MASTER_F15 0
#endif
#ifndef LIGHTMODBUS_MASTER_F16
#define LIGHTMODBUS_MASTER_F16 0
#endif
#ifndef LIGHTMODBUS_MASTER_F22
#define LIGHTMODBUS_MASTER_F22 0
#endif

//Functions for building requests
#define modbusBuildRequest01( status, address, index, count ) modbusBuildRequest0102( (status), 01, (address), (index), (count) )
#define modbusBuildRequest02( status, address, index, count ) modbusBuildRequest0102( (status), 02, (address), (index), (count) )
extern uint8_t modbusBuildRequest0102( ModbusMaster *status, uint8_t function, uint8_t address, uint16_t index, uint16_t count );
extern uint8_t modbusBuildRequest05( ModbusMaster *status, uint8_t address, uint16_t index, uint16_t value );
extern uint8_t modbusBuildRequest15( ModbusMaster *status, uint8_t address, uint16_t index, uint16_t count, uint8_t *values );
#define modbusBuildRequest03( status, address, index, count ) modbusBuildRequest0304( (status), 3, (address), (index), (count) )
#define modbusBuildRequest04( status, address, index, count ) modbusBuildRequest0304( (status), 4, (address), (index), (count) )
extern uint8_t modbusBuildRequest0304( ModbusMaster *status, uint8_t function, uint8_t address, uint16_t index, uint16_t count );
extern uint8_t modbusBuildRequest06( ModbusMaster *status, uint8_t address, uint16_t index, uint16_t value );
extern uint8_t modbusBuildRequest16( ModbusMaster *status, uint8_t address, uint16_t index, uint16_t count, uint16_t *values );
extern uint8_t modbusBuildRequest22( ModbusMaster *status, uint8_t address, uint16_t index, uint16_t andmask, uint16_t ormask );
extern uint8_t modbusParseResponse( ModbusMaster *status );
extern uint8_t modbusMasterInit( ModbusMaster *status );
extern uint8_t modbusMasterEnd( ModbusMaster *status ); //Free memory used by master

#endif
