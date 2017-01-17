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

#include <lightmodbus/core.h>
#include <lightmodbus/slave.h>
#include <lightmodbus/parser.h>

#define modbusParseRequest01 modbusParseRequest0102
#define modbusParseRequest02 modbusParseRequest0102
#define modbusParseRequest03 modbusParseRequest0304
#define modbusParseRequest04 modbusParseRequest0304
static uint8_t modbusBuildException( ModbusSlave *status, uint8_t function, uint8_t code );
static uint8_t modbusParseRequest0102( ModbusSlave *status, union ModbusParser *parser );
static uint8_t modbusParseRequest05( ModbusSlave *status, union ModbusParser *parser );
static uint8_t modbusParseRequest15( ModbusSlave *status, union ModbusParser *parser );
static uint8_t modbusParseRequest0304( ModbusSlave *status, union ModbusParser *parser );
static uint8_t modbusParseRequest06( ModbusSlave *status, union ModbusParser *parser );
static uint8_t modbusParseRequest16( ModbusSlave *status, union ModbusParser *parser );
static uint8_t modbusParseRequest22( ModbusSlave *status, union ModbusParser *parser );


#if ( LIGHTMODBUS_SLAVE_F0102 || LIGHTMODBUS_SLAVE_F01 || LIGHTMODBUS_SLAVE_F02 )
static uint8_t modbusParseRequest0102( ModbusSlave *status, union ModbusParser *parser )
{
	//Read multiple coils or discrete inputs
	//Using data from union pointer

	//Update frame length
	uint8_t frameLength = 8;
	uint8_t coil = 0;
	uint16_t i = 0;

	//Check if given pointers are valid
	if ( status == NULL || parser == NULL || ( parser->base.function != 1 && parser->base.function != 2 ) ) return MODBUS_ERROR_OTHER;

	//Don't do anything when frame is broadcasted
	//Base of the frame can be always safely checked, because main parser function takes care of that
	if ( parser->base.address == 0 ) return MODBUS_ERROR_OK;

	//Check if frame length is valid
	if ( status->request.length != frameLength )
		return modbusBuildException( status, parser->base.function, MODBUS_EXCEP_ILLEGAL_VAL );

	//Swap endianness of longer members (but not crc)
	uint16_t index = modbusSwapEndian( parser->request0102.index );
	uint16_t count = modbusSwapEndian( parser->request0102.count );

	//Check if coil is in valid range
	if ( count == 0 || count > 2000 )
		return modbusBuildException( status, parser->base.function, MODBUS_EXCEP_ILLEGAL_VAL );

	if ( index >= ( parser->base.function == 1 ? status->coilCount : status->discreteInputCount ) || \
		(uint32_t) index + (uint32_t) count > (uint32_t) ( parser->base.function == 1 ? status->coilCount : status->discreteInputCount ) )
			return modbusBuildException( status, parser->base.function, MODBUS_EXCEP_ILLEGAL_ADDR );

	//Respond
	frameLength = 5 + BITSTOBYTES( count );

	status->response.frame = (uint8_t *) calloc( frameLength, sizeof( uint8_t ) ); //Reallocate response frame memory to needed memory
	if ( status->response.frame == NULL ) return MODBUS_ERROR_ALLOC;
	union ModbusParser *builder = (union ModbusParser *) status->response.frame;

	//Set up basic response data
	builder->base.address = status->address;
	builder->base.function = parser->base.function;
	builder->response0102.length = BITSTOBYTES( count );

	//Copy registers to response frame
	for ( i = 0; i < count; i++ )
	{
		if ( ( coil = modbusMaskRead( parser->base.function == 1 ? status->coils : status->discreteInputs, \
			BITSTOBYTES( status->coilCount ), i + index ) ) == 255 )
				return MODBUS_ERROR_OTHER;
		if ( modbusMaskWrite( builder->response0102.values, builder->response0102.length, i, coil ) == 255 )
			return MODBUS_ERROR_OTHER;
	}

	//Calculate crc
	//That could be written as a single line, without the temporary variable, but avr-gcc doesn't like that
	//warning: dereferencing type-punned pointer will break strict-aliasing rules
	uint16_t *crc = (uint16_t*)( builder->frame + frameLength - 2 );
	*crc = modbusCRC( builder->frame, frameLength - 2 );

	//Set frame length - frame is ready
	status->response.length = frameLength;
	return MODBUS_ERROR_OK;
}
#endif

#if ( LIGHTMODBUS_SLAVE_F05 )
static uint8_t modbusParseRequest05( ModbusSlave *status, union ModbusParser *parser )
{
	//Write single coil
	//Using data from union pointer

	//Update frame length
	uint8_t frameLength = 8;

	//Check if given pointers are valid
	if ( status == NULL || parser == NULL ) return MODBUS_ERROR_OTHER;

	//Check if frame length is valid
	if ( status->request.length != frameLength )
	{
		if ( parser->base.address != 0 ) return modbusBuildException( status, 5, MODBUS_EXCEP_ILLEGAL_VAL );
		return MODBUS_ERROR_OK;
	}

	//Swap endianness of longer members (but not crc)
	uint16_t index = modbusSwapEndian( parser->request05.index );
	uint16_t value = modbusSwapEndian( parser->request05.value );

	//Check if coil value is valid
	if ( value != 0x0000 && value != 0xFF00 )
	{
		//Illegal data address error
		if ( parser->base.address != 0 ) return modbusBuildException( status, 5, MODBUS_EXCEP_ILLEGAL_VAL );
		return MODBUS_ERROR_OK;
	}

	//Check if coil is in valid range
	if ( index >= status->coilCount )
	{
		//Illegal data address error
		if ( parser->base.address != 0 ) return modbusBuildException( status, 5, MODBUS_EXCEP_ILLEGAL_ADDR );
		return MODBUS_ERROR_OK;
	}

	//Check if reg is allowed to be written
	if ( modbusMaskRead( status->coilMask, status->coilMaskLength, index ) == 1 )
	{
		//Slave failure exception
		if ( parser->base.address != 0 ) return modbusBuildException( status, 5, MODBUS_EXCEP_SLAVE_FAIL );
		return MODBUS_ERROR_OK;
	}

	//Respond
	frameLength = 8;

	status->response.frame = (uint8_t *) calloc( frameLength, sizeof( uint8_t ) ); //Reallocate response frame memory to needed memory
	if ( status->response.frame == NULL ) return MODBUS_ERROR_ALLOC;
	union ModbusParser *builder = (union ModbusParser *) status->response.frame;

	//After all possible exceptions, write coils
	if ( modbusMaskWrite( status->coils, BITSTOBYTES( status->coilCount ), index, value == 0xFF00 ) == 255 )
		return MODBUS_ERROR_OTHER;

	//Do not respond when frame is broadcasted
	if ( parser->base.address == 0 ) return MODBUS_ERROR_OK;

	//Set up basic response data
	builder->base.address = status->address;
	builder->base.function = parser->base.function;
	builder->response05.index = parser->request05.index;
	builder->response05.value = parser->request05.value;

	//Calculate crc
	builder->response05.crc = modbusCRC( builder->frame, frameLength - 2 );

	//Set frame length - frame is ready
	status->response.length = frameLength;
	return MODBUS_ERROR_OK;
}
#endif

#if ( LIGHTMODBUS_SLAVE_F15 )
static uint8_t modbusParseRequest15( ModbusSlave *status, union ModbusParser *parser )
{
	//Write multiple coils
	//Using data from union pointer

	//Update frame length
	uint16_t i = 0;
	uint8_t frameLength;
	uint8_t coil = 0;

	//Check if given pointers are valid
	if ( status == NULL || parser == NULL ) return MODBUS_ERROR_OTHER;

	//Check if frame length is valid
	if ( status->request.length >= 7u )
	{
		frameLength = 9 + parser->request15.length;
		if ( status->request.length != frameLength )
		{
			if ( parser->base.address != 0 ) return modbusBuildException( status, 15, MODBUS_EXCEP_ILLEGAL_VAL );
			return MODBUS_ERROR_OK;
		}
	}
	else
	{
		if ( parser->base.address != 0 ) return modbusBuildException( status, 15, MODBUS_EXCEP_ILLEGAL_VAL );
		return MODBUS_ERROR_OK;
	}

	//Swap endianness of longer members (but not crc)
	uint16_t index = modbusSwapEndian( parser->request15.index );
	uint16_t count = modbusSwapEndian( parser->request15.count );

	//Data checks
	if ( parser->request15.length == 0 || \
		count == 0 || \
		BITSTOBYTES( count ) != parser->request15.length || \
		count > 1968 )
	{
		//Illegal data value error
		if ( parser->base.address != 0 ) return modbusBuildException( status, 15, MODBUS_EXCEP_ILLEGAL_VAL );
		return MODBUS_ERROR_OK;
	}

	if ( index >= status->coilCount || \
		(uint32_t) index + (uint32_t) count > (uint32_t) status->coilCount )
	{
		//Illegal data address error
		if ( parser->base.address != 0 ) return modbusBuildException( status, 15, MODBUS_EXCEP_ILLEGAL_ADDR );
		return MODBUS_ERROR_OK;
	}

	//Check for write protection
	for ( i = 0; i < count; i++ )
		if ( modbusMaskRead( status->coilMask, status->coilMaskLength, index + i ) == 1 )
		{
			//Slave failure exception
			if ( parser->base.address != 0 ) return modbusBuildException( status, 15, MODBUS_EXCEP_SLAVE_FAIL );
			return MODBUS_ERROR_OK;
		}

	//Respond
	frameLength = 8;

	status->response.frame = (uint8_t *) calloc( frameLength, sizeof( uint8_t ) ); //Reallocate response frame memory to needed memory
	if ( status->response.frame == NULL ) return MODBUS_ERROR_ALLOC;
	union ModbusParser *builder = (union ModbusParser *) status->response.frame; //Allocate memory for builder union

	//After all possible exceptions write values to registers
	for ( i = 0; i < count; i++ )
	{
		if ( ( coil = modbusMaskRead( parser->request15.values, parser->request15.length, i ) ) == 255 ) return MODBUS_ERROR_OTHER;
		if ( modbusMaskWrite( status->coils, BITSTOBYTES( status->coilCount ), index + i, coil ) == 255 ) return MODBUS_ERROR_OTHER;
	}

	//Do not respond when frame is broadcasted
	if ( parser->base.address == 0 ) return MODBUS_ERROR_OK;

	//Set up basic response data
	builder->base.address = status->address;
	builder->base.function = parser->base.function;
	builder->response15.index = parser->request15.index;
	builder->response15.count = parser->request15.count;

	//Calculate crc
	builder->response15.crc = modbusCRC( builder->frame, frameLength - 2 );

	//Set frame length - frame is ready
	status->response.length = frameLength;
	return 0;
}
#endif

#if ( LIGHTMODBUS_SLAVE_F0304 || LIGHTMODBUS_SLAVE_F03 || LIGHTMODBUS_SLAVE_F04 )
static uint8_t modbusParseRequest0304( ModbusSlave *status, union ModbusParser *parser )
{
	//Read multiple holding registers or input registers
	//Using data from union pointer

	//Update frame length
	uint8_t frameLength = 8;
	uint8_t i = 0;

	//Check if given pointers are valid
	if ( status == NULL || parser == NULL || ( parser->base.function != 3 && parser->base.function != 4 ) ) return MODBUS_ERROR_OTHER;

	//Don't do anything when frame is broadcasted
	//Base of the frame can be always safely checked, because main parser function takes care of that
	if ( parser->base.address == 0 ) return MODBUS_ERROR_OK;

	//Check if frame length is valid
	if ( status->request.length != frameLength )
	{
		return modbusBuildException( status, parser->base.function, MODBUS_EXCEP_ILLEGAL_VAL );
	}

	//Swap endianness of longer members (but not crc)
	uint16_t index = modbusSwapEndian( parser->request0304.index );
	uint16_t count = modbusSwapEndian( parser->request0304.count );

	//Check if reg is in valid range
	if ( count == 0 || count > 125 )
	{
		//Illegal data value error
		return modbusBuildException( status, parser->base.function, MODBUS_EXCEP_ILLEGAL_VAL );
	}

	if ( index >= ( parser->base.function == 3 ? status->registerCount : status->inputRegisterCount ) || \
		(uint32_t) index + (uint32_t) count > \
		(uint32_t) ( parser->base.function == 3 ? status->registerCount : status->inputRegisterCount ) )
	{
		//Illegal data address exception
		return modbusBuildException( status, parser->base.function, MODBUS_EXCEP_ILLEGAL_ADDR );
	}

	//Respond
	frameLength = 5 + ( count << 1 );

	status->response.frame = (uint8_t *) calloc( frameLength, sizeof( uint8_t ) ); //Reallocate response frame memory to needed memory
	if ( status->response.frame == NULL )return MODBUS_ERROR_ALLOC;
	union ModbusParser *builder = (union ModbusParser *) status->response.frame;

	//Set up basic response data
	builder->response0304.address = status->address;
	builder->response0304.function = parser->request0304.function;
	builder->response0304.length = count << 1;

	//Copy registers to response frame
	for ( i = 0; i < count; i++ )
		builder->response0304.values[i] = modbusSwapEndian( ( parser->base.function == 3 ? status->registers : status->inputRegisters )[index + i] );

	//Calculate crc
	builder->response0304.values[count] = modbusCRC( builder->frame, frameLength - 2 );

	//Set frame length - frame is ready
	status->response.length = frameLength;
	return MODBUS_ERROR_OK;
}
#endif

#if ( LIGHTMODBUS_SLAVE_F06 )
static uint8_t modbusParseRequest06( ModbusSlave *status, union ModbusParser *parser )
{
	//Write single holding reg
	//Using data from union pointer

	//Update frame length
	uint8_t frameLength = 8;

	//Check if given pointers are valid
	if ( status == NULL || parser == NULL ) return MODBUS_ERROR_OTHER;

	//Check if frame length is valid
	if ( status->request.length != frameLength )
	{
		if ( parser->base.address != 0 ) return modbusBuildException( status, 6, MODBUS_EXCEP_ILLEGAL_VAL );
		return MODBUS_ERROR_OK;
	}

	//Swap endianness of longer members (but not crc)
	uint16_t index = modbusSwapEndian( parser->request06.index );
	uint16_t value = modbusSwapEndian( parser->request06.value );

	//Check if reg is in valid range
	if ( index >= status->registerCount )
	{
		//Illegal data address exception
		if ( parser->base.address != 0 ) return modbusBuildException( status, 6, MODBUS_EXCEP_ILLEGAL_ADDR );
		return MODBUS_ERROR_OK;
	}

	//Check if reg is allowed to be written
	if ( modbusMaskRead( status->registerMask, status->registerMaskLength, index ) == 1 )
	{
		//Slave failure exception
		if ( parser->base.address != 0 ) return modbusBuildException( status, 6, MODBUS_EXCEP_SLAVE_FAIL );
		return MODBUS_ERROR_OK;
	}

	//Respond
	frameLength = 8;

	status->response.frame = (uint8_t *) calloc( frameLength, sizeof( uint8_t ) ); //Reallocate response frame memory to needed memory
	if ( status->response.frame == NULL ) return MODBUS_ERROR_ALLOC;
	union ModbusParser *builder = (union ModbusParser *) status->response.frame;

	//After all possible exceptions, write reg
	status->registers[index] = value;

	//Do not respond when frame is broadcasted
	if ( parser->base.address == 0 ) return MODBUS_ERROR_OK;

	//Set up basic response data
	builder->response06.address = status->address;
	builder->response06.function = parser->request06.function;
	builder->response06.index = parser->request06.index;
	builder->response06.value = modbusSwapEndian( status->registers[index] );

	//Calculate crc
	builder->response06.crc = modbusCRC( builder->frame, frameLength - 2 );

	//Set frame length - frame is ready
	status->response.length = frameLength;
	return MODBUS_ERROR_OK;
}
#endif

#if ( LIGHTMODBUS_SLAVE_F16 )
static uint8_t modbusParseRequest16( ModbusSlave *status, union ModbusParser *parser )
{
	//Write multiple holding registers
	//Using data from union pointer

	//Update frame length
	uint8_t i = 0;
	uint8_t frameLength;

	//Check if given pointers are valid
	if ( status == NULL || parser == NULL ) return MODBUS_ERROR_OTHER;

	//Check if frame length is valid
	if ( status->request.length >= 7u )
	{
		frameLength = 9 + parser->request16.length;
		if ( status->request.length != frameLength )
		{
			return modbusBuildException( status, 16, MODBUS_EXCEP_ILLEGAL_VAL );
			return MODBUS_ERROR_OK;
		}
	}
	else
	{
		if ( parser->base.address != 0 ) return modbusBuildException( status, 16, MODBUS_EXCEP_ILLEGAL_VAL );
		return MODBUS_ERROR_OK;
	}

	//Swap endianness of longer members (but not crc)
	uint16_t index = modbusSwapEndian( parser->request16.index );
	uint16_t count = modbusSwapEndian( parser->request16.count );

	//Data checks
	if ( parser->request16.length == 0 || \
		count == 0 || \
		count != ( parser->request16.length >> 1 ) || \
		count > 123 )
	{
		//Illegal data value error
		if ( parser->base.address != 0 ) return modbusBuildException( status, 16, MODBUS_EXCEP_ILLEGAL_VAL );
		return MODBUS_ERROR_OK;
	}

	if ( index >= status->registerCount || \
		(uint32_t) index + (uint32_t) count > (uint32_t) status->registerCount )
	{
		//Illegal data address error
		if ( parser->base.address != 0 ) return modbusBuildException( status, 16, MODBUS_EXCEP_ILLEGAL_ADDR );
		return MODBUS_ERROR_OK;
	}

	//Check for write protection
	for ( i = 0; i < count; i++ )
		if ( modbusMaskRead( status->registerMask, status->registerMaskLength, index + i ) == 1 )
		{
			//Slave failure exception
			if ( parser->base.address != 0 ) return modbusBuildException( status, 16, MODBUS_EXCEP_SLAVE_FAIL );
			return MODBUS_ERROR_OK;
		}

	//Respond
	frameLength = 8;

	status->response.frame = (uint8_t *) calloc( frameLength, sizeof( uint8_t ) ); //Reallocate response frame memory to needed memory
	if ( status->response.frame == NULL ) return MODBUS_ERROR_ALLOC;
	union ModbusParser *builder = (union ModbusParser *) status->response.frame;


	//After all possible exceptions, write values to registers
	for ( i = 0; i < count; i++ )
		status->registers[index + i] = modbusSwapEndian( parser->request16.values[i] );

	//Do not respond when frame is broadcasted
	if ( parser->base.address == 0 ) return MODBUS_ERROR_OK;

	//Set up basic response data
	builder->response16.address = status->address;
	builder->response16.function = parser->request16.function;
	builder->response16.index = parser->request16.index;
	builder->response16.count = parser->request16.count;

	//Calculate crc
	builder->response16.crc = modbusCRC( builder->frame, frameLength - 2 );

	//Set frame length - frame is ready
	status->response.length = frameLength;
	return MODBUS_ERROR_OK;
}
#endif

#if ( LIGHTMODBUS_SLAVE_F22 )
static uint8_t modbusParseRequest22( ModbusSlave *status, union ModbusParser *parser )
{
	//Mask write single holding reg
	//Using data from union pointer

	//Update frame length
	uint8_t frameLength = 10;

	//Check if given pointers are valid
	if ( status == NULL || parser == NULL ) return MODBUS_ERROR_OTHER;

	//Check if frame length is valid
	if ( status->request.length != frameLength )
	{
		if ( parser->base.address != 0 ) return modbusBuildException( status, 22, MODBUS_EXCEP_ILLEGAL_VAL );
		return MODBUS_ERROR_OK;
	}

	//Check frame crc
	if ( modbusCRC( parser->frame, frameLength - 2 ) != parser->request22.crc ) return MODBUS_ERROR_CRC;

	//Swap endianness of longer members (but not crc)
	uint16_t index = modbusSwapEndian( parser->request22.index );
	uint16_t andmask = modbusSwapEndian( parser->request22.andmask );
	uint16_t ormask = modbusSwapEndian( parser->request22.ormask );

	//Check if reg is in valid range
	if ( index >= status->registerCount )
	{
		//Illegal data address exception
		if ( parser->base.address != 0 ) return modbusBuildException( status, 22, MODBUS_EXCEP_ILLEGAL_ADDR );
		return MODBUS_ERROR_OK;
	}

	//Check if reg is allowed to be written
	if ( modbusMaskRead( status->registerMask, status->registerMaskLength, index ) == 1 )
	{
		//Slave failure exception
		if ( parser->base.address != 0 ) return modbusBuildException( status, 22, MODBUS_EXCEP_SLAVE_FAIL );
		return MODBUS_ERROR_OK;
	}

	//Respond
	frameLength = 10;

	status->response.frame = (uint8_t *) calloc( frameLength, sizeof( uint8_t ) ); //Reallocate response frame memory to needed memory
	if ( status->response.frame == NULL ) return MODBUS_ERROR_ALLOC;
	union ModbusParser *builder = (union ModbusParser *) status->response.frame;

	//After all possible exceptions, write reg
	status->registers[index] = ( status->registers[index] & andmask ) | ( ormask & ~andmask );

	//Do not respond when frame is broadcasted
	if ( parser->base.address == 0 ) return MODBUS_ERROR_OK;

	//Set up basic response data
	builder->response22.address = status->address;
	builder->response22.function = parser->request22.function;
	builder->response22.index = parser->request22.index;
	builder->response22.andmask = parser->request22.andmask;
	builder->response22.ormask = parser->request22.ormask;

	//Calculate crc
	builder->response22.crc = modbusCRC( builder->frame, frameLength - 2 );

	//Set frame length - frame is ready
	status->response.length = frameLength;
	return MODBUS_ERROR_OK;
}
#endif

static uint8_t modbusBuildException( ModbusSlave *status, uint8_t function, uint8_t code )
{
	//Generates modbus exception frame in allocated memory frame
	//Returns generated frame length

	//Check if given pointer is valid
	if ( status == NULL || code == 0 ) return MODBUS_ERROR_OTHER;

	//Reallocate frame memory
	status->response.frame = (uint8_t *) calloc( 5, sizeof( uint8_t ) );
	if ( status->response.frame == NULL ) return MODBUS_ERROR_ALLOC;
	union ModbusParser *exception = (union ModbusParser *) status->response.frame;

	//Setup exception frame
	exception->exception.address = status->address;
	exception->exception.function = ( 1 << 7 ) | function;
	exception->exception.code = code;
	exception->exception.crc = modbusCRC( exception->frame, 3 );

	//Set frame length - frame is ready
	status->response.length = 5;

	//So, user should rather know, that master slave had to throw exception, right?
	//That's the reason exception should be thrown - just like that, an information
	return MODBUS_ERROR_EXCEPTION;
}

uint8_t modbusParseRequest( ModbusSlave *status )
{
	//Parse and interpret given modbus frame on slave-side
	uint8_t err = 0;

	//Check if given pointer is valid
	if ( status == NULL ) return MODBUS_ERROR_OTHER;

	//Reset response frame status
	status->response.length = 0;

	//If there is memory allocated for response frame - free it
	free( status->response.frame );
	status->response.frame = NULL;

	//If user tries to parse an empty frame return error
	//That enables us to ommit the check in each parsing function
	if ( status->request.length < 4u || status->request.frame == NULL ) return MODBUS_ERROR_OTHER;

	//Check CRC
	if ( *( (uint16_t*)( status->request.frame + status->request.length - 2 ) )\
		!= modbusCRC( status->request.frame, status->request.length - 2 ) )
			return MODBUS_ERROR_CRC;


	union ModbusParser *parser = (union ModbusParser *) status->request.frame;

	//If frame is not broadcasted and address doesn't match skip parsing
	if ( parser->base.address != status->address && parser->base.address != 0 )
		return MODBUS_ERROR_OK;

	switch ( parser->base.function )
	{
		case 1: //Read multiple coils
		case 2: //Read multiple discrete inputs
			if ( LIGHTMODBUS_SLAVE_F0102 || LIGHTMODBUS_SLAVE_F01 || LIGHTMODBUS_SLAVE_F02 )
				err = modbusParseRequest0102( status, parser );
			else err = MODBUS_ERROR_PARSE;
			break;

		case 3: //Read multiple holding registers
		case 4: //Read multiple input registers
			if (  LIGHTMODBUS_SLAVE_F0304 || LIGHTMODBUS_SLAVE_F03 || LIGHTMODBUS_SLAVE_F04 )
				err = modbusParseRequest0304( status, parser );
			else err = MODBUS_ERROR_PARSE;
			break;

		case 5: //Write single coil
			if ( LIGHTMODBUS_SLAVE_F05 ) err = modbusParseRequest05( status, parser );
			else err = MODBUS_ERROR_PARSE;
			break;

		case 6: //Write single holding reg
			if ( LIGHTMODBUS_SLAVE_F06 ) err = modbusParseRequest06( status, parser );
			else err = MODBUS_ERROR_PARSE;
			break;

		case 15: //Write multiple coils
			if ( LIGHTMODBUS_SLAVE_F15 ) err = modbusParseRequest15( status, parser );
			else err = MODBUS_ERROR_PARSE;
			break;

		case 16: //Write multiple holding registers
			if ( LIGHTMODBUS_SLAVE_F16 ) err = modbusParseRequest16( status, parser );
			else err = MODBUS_ERROR_PARSE;
			break;

		case 22: //Mask write single register
			if ( LIGHTMODBUS_SLAVE_F22 ) err = modbusParseRequest22( status, parser );
			else err = MODBUS_ERROR_PARSE;
			break;

		default:
			err = MODBUS_ERROR_PARSE;
			break;
	}

	if ( err == MODBUS_ERROR_PARSE )
		if ( parser->base.address != 0 ) err = modbusBuildException( status, parser->base.function, MODBUS_EXCEP_ILLEGAL_FUNC );

	return err;
}

uint8_t modbusSlaveInit( ModbusSlave *status )
{
	//Very basic init of slave side
	//User has to modify pointers etc. himself

	//Check if given pointer is valid
	if ( status == NULL ) return MODBUS_ERROR_OTHER;

	//Reset response frame status
	status->request.length = 0;
	status->request.frame = NULL;
	status->response.length = 0;
	status->response.frame = NULL;

	if ( status->address == 0 )
	{
		status->address = 1;
		return MODBUS_ERROR_OTHER;
	}

	//Some safety checks
	if ( status->registerCount == 0 || status->registers == NULL )
	{
		status->registerCount = 0;
		status->registers = NULL;
	}

	if ( status->coilCount == 0 || status->coils == NULL )
	{
		status->coilCount = 0;
		status->coils = NULL;
	}

	if ( status->discreteInputCount == 0 || status->discreteInputs == NULL )
	{
		status->discreteInputCount = 0;
		status->discreteInputs = NULL;
	}

	if ( status->inputRegisterCount == 0 || status->inputRegisters == NULL )
	{
		status->inputRegisterCount = 0;
		status->inputRegisters = NULL;
	}

	return MODBUS_ERROR_OK;
}

uint8_t modbusSlaveEnd( ModbusSlave *status )
{
	//Check if given pointer is valid
	if ( status == NULL ) return MODBUS_ERROR_OTHER;

	//Free memory
	free( status->response.frame );

	return MODBUS_ERROR_OK;
}
