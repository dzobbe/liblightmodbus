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

#include <lightmodbus/master.h>
#include <lightmodbus/core.h>
#include <lightmodbus/parser.h>

//Functions for parsing responses
#define modbusParseResponse01 modbusParseResponse0102
#define modbusParseResponse02 modbusParseResponse0102
#define modbusParseResponse03 modbusParseResponse0304
#define modbusParseResponse04 modbusParseResponse0304
static uint8_t modbusParseResponse0102( ModbusMaster *status, union ModbusParser *parser, union ModbusParser *requestParser );
static uint8_t modbusParseResponse05( ModbusMaster *status, union ModbusParser *parser, union ModbusParser *requestParser );
static uint8_t modbusParseResponse15( ModbusMaster *status, union ModbusParser *parser, union ModbusParser *requestParser );
static uint8_t modbusParseResponse0304( ModbusMaster *status, union ModbusParser *parser, union ModbusParser *requestParser );
static uint8_t modbusParseResponse06( ModbusMaster *status, union ModbusParser *parser, union ModbusParser *requestParser );
static uint8_t modbusParseResponse16( ModbusMaster *status, union ModbusParser *parser, union ModbusParser *requestParser );
static uint8_t modbusParseResponse22( ModbusMaster *status, union ModbusParser *parser, union ModbusParser *requestParser );


#if ( LIGHTMODBUS_MASTER_F0102 || LIGHTMODBUS_MASTER_F01 || LIGHTMODBUS_MASTER_F02 )
static uint8_t modbusBuildRequest0102( ModbusMaster *status, uint8_t function, uint8_t address, uint16_t index, uint16_t count )
{
	//Build request01 frame, to send it so slave
	//Read multiple coils

	//Set frame length
	uint8_t frameLength = 8;

	//Check if given pointer is valid
	if ( status == NULL || ( function != 1 && function != 2 ) ) return MODBUS_ERROR_OTHER;

	//Set output frame length to 0 (in case of interrupts)
	status->request.length = 0;
	status->predictedResponseLength = 0;

	//Check values pointer
	if ( count == 0 || count > 2000 || address == 0 ) return MODBUS_ERROR_OTHER;

	//Reallocate memory for final frame
	free( status->request.frame );
	status->request.frame = (uint8_t *) calloc( frameLength, sizeof( uint8_t ) );
	if ( status->request.frame == NULL ) return MODBUS_ERROR_ALLOC;
	union ModbusParser *builder = (union ModbusParser *) status->request.frame;

	builder->base.address = address;
	builder->base.function = function;
	builder->request0102.index = modbusSwapEndian( index );
	builder->request0102.count = modbusSwapEndian( count );

	//Calculate crc
	builder->request0102.crc = modbusCRC( builder->frame, frameLength - 2 );

	status->request.length = frameLength;
	status->predictedResponseLength = 4 + 1 + BITSTOBYTES( count );

	return MODBUS_ERROR_OK;
}

static uint8_t modbusParseResponse0102( ModbusMaster *status, union ModbusParser *parser, union ModbusParser *requestParser )
{
	//Parse slave response to request 01 (read multiple coils)

	uint8_t dataok = 1;

	//Check if given pointers are valid
	if ( status == NULL || parser == NULL || requestParser == NULL || ( parser->base.function != 1 && parser->base.function != 2 ) )
		return MODBUS_ERROR_OTHER;

	//Check if frame length is valid
	//Frame has to be at least 4 bytes long so byteCount can always be accessed in this case
	if ( status->response.length != 5 + parser->response0102.length || status->request.length != 8 ) return MODBUS_ERROR_FRAME;

	uint16_t count = modbusSwapEndian( requestParser->request0102.count );

	//Check between data sent to slave and received from slave
	dataok &= parser->base.address != 0;
	dataok &= parser->base.address == requestParser->base.address;
	dataok &= parser->base.function == requestParser->base.function;
	dataok &= parser->response0102.length != 0;
	dataok &= parser->response0102.length <= 250;
	dataok &= parser->response0102.length == BITSTOBYTES( count );

	//If data is bad abort parsing, and set error flag
	if ( !dataok ) return MODBUS_ERROR_FRAME;

	status->data.coils = (uint8_t*) calloc( BITSTOBYTES( count ), sizeof( uint8_t ) );
	status->data.regs = (uint16_t*) status->data.coils;
	if ( status->data.coils == NULL ) return MODBUS_ERROR_ALLOC;

	status->data.function = parser->base.function;
	status->data.address = parser->base.address;
	status->data.type = parser->base.function == 1 ? MODBUS_COIL : MODBUS_DISCRETE_INPUT;
	status->data.index = modbusSwapEndian( requestParser->request0102.index );
	status->data.count = count;
	memcpy( status->data.coils, parser->response0102.values, parser->response0102.length );
	status->data.length = parser->response0102.length;
	return MODBUS_ERROR_OK;
}
#endif

#if ( LIGHTMODBUS_MASTER_F05 )
static uint8_t modbusBuildRequest05( ModbusMaster *status, uint8_t address, uint16_t index, uint16_t value )
{
	//Build request05 frame, to send it so slave
	//Write single coil

	//Set frame length
	uint8_t frameLength = 8;

	//Check if given pointer is valid
	if ( status == NULL ) return MODBUS_ERROR_OTHER;

	//Set output frame length to 0 (in case of interrupts)
	status->request.length = 0;
	status->predictedResponseLength = 0;

	//Reallocate memory for final frame
	free( status->request.frame );
	status->request.frame = (uint8_t *) calloc( frameLength, sizeof( uint8_t ) );
	if ( status->request.frame == NULL ) return MODBUS_ERROR_ALLOC;
	union ModbusParser *builder = (union ModbusParser *) status->request.frame;

	value = ( value != 0 ) ? 0xFF00 : 0x0000;

	builder->base.address = address;
	builder->base.function = 5;
	builder->request05.index = modbusSwapEndian( index );
	builder->request05.value = modbusSwapEndian( value );

	//Calculate crc
	builder->request05.crc = modbusCRC( builder->frame, frameLength - 2 );

	status->request.length = frameLength;
	if ( address ) status->predictedResponseLength = 8;

	return MODBUS_ERROR_OK;
}

static uint8_t modbusParseResponse05( ModbusMaster *status, union ModbusParser *parser, union ModbusParser *requestParser )
{
	//Parse slave response to request 05 (write single coil)

	uint8_t dataok = 1;

	//Check if given pointers are valid
	if ( status == NULL || parser == NULL || requestParser == NULL ) return MODBUS_ERROR_OTHER;

	//Check frame lengths
	if ( status->response.length != 8 || status->request.length != 8 ) return MODBUS_ERROR_FRAME;

	//Check between data sent to slave and received from slave
	dataok &= parser->base.address == requestParser->base.address;
	dataok &= parser->base.function == requestParser->base.function;

	//If data is bad abort parsing, and set error flag
	if ( !dataok ) return MODBUS_ERROR_FRAME;

	status->data.coils = (uint8_t*) calloc( 1, sizeof( uint8_t ) );
	status->data.regs = (uint16_t*) status->data.coils;
	if ( status->data.coils == NULL ) return MODBUS_ERROR_ALLOC;
	status->data.function = 5;
	status->data.address = parser->base.address;
	status->data.type = MODBUS_COIL;
	status->data.index = modbusSwapEndian( requestParser->request05.index );
	status->data.count = 1;
	status->data.coils[0] = parser->response05.value != 0;
	status->data.length = 1;
	return MODBUS_ERROR_OK;
}
#endif

#if ( LIGHTMODBUS_MASTER_F15 )
static uint8_t modbusBuildRequest15( ModbusMaster *status, uint8_t address, uint16_t index, uint16_t count, uint8_t *values )
{
	//Build request15 frame, to send it so slave
	//Write multiple coils

	//Set frame length
	uint8_t frameLength = 9 + BITSTOBYTES( count );
	uint8_t i = 0;

	//Check if given pointer is valid
	if ( status == NULL ) return MODBUS_ERROR_OTHER;

	//Set output frame length to 0 (in case of interrupts)
	status->request.length = 0;
	status->predictedResponseLength = 0;

	//Check values pointer
	if ( values == NULL || count == 0 || count > 1968 ) return MODBUS_ERROR_OTHER;

	//Reallocate memory for final frame
	free( status->request.frame );
	status->request.frame = (uint8_t *) calloc( frameLength, sizeof( uint8_t ) );
	if ( status->request.frame == NULL ) return MODBUS_ERROR_ALLOC;
	union ModbusParser *builder = (union ModbusParser *) status->request.frame;

	builder->base.address = address;
	builder->base.function = 15;
	builder->request15.index = modbusSwapEndian( index );
	builder->request15.count = modbusSwapEndian( count );
	builder->request15.length = BITSTOBYTES( count );

	for ( i = 0; i < builder->request15.length; i++ )
		builder->request15.values[i] = values[i];


	//That could be written as a single line, without the temporary variable, but avr-gcc doesn't like that
	//warning: dereferencing type-punned pointer will break strict-aliasing rules
	uint16_t *crc = (uint16_t*)( builder->frame + frameLength - 2 );
	*crc = modbusCRC( builder->frame, frameLength - 2 );

	status->request.length = frameLength;
	if ( address ) status->predictedResponseLength = 4 + 4;

	return MODBUS_ERROR_OK;
}

static uint8_t modbusParseResponse15( ModbusMaster *status, union ModbusParser *parser, union ModbusParser *requestParser )
{
	//Parse slave response to request 15 (write multiple coils)

	uint8_t dataok = 1;

	//Check if given pointers are valid
	if ( status == NULL || parser == NULL || requestParser == NULL ) return MODBUS_ERROR_OTHER;

	//Check frame lengths
	if ( status->request.length < 7u || status->request.length != 9 + requestParser->request15.length ) return MODBUS_ERROR_FRAME;
	if ( status->response.length != 8 ) return MODBUS_ERROR_FRAME;

	//Check between data sent to slave and received from slave
	dataok &= parser->base.address == requestParser->base.address;
	dataok &= parser->base.function == requestParser->base.function;
	dataok &= parser->response15.index == requestParser->request15.index;
	dataok &= parser->response15.count == requestParser->request15.count;

	//If data is bad abort parsing, and set error flag
	if ( !dataok ) return MODBUS_ERROR_FRAME;

	status->data.address = parser->base.address;
	status->data.function = 15;
	status->data.type = MODBUS_COIL;
	status->data.index = modbusSwapEndian( parser->response15.index );
	status->data.count = modbusSwapEndian( parser->response15.count );
	status->data.length = 0;
	return MODBUS_ERROR_OK;
}
#endif

#if ( LIGHTMODBUS_MASTER_F0304 || LIGHTMODBUS_MASTER_F03 || LIGHTMODBUS_MASTER_F04 )
static uint8_t modbusBuildRequest0304( ModbusMaster *status, uint8_t function, uint8_t address, uint16_t index, uint16_t count )
{
	//Build request03 frame, to send it so slave
	//Read multiple holding registers

	//Set frame length
	uint8_t frameLength = 8;

	//Check if given pointer is valid
	if ( status == NULL || ( function != 3 && function != 4 ) ) return MODBUS_ERROR_OTHER;

	//Set output frame length to 0 (in case of interrupts)
	status->request.length = 0;
	status->predictedResponseLength = 0;

	//Check values pointer
	if ( count == 0 || count > 125 || address == 0 ) return MODBUS_ERROR_OTHER;

	//Reallocate memory for final frame
	free( status->request.frame );
	status->request.frame = (uint8_t *) calloc( frameLength, sizeof( uint8_t ) );
	if ( status->request.frame == NULL ) return MODBUS_ERROR_ALLOC;
	union ModbusParser *builder = (union ModbusParser *) status->request.frame;

	builder->base.address = address;
	builder->base.function = function;
	builder->request0304.index = modbusSwapEndian( index );
	builder->request0304.count = modbusSwapEndian( count );

	//Calculate crc
	builder->request0304.crc = modbusCRC( builder->frame, frameLength - 2 );

	status->request.length = frameLength;
	status->predictedResponseLength = 4 + 1 + ( count << 1 );
	return MODBUS_ERROR_OK;
}

static uint8_t modbusParseResponse0304( ModbusMaster *status, union ModbusParser *parser, union ModbusParser *requestParser )
{
	//Parse slave response to request 03
	//Read multiple holding registers

	uint8_t dataok = 1;
	uint8_t i = 0;

	//Check if given pointers are valid
	if ( status == NULL || parser == NULL || requestParser == NULL || ( parser->base.function != 3 && parser->base.function != 4 ) )
		return MODBUS_ERROR_OTHER;

	//Check if frame length is valid
	//Frame has to be at least 4 bytes long so byteCount can always be accessed in this case
	if ( status->response.length != 5 + parser->response0304.length || status->request.length != 8 ) return MODBUS_ERROR_FRAME;

	uint16_t count = modbusSwapEndian( requestParser->request0304.count );

	//Check between data sent to slave and received from slave
	dataok &= parser->base.address != 0;
	dataok &= parser->response0304.address == requestParser->request0304.address;
	dataok &= parser->response0304.function == requestParser->request0304.function;
	dataok &= parser->response0304.length != 0;
	dataok &= parser->response0304.length == count << 1 ;
	dataok &= parser->response0304.length <= 250;

	//If data is bad, abort parsing, and set error flag
	if ( !dataok ) return MODBUS_ERROR_FRAME;

	//Allocate memory for ModbusData structures array
	status->data.coils = (uint8_t*) calloc( parser->response0304.length >> 1, sizeof( uint16_t ) );
	status->data.regs = (uint16_t*) status->data.coils;
	if ( status->data.coils == NULL ) return MODBUS_ERROR_ALLOC;
	status->data.address = parser->base.address;
	status->data.function = parser->base.function;
	status->data.type = parser->base.function == 3 ? MODBUS_HOLDING_REGISTER : MODBUS_INPUT_REGISTER;
	status->data.index = modbusSwapEndian( requestParser->request0304.index );
	status->data.count = count;

	//Copy received data (with swapping endianness)
	for ( i = 0; i < count; i++ )
		status->data.regs[i] = modbusSwapEndian( parser->response0304.values[i] );

	status->data.length = parser->response0304.length;
	return MODBUS_ERROR_OK;
}
#endif

#if ( LIGHTMODBUS_MASTER_F06 )
static uint8_t modbusBuildRequest06( ModbusMaster *status, uint8_t address, uint16_t index, uint16_t value )
{
	//Build request06 frame, to send it so slave
	//Write single holding reg

	//Set frame length
	uint8_t frameLength = 8;

	//Check if given pointer is valid
	if ( status == NULL ) return MODBUS_ERROR_OTHER;

	//Set output frame length to 0 (in case of interrupts)
	status->request.length = 0;
	status->predictedResponseLength = 0;

	//Reallocate memory for final frame
	free( status->request.frame );
	status->request.frame = (uint8_t *) calloc( frameLength, sizeof( uint8_t ) );
	if ( status->request.frame == NULL ) return MODBUS_ERROR_ALLOC;
	union ModbusParser *builder = (union ModbusParser *) status->request.frame;

	builder->base.address = address;
	builder->base.function = 6;
	builder->request06.index = modbusSwapEndian( index );
	builder->request06.value = modbusSwapEndian( value );

	//Calculate crc
	builder->request06.crc = modbusCRC( builder->frame, frameLength - 2 );

	status->request.length = frameLength;
	if ( address ) status->predictedResponseLength = 8;
	return MODBUS_ERROR_OK;
}

static uint8_t modbusParseResponse06( ModbusMaster *status, union ModbusParser *parser, union ModbusParser *requestParser )
{
	//Parse slave response to request 06 (write single holding reg)

	uint8_t dataok = 1;

	//Check if given pointers are valid
	if ( status == NULL || parser == NULL || requestParser == NULL ) return MODBUS_ERROR_OTHER;

	//Check if frame length is valid
	//Frame has to be at least 4 bytes long so byteCount can always be accessed in this case
	if ( status->response.length != 8 || status->request.length != 8 ) return MODBUS_ERROR_FRAME;

	//Check between data sent to slave and received from slave
	dataok &= parser->response06.address == requestParser->request06.address;
	dataok &= parser->response06.function == requestParser->request06.function;
	dataok &= parser->response06.index == requestParser->request06.index;
	dataok &= parser->response06.value == requestParser->request06.value;

	//If data is bad abort parsing, and set error flag
	if ( !dataok ) return MODBUS_ERROR_FRAME;

	//Set up new data table
	status->data.coils = (uint8_t*) calloc( 1, sizeof( uint16_t ) );
	status->data.regs = (uint16_t*) status->data.coils;
	if ( status->data.coils == NULL ) return MODBUS_ERROR_ALLOC;
	status->data.function = 6;
	status->data.address = parser->base.address;
	status->data.type = MODBUS_HOLDING_REGISTER;
	status->data.index = modbusSwapEndian( parser->response06.index );
	status->data.count = 1;
	status->data.regs[0] = modbusSwapEndian( parser->response06.value );
	status->data.length = 2;
	return MODBUS_ERROR_OK;
}
#endif

#if ( LIGHTMODBUS_MASTER_F16 )
static uint8_t modbusBuildRequest16( ModbusMaster *status, uint8_t address, uint16_t index, uint16_t count, uint16_t *values )
{
	//Build request16 frame, to send it so slave
	//Write multiple holding registers

	//Set frame length
	uint8_t frameLength = 9 + ( count << 1 );
	uint8_t i = 0;

	//Check if given pointer is valid
	if ( status == NULL ) return MODBUS_ERROR_OTHER;

	//Set output frame length to 0 (in case of interrupts)
	status->request.length = 0;
	status->predictedResponseLength = 0;

	//Check values pointer
	if ( values == NULL || count == 0 || count > 123 ) return MODBUS_ERROR_OTHER;

	//Reallocate memory for final frame
	free( status->request.frame );
	status->request.frame = (uint8_t *) calloc( frameLength, sizeof( uint8_t ) );
	if ( status->request.frame == NULL ) return MODBUS_ERROR_ALLOC;
	union ModbusParser *builder = (union ModbusParser *) status->request.frame;

	builder->base.address = address;
	builder->base.function = 16;
	builder->request16.index = modbusSwapEndian( index );
	builder->request16.count = modbusSwapEndian( count );
	builder->request16.length = count << 1;

	for ( i = 0; i < count; i++ )
		builder->request16.values[i] = modbusSwapEndian( values[i] );

	builder->request16.values[count] = modbusCRC( builder->frame, frameLength - 2 );

	status->request.length = frameLength;
	if ( address ) status->predictedResponseLength = 4 + 4;

	return MODBUS_ERROR_OK;
}

static uint8_t modbusParseResponse16( ModbusMaster *status, union ModbusParser *parser, union ModbusParser *requestParser )
{
	//Parse slave response to request 16 (write multiple holding reg)

	uint8_t dataok = 1;

	//Check if given pointers are valid
	if ( status == NULL || parser == NULL || requestParser == NULL ) return MODBUS_ERROR_OTHER;

	//Check frame lengths
	if ( status->request.length < 7u || status->request.length != 9 + requestParser->request16.length ) return MODBUS_ERROR_FRAME;
	if ( status->response.length != 8 ) return MODBUS_ERROR_FRAME;

	uint16_t count = modbusSwapEndian( parser->response16.count );

	//Check between data sent to slave and received from slave
	dataok &= parser->response16.address == requestParser->request16.address;
	dataok &= parser->response16.function == requestParser->request16.function;
	dataok &= parser->response16.index == requestParser->request16.index;
	dataok &= parser->response16.count == requestParser->request16.count;
	dataok &= count <= 123;

	//If data is bad abort parsing, and set error flag
	if ( !dataok ) return MODBUS_ERROR_FRAME;

	//Set up data length - response successfully parsed
	status->data.address = parser->base.address;
	status->data.function = 16;
	status->data.type = MODBUS_HOLDING_REGISTER;
	status->data.index = modbusSwapEndian( parser->response16.index );
	status->data.count = count;
	status->data.regs = NULL;
	status->data.length = 0;
	return MODBUS_ERROR_OK;
}
#endif

#if ( LIGHTMODBUS_MASTER_F22 )
static uint8_t modbusBuildRequest22( ModbusMaster *status, uint8_t address, uint16_t index, uint16_t andmask, uint16_t ormask )
{
	//Build request22 frame, to send it so slave
	//Mask write single holding reg

	//Set frame length
	uint8_t frameLength = 10;

	//Check if given pointer is valid
	if ( status == NULL ) return MODBUS_ERROR_OTHER;

	//Set output frame length to 0 (in case of interrupts)
	status->request.length = 0;
	status->predictedResponseLength = 0;

	//Reallocate memory for final frame
	free( status->request.frame );
	status->request.frame = (uint8_t *) calloc( frameLength, sizeof( uint8_t ) );
	if ( status->request.frame == NULL ) return MODBUS_ERROR_ALLOC;
	union ModbusParser *builder = (union ModbusParser *) status->request.frame;

	builder->base.address = address;
	builder->base.function = 22;
	builder->request22.index = modbusSwapEndian( index );
	builder->request22.andmask = modbusSwapEndian( andmask );
	builder->request22.ormask = modbusSwapEndian( ormask );

	//Calculate crc
	builder->request22.crc = modbusCRC( builder->frame, frameLength - 2 );

	status->request.length = frameLength;
	if ( address ) status->predictedResponseLength = 10;
	return MODBUS_ERROR_OK;
}

static uint8_t modbusParseResponse22( ModbusMaster *status, union ModbusParser *parser, union ModbusParser *requestParser )
{
	//Parse slave response to request 22 (mask write single holding reg)

	uint8_t dataok = 1;

	//Check if given pointers are valid
	if ( status == NULL || parser == NULL || requestParser == NULL ) return MODBUS_ERROR_OTHER;

	//Check if frame length is valid
	//Frame has to be at least 4 bytes long so byteCount can always be accessed in this case
	if ( status->response.length != 10 || status->request.length != 10 ) return MODBUS_ERROR_FRAME;

	//Check between data sent to slave and received from slave
	dataok &= parser->response22.address == requestParser->request22.address;
	dataok &= parser->response22.function == requestParser->request22.function;
	dataok &= parser->response22.index == requestParser->request22.index;
	dataok &= parser->response22.andmask == requestParser->request22.andmask;
	dataok &= parser->response22.ormask == requestParser->request22.ormask;

	//If data is bad abort parsing, and set error flag
	if ( !dataok ) return MODBUS_ERROR_FRAME;

	//Set up new data table
	status->data.address = parser->base.address;
	status->data.function = 22;
	status->data.type = MODBUS_HOLDING_REGISTER;
	status->data.index = modbusSwapEndian( parser->response22.index );
	status->data.count = 1;
	status->data.length = 0;
	return MODBUS_ERROR_OK;
}
#endif

static uint8_t modbusParseException( ModbusMaster *status, union ModbusParser *parser )
{
	//Parse exception frame and write data to MODBUSMaster structure

	//Check if given pointers are valid
	if ( status == NULL || parser == NULL ) return MODBUS_ERROR_OTHER;

	//Copy data (modbusParseResponse checked if length is 5 so it should be safe)
	status->exception.address = parser->exception.address;
	status->exception.function = parser->exception.function;
	status->exception.code = parser->exception.code;

	return MODBUS_ERROR_EXCEPTION;
}

uint8_t modbusParseResponse( ModbusMaster *status )
{
	//This function parses response from master
	//Calling it will lead to losing all data and exceptions stored in MODBUSMaster (space will be reallocated)

	//Note: crc is now checked here

	//If non-zero some parser failed its job
	uint8_t err = 0;

	//Check if given pointer is valid
	if ( status == NULL ) return MODBUS_ERROR_OTHER;

	//Reset output registers before parsing frame
	status->exception.address = 0;
	status->exception.function = 0;
	status->exception.code = 0;
	free( status->data.coils );
	status->data.coils = NULL;
	status->data.regs = NULL;
	status->data.length = 0;
	status->data.index = 0;
	status->data.count = 0;
	status->data.type = 0;
	status->data.address = 0;
	status->data.function = 0;

	//Check if frames are not too short and return error (to avoid problems with memory allocation)
	//That enables us to ommit the check in each parsing function
	if ( status->response.length < 4u || status->response.frame == NULL || \
	 	status->request.length < 4u || status->request.frame == NULL )
			return MODBUS_ERROR_OTHER;

	//Check both response and request frames CRC
	if ( *( (uint16_t*)( status->response.frame + status->response.length - 2 ) )\
		!= modbusCRC( status->response.frame, status->response.length - 2 ) ||\
		*( (uint16_t*)( status->request.frame + status->request.length - 2 ) ) \
		!= modbusCRC( status->request.frame, status->request.length - 2 ) )
			return MODBUS_ERROR_CRC;

	union ModbusParser *parser = (union ModbusParser*) status->response.frame;
	union ModbusParser *requestParser = (union ModbusParser*) status->request.frame;

	//Check if frame is exception response
	if ( parser->base.function & 128 && status->response.length == 5 )
	{
		err = modbusParseException( status, parser );
	}
	else
	{
		switch ( parser->base.function )
		{
			case 1: //Read multiple coils
			case 2: //Read multiple discrete inputs
				if ( LIGHTMODBUS_MASTER_F0102 || LIGHTMODBUS_MASTER_F01 || LIGHTMODBUS_MASTER_F02 )
					err = modbusParseResponse0102( status, parser, requestParser );
				else err = MODBUS_ERROR_PARSE;
				break;

			case 3: //Read multiple holding registers
			case 4: //Read multiple input registers
				if ( LIGHTMODBUS_MASTER_F0304 || LIGHTMODBUS_MASTER_F03 || LIGHTMODBUS_MASTER_F04 )
					err = modbusParseResponse0304( status, parser, requestParser );
				else err = MODBUS_ERROR_PARSE;
				break;

			case 5: //Write single coil
				if ( LIGHTMODBUS_MASTER_F05 ) err = modbusParseResponse05( status, parser, requestParser );
				else err = MODBUS_ERROR_PARSE;
				break;

			case 6: //Write single holding reg
				if ( LIGHTMODBUS_MASTER_F06 ) err = modbusParseResponse06( status, parser, requestParser );
				else err = MODBUS_ERROR_PARSE;
				break;

			case 15: //Write multiple coils
				if ( LIGHTMODBUS_MASTER_F15 ) err = modbusParseResponse15( status, parser, requestParser );
				else err = MODBUS_ERROR_PARSE;
				break;

			case 16: //Write multiple holding registers
				if ( LIGHTMODBUS_MASTER_F16 ) err = modbusParseResponse16( status, parser, requestParser );
				else err = MODBUS_ERROR_PARSE;
				break;

			case 22: //Mask write holding register
				if ( LIGHTMODBUS_MASTER_F22 ) err = modbusParseResponse22( status, parser, requestParser );
				else err = MODBUS_ERROR_PARSE;
				break;

			default: //function code not known by master
				err = MODBUS_ERROR_PARSE;
				break;
		}
	}
	return err;
}

uint8_t modbusMasterInit( ModbusMaster *status )
{
	//Check if given pointer is valid
	if ( status == NULL ) return MODBUS_ERROR_OTHER;

	//Very basic init of master side
	status->request.frame = NULL;
	status->request.length = 0;
	status->response.frame = NULL;
	status->response.length = 0;
	status->data.coils = NULL;
	status->data.regs = NULL;
	status->data.length = 0;
	status->data.count = 0;
	status->data.index = 0;
	status->data.type = 0;
	status->data.address = 0;

	status->exception.address = 0;
	status->exception.function = 0;
	status->exception.code = 0;

	return MODBUS_ERROR_OK;
}

uint8_t modbusMasterEnd( ModbusMaster *status )
{
	//Check if given pointer is valid
	if ( status == NULL ) return MODBUS_ERROR_OTHER;

	//Free memory
	free( status->request.frame );
	free( status->data.coils );
	status->data.coils = NULL;
	status->data.regs = NULL;

	return MODBUS_ERROR_OK;
}
