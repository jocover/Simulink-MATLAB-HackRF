/*
* Copyright 2012 Communications Engineering Lab, KIT
* Copyright 2014 Nuand, LLC
*
* This is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation; either version 3, or (at your option)
* any later version.
*
* This software is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this software; see the file COPYING. If not, write to
* the Free Software Foundation, Inc., 51 Franklin Street,
* Boston, MA 02110-1301, USA.
*/

/*
    Find HackRF devices attached to the host.
*/

/* Simulink includes */
#include <simstruc.h>
#include <mex.h>

#include <hackrf.h>

/* Entry point to C/C++ */
void mexFunction(int nlhs,mxArray *plhs[],int nrhs,const mxArray *prhs[])
{
	int result =HACKRF_SUCCESS,i;
	struct hackrf_device *_device [10];
	hackrf_device_list_t *list;
	uint8_t board_id = BOARD_ID_INVALID;
	char version[255 + 1];
	read_partid_serialno_t read_partid_serialno;
	
	result = hackrf_init();
	if (result != HACKRF_SUCCESS) {
		mexPrintf("hackrf_init() failed: %s (%d)\n",hackrf_error_name((hackrf_error)result), result);
	}
	list = hackrf_device_list();
	
	if (list->devicecount < 1 ) {
		mexPrintf("No HackRF boards found.\n");
	}	
		
	for (i = 0; i < list->devicecount; i++){
	mexPrintf("Found HackRF board %d:\n", i);
	if (list->serial_numbers[i])
			mexPrintf("USB descriptor string: %s\n", list->serial_numbers[i]);
			
		result = hackrf_device_list_open(list, i, &_device [i]);	
			if (result != HACKRF_SUCCESS) {
			mexPrintf("hackrf_open() failed: %s (%d)\n",hackrf_error_name((hackrf_error)result), result);
		}
	
		result = hackrf_board_id_read(_device[i], &board_id);
		if (result != HACKRF_SUCCESS) {
			mexPrintf("hackrf_board_id_read() failed: %s (%d)\n",hackrf_error_name((hackrf_error)result), result);
		}
		mexPrintf("Board ID Number: %d (%s)\n", board_id,hackrf_board_id_name((hackrf_board_id)board_id));
		
		result = hackrf_version_string_read(_device[i], &version[0], 255);
		if (result != HACKRF_SUCCESS) {
			mexPrintf("hackrf_version_string_read() failed: %s (%d)\n",hackrf_error_name((hackrf_error)result), result);

		}
		printf("Firmware Version: %s\n", version);

		result = hackrf_board_partid_serialno_read(_device[i], &read_partid_serialno);	
		if (result != HACKRF_SUCCESS) {
			mexPrintf( "hackrf_board_partid_serialno_read() failed: %s (%d)\n",hackrf_error_name((hackrf_error)result), result);
		}
		printf("Part ID Number: 0x%08x 0x%08x\n", 
					read_partid_serialno.part_id[0],
					read_partid_serialno.part_id[1]);
		printf("Serial Number: 0x%08x 0x%08x 0x%08x 0x%08x\n", 
					read_partid_serialno.serial_no[0],
					read_partid_serialno.serial_no[1],
					read_partid_serialno.serial_no[2],
					read_partid_serialno.serial_no[3]);
		
		result = hackrf_close(_device[i]);
		if (result != HACKRF_SUCCESS) {
			mexPrintf("hackrf_close() failed: %s (%d)\n",hackrf_error_name((hackrf_error)result), result);
		}
		
	
	}
	
	hackrf_device_list_free(list);
	hackrf_exit();
	
		
}
