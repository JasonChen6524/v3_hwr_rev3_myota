/***********************************************************************************************//**
 * \file   spp_server_main.c
 * \brief  SPP server example
 *
 *
 ***************************************************************************************************
 * <b> (C) Copyright 2016 Silicon Labs, http://www.silabs.com</b>
 ***************************************************************************************************
 * This file is licensed under the Silabs License Agreement. See the file
 * "Silabs_License_Agreement.txt" for details. Before using this software for
 * any purpose, you must agree to the terms of that agreement.
 **************************************************************************************************/

/* Board headers */
#include "ble-configuration.h"
#include "board_features.h"

/* Bluetooth stack headers */
#include "bg_types.h"
#include "native_gecko.h"
#include "gatt_db.h"


#include <stdio.h>
#include "retargetswo.h"
#include "sleep.h"
#include "spp_utils.h"
#include "em_usart.h"
#include "sl_sleeptimer.h"
#include "infrastructure.h"

/* Vecttor3 */
#include "global.h"
#include "v3.h"
#include "i2c.h"

/***************************************************************************************************
  Local Macros and Definitions
 **************************************************************************************************/
// Moved to V3.h
//#define STATE_ADVERTISING 1
//#define STATE_CONNECTED   2
//#define STATE_SPP_MODE    3

/* Maximum number of iterations when polling UART RX data before sending data over BLE connection
 * set value to 0 to disable optimization -> minimum latency but may decrease throughput */
#define UART_POLL_TIMEOUT  5000

/* soft timer handles */
#define TIC_TIMER_HANDLE   1
#define OS_TIMER_HANDLE    2
#define OS_1S_TIMER_HANDLE 3


#define TIC_TIMER_CONST  32768
#define TIC_TIMER_PERSEC 10

enum
{
  HANDLE_V3, HANDLE_EDDYSTONE, HANDLE_IBEACON
};

//ibeacon setup call
void bcnSetupAdvBeaconing(void);

extern void bpt_main(void); // Temporarily put here, belongs elsewhere
extern void bpt_main_reset(void); // Added by Jason, 2021.01.07

// Eddystone Advertising data
#define EDDYSTONE_DATA_LEN (23)

static const uint8_t eddystone_data[EDDYSTONE_DATA_LEN] = {
  0x03,          // Length of service list
  0x03,          // Service list
  0xAA, 0xFE,    // Eddystone ID
  0x12,          // Length of service data
  0x16,          // Service data
  0xAA,  0xFE,   // Eddystone ID
  0x10,          // Frame type Eddystone-URL
  0x00,          // Tx power
  0x00,          // 0x00=http://www., 0x01=https://www.
  'a','r','t','a','f','l','e','x','.','c','o','m'
};

/***************************************************************************************************
 Local Variables
 **************************************************************************************************/
#include "btl_interface.h"
#include "btl_interface_storage.h"

#define uart_flush()

static void bootMessage(struct gecko_msg_system_boot_evt_t *bootevt);

static BootloaderInformation_t bldInfo;
static BootloaderStorageSlot_t slotInfo;

/* OTA variables */
static uint32 ota_image_position = 0;
static uint8 ota_in_progress = 0;
static uint8 ota_image_finished = 0;
static uint16 ota_time_elapsed = 0;

static int32_t get_slot_info()
{
	int32_t err;

	bootloader_getInfo(&bldInfo);
	printLog("Gecko bootloader version: %u.%u\r\n", (uint16_t)((bldInfo.version & 0xFF000000) >> 24), (uint16_t)((bldInfo.version & 0x00FF0000) >> 16));

	err = bootloader_getStorageSlotInfo(0, &slotInfo);

	if(err == BOOTLOADER_OK)
	{
		printLog("Slot 0 starts @ 0x%8.8x, size %u bytes\r\n", (uint16_t)slotInfo.address, (uint16_t)slotInfo.length);
	}
	else
	{
		printLog("Unable to get storage slot info, error %x\r\n", (uint16_t)err);
	}

	return(err);
}

static void erase_slot_if_needed()
{
	uint32_t offset = 0;
	uint8_t buffer[256];
	int i;
	int dirty = 0;
	int32_t err = BOOTLOADER_OK;
	int num_blocks = 0;

	/* check the download area content by reading it in 256-byte blocks */

	num_blocks = slotInfo.length / 256;

	while((dirty == 0) && (offset < 256*num_blocks) && (err == BOOTLOADER_OK))
	{
		err = bootloader_readStorage(0, offset, buffer, 256);
		if(err == BOOTLOADER_OK)
		{
			i=0;
			while(i<256)
			{
				if(buffer[i++] != 0xFF)
				{
					dirty = 1;
					break;
				}
			}
			offset += 256;
		}
		printf(".");
	}

	if(err != BOOTLOADER_OK)
	{
		printLog("\r\nerror reading flash! %x\r\n", (uint16_t)err);
	}
	else if(dirty)
	{
		printLog("\r\ndownload area is not empty, erasing...\r\n");
		bootloader_eraseStorageSlot(0);
		printLog("done\r\n");
	}
	else
	{
		printLog("\r\ndownload area is empty\r\n");
	}

	return;
}

static void print_progress()
{
	// estimate transfer speed in kbps
	int kbps = ota_image_position*8/(1024*ota_time_elapsed);

	printLog("pos: %u, time: %u, kbps: %u\r\n", (uint16_t)ota_image_position, (uint16_t)ota_time_elapsed, kbps);
}

static uint8 _conn_handle = 0xFF;
static int _main_state;

tsCounters _sCounters;

static uint8 _max_packet_size = 20; // Maximum bytes per one packet
static uint8 _min_packet_size = 20; // Target minimum bytes for one packet

static void reset_variables()
{
	_conn_handle = 0xFF;
	_main_state = STATE_ADVERTISING;
   v3status.spp = STATE_ADVERTISING;
    //gecko_cmd_hardware_set_soft_timer(0, TIC_TIMER_HANDLE, 0);
    
	_max_packet_size = 20;

	memset(&_sCounters, 0, sizeof(_sCounters));
   
   v3CommBuf.rxhead = 0;
   v3CommBuf.rxtail = 0;
   v3CommBuf.txhead = 0;
   v3CommBuf.txtail = 0;
}

static void send_spp_msg()
{
	uint8 len = 0;
	uint8 data[256];
	uint16 result;

	//int c;
	//int timeout = 0;

	if (v3CommBuf.txhead==v3CommBuf.txtail) return;  // no data to send;
   
   // Read up to _max_packet_size characters from local buffer
	while (len < _max_packet_size) 
   {
      data[len++] = v3CommBuf.tx[v3CommBuf.txtail++];
      //v3CommBuf.txtail &= V3BUFMASK;  // not needed for 256 size buffer with U8 index
      if (v3CommBuf.txhead==v3CommBuf.txtail) break;
	}

	if (len > 0) {
		// Stack may return "out-of-memory" error if the local buffer is full -> in that case, just keep trying until the command succeeds
		do {
			result = gecko_cmd_gatt_server_send_characteristic_notification(_conn_handle, gattdb_gatt_spp_data, len, data)->result;
			_sCounters.num_writes++;
		} while(result == bg_err_out_of_memory);

		if (result != 0) {
			printLog("Unexpected error: %x\r\n", result);
		} else {
			_sCounters.num_pack_sent++;
			_sCounters.num_bytes_sent += len;
		}
	}
}


/**
 * @brief  SPP server mode main loop
 */
void spp_server_main(void)
{
int i;
//char logout[80];
U8 sectic = TIC_TIMER_PERSEC;

   SLEEP_SleepBlockBegin(sleepEM2); // Disable sleeping

  // Create one-shot soft timer to delay for I/O expander powerup
//  gecko_cmd_hardware_set_soft_timer(TIC_TIMER_CONST/TIC_TIMER_PERSEC, OS_TIMER_HANDLE, true);

  // Create soft timer to Handle init I/O and runtime I/O
  gecko_cmd_hardware_set_soft_timer(TIC_TIMER_CONST/TIC_TIMER_PERSEC, OS_TIMER_HANDLE, false);

  while (1) {
    /* Event pointer for handling events */
    struct gecko_cmd_packet* evt;

   //bpt_main();   // For Bio-Sensor estimation -

    if(_main_state == STATE_SPP_MODE) {
    	/* If SPP data mode is active, use non-blocking gecko_peek_event() */
    	evt = gecko_peek_event();

    	if(evt == NULL) {
    		/* No stack events to be handled -> send data from local TX buffer */
    	//send_spp_data();
         send_spp_msg(); // send V3 Message, if any from circular buffer
    		recv_spp_msg(); // receive V3 Message, if any from circular buffer
         continue;  		// Jump directly to next iteration i.e. call gecko_peek_event() again
    	}
    } else {
//      /* if there are no events pending then the next call to gecko_wait_event() may cause
//      * device go to deep sleep. Make sure that debug prints are flushed before going to sleep */
//      if (!gecko_event_pending()){flushLog();}

    	/* SPP data mode not active -> check for stack events using the blocking API */
    	evt = gecko_wait_event();
    }

    /* Handle events */
   switch (BGLIB_MSG_ID(evt->header))
   {

      /* This boot event is generated when the system boots up after reset.
       * Here the system is set to start advertising immediately after boot procedure. */
      case gecko_evt_system_boot_id:

         reset_variables();
         gecko_cmd_gatt_set_max_mtu(247);
         gecko_cmd_le_gap_start_advertising(HANDLE_V3, le_gap_general_discoverable, le_gap_undirected_connectable);
#if 1  //include this code for Eddystone and iBeacon support

        // eddystone additions
        //gecko_cmd_system_set_tx_power(0);

        /* Set 8 dBm Transmit Power */
         gecko_cmd_le_gap_set_advertise_tx_power(HANDLE_EDDYSTONE, 75);

        /* Set advertising parameters. 1000ms advertisement interval.
         * The first parameter is advertising set handle
         * The next two parameters are minimum and maximum advertising interval, both in
         * units of (milliseconds * 1.6).
         * The last two parameters are duration and maxevents left as default. */
        gecko_cmd_le_gap_set_advertise_timing(HANDLE_EDDYSTONE, 1600, 1600, 0, 0);
        gecko_cmd_le_gap_bt5_set_adv_data(HANDLE_EDDYSTONE, 0, EDDYSTONE_DATA_LEN, eddystone_data);
        /* Start general advertising. */
        gecko_cmd_le_gap_start_advertising(HANDLE_EDDYSTONE, le_gap_user_data, le_gap_non_connectable);

        /* Start non-connectable advertising ibeacons */
        bcnSetupAdvBeaconing();
#endif
#if 1
#if DEBUG_LEVEL
	    bootMessage(&(evt->data.evt_system_boot));
#endif
	    /* 1 second soft timer, used for performance statistics during OTA file upload */
	    gecko_cmd_hardware_set_soft_timer(32768, OS_1S_TIMER_HANDLE, 0);

	    /* set advertising interval to 100 ms */
	    //gecko_cmd_le_gap_set_advertise_timing(0, 160, 160, 0, 0);

	    /* Start general advertising and enable connections. */
	    //printLog("boot event - starting advertising\r\n");
	    //gecko_cmd_le_gap_start_advertising(0, le_gap_general_discoverable, le_gap_connectable_scannable);

	    /* bootloader init must be called before calling other bootloader_xxx API calls */
	    bootloader_init();

	    /* read slot information from bootloader */
	    if(get_slot_info() == BOOTLOADER_OK)
	    {
	       	/* the download area is erased here (if needed), prior to any connections are opened */
	    	erase_slot_if_needed();
	    }
	    else
	    {
	    	printLog("Check that you have installed correct type of Gecko bootloader!\r\n");
	    }
#endif
    	break;

    	/* Connection opened event */
      case gecko_evt_le_connection_opened_id:

         _conn_handle = evt->data.evt_le_connection_opened.connection;
       //printLog("Connected\r\n");
         printf("connection opened\r\n");
         _main_state = STATE_CONNECTED;
         v3status.spp = STATE_CONNECTED;

         /* Request connection parameter update.
          * conn.interval min 20ms, max 40ms, slave latency 4 intervals,
          * supervision timeout 2 seconds
          * (These should be compliant with Apple Bluetooth Accessory Design Guidelines, both R7 and R8) */
         gecko_cmd_le_connection_set_timing_parameters(_conn_handle, 24, 40, 0, 200, 0, 0xFFFF);
    	break;

      case gecko_evt_le_connection_parameters_id:
         printLog("Conn.parameters: interval %u units, txsize %u\r\n", evt->data.evt_le_connection_parameters.interval, evt->data.evt_le_connection_parameters.txsize);
    	break;

      case gecko_evt_gatt_mtu_exchanged_id:
         /* Calculate maximum data per one notification / write-without-response, this depends on the MTU.
          * up to ATT_MTU-3 bytes can be sent at once  */
         _max_packet_size = evt->data.evt_gatt_mtu_exchanged.mtu - 3;
         _min_packet_size = _max_packet_size; /* Try to send maximum length packets whenever possible */
         printLog("MTU exchanged: %d\r\n", evt->data.evt_gatt_mtu_exchanged.mtu);
    	break;

      case gecko_evt_le_connection_closed_id:
    	  printLog("connection closed, reason: 0x%2.2x\r\n", evt->data.evt_le_connection_closed.reason);
    	  if (ota_image_finished) {
    		  printLog("Installing new image\r\n");
#if DEBUG_LEVEL
    		  flushLog();
#endif
    		  bootloader_setImageToBootload(0);
    		  bootloader_rebootAndInstall();
    	  } else {

	         /* Show statistics (RX/TX counters) after disconnect: */
   	         printStats(&_sCounters);

   	         reset_variables();
   	         SLEEP_SleepBlockEnd(sleepEM2); // Enable sleeping                 //Commented by Jason

             /* Restart advertising after client has disconnected */
    		 gecko_cmd_le_gap_start_advertising(0, le_gap_general_discoverable, le_gap_connectable_scannable);
    	  }


         /* Restart advertising */
         //gecko_cmd_le_gap_start_advertising(0, le_gap_general_discoverable, le_gap_undirected_connectable);

        /* Check if need to boot to OTA DFU mode */
        //if (boot_to_dfu) {
          /* Enter to OTA DFU mode */
        //  gecko_cmd_system_reset(2);
        //} else {
          /* Restart advertising after client has disconnected */
        //  gecko_cmd_le_gap_start_advertising(0, le_gap_general_discoverable, le_gap_connectable_scannable);
        //}
    	break;

      case gecko_evt_gatt_server_characteristic_status_id:
      {
         struct gecko_msg_gatt_server_characteristic_status_evt_t *pStatus;
         pStatus = &(evt->data.evt_gatt_server_characteristic_status);

         if (pStatus->characteristic == gattdb_gatt_spp_data) {
            if (pStatus->status_flags == gatt_server_client_config) {
               // Characteristic client configuration (CCC) for spp_data has been changed
               if (pStatus->client_config_flags == gatt_notification) {
                  _main_state = STATE_SPP_MODE;
                  v3status.spp = STATE_SPP_MODE;
                  //gecko_cmd_hardware_set_soft_timer(32768, TIC_TIMER_HANDLE, 0);
                  SLEEP_SleepBlockBegin(sleepEM2); // Disable sleeping
                  //initBoard();
                  printLog("SPP Mode ON\r\n");
               } else {
                  printLog("SPP Mode OFF\r\n");
                  _main_state = STATE_CONNECTED;
                  v3status.spp = STATE_CONNECTED;
                  SLEEP_SleepBlockEnd(sleepEM2); // Enable sleeping                         //Commented by Jason
               }

    		}
    	 }
      }
      break;

      case gecko_evt_gatt_server_attribute_value_id:
      {
    	 for(i=0;i<evt->data.evt_gatt_server_attribute_value.value.len;i++) {
    		 //USART_Tx(RETARGET_UART, evt->data.evt_gatt_server_attribute_value.value.data[i]);
          v3CommBuf.rx[v3CommBuf.rxhead++]  = evt->data.evt_gatt_server_attribute_value.value.data[i];
          if(v3CommBuf.rxhead == v3CommBuf.rxtail)
          {
             // make error condition for receive buffer overflow
             break;
          }

         }

    	 _sCounters.num_pack_received++;
    	 _sCounters.num_bytes_received += evt->data.evt_gatt_server_attribute_value.value.len;
      }
      break;

      /* Software Timer event */
      case gecko_evt_hardware_soft_timer_id:
      {
		  switch (evt->data.evt_hardware_soft_timer.handle)
		  {
			 //case TIC_TIMER_HANDLE:  // currently once per second
				//v3_state(); // sequence main V3 state machine
			 //break;
		     case OS_1S_TIMER_HANDLE:
		    	 if(ota_in_progress)
		    	 {
		    		 ota_time_elapsed++;
		    		 print_progress();
		    	 }
		    	 break;
			 case OS_TIMER_HANDLE:
			 //SLEEP_SleepBlockBegin(sleepEM2); // Disable sleeping

			 // Keep v3_state call before LED and feedback calls to make the response more immediate
			 sectic++;

			 if (sectic >= TIC_TIMER_PERSEC)
			 {
				v3_state(); // sequence main V3 state machine
				sectic = 0;
			 }

			 ledseq();  // step the LED player
			 fbseq(); // Step the feedback player (Haptic and buzzer)

			 //Jason // For Bio-Sensor estimation -
			 if((v3status.spp == STATE_CONNECTED)||(v3status.spp == STATE_SPP_MODE))  bpt_main();
			 else  bpt_main_reset();

			 //bpt_main();   // For Bio-Sensor estimation - Jason had this in the main while(1) loop.  Should go here?  Need to test

			 if (v3sleep.sleepsec)
			 {
				   v3_state(); // sequence main V3 state machine
				   gecko_cmd_hardware_set_soft_timer(0, OS_TIMER_HANDLE, false);  // turn off timer
				   gecko_cmd_hardware_set_soft_timer((v3sleep.sleepsec*TIC_TIMER_CONST), OS_TIMER_HANDLE, false);  // set new sleep timer
				   v3sleep.sleepsec = 0;  // clear flag
				   sectic = TIC_TIMER_PERSEC;   // Force state machine to be called next entry
			 }

			 //SLEEP_SleepBlockEnd(sleepEM2); // Enable sleeping
			 break;

			 default:
			 break;
		  }
      }
	  break;
      case gecko_evt_gatt_server_user_write_request_id:
      {
    	  uint32_t connection = evt->data.evt_gatt_server_user_write_request.connection;
    	  uint32_t characteristic = evt->data.evt_gatt_server_user_write_request.characteristic;
    	  if(characteristic == gattdb_ota_control)
    	  {
    		  switch(evt->data.evt_gatt_server_user_write_request.value.data[0])
    		  {
    		  case 0://Erase and use slot 0
    			  // NOTE: download are is NOT erased here, because the long blocking delay would result in supervision timeout
    			  //bootloader_eraseStorageSlot(0);
    			  ota_image_position=0;
    			  ota_in_progress=1;
    			  break;
    		  case 3://END OTA process
    			  //wait for connection close and then reboot
    			  ota_in_progress=0;
    			  ota_image_finished=1;
    			  printLog("upload finished. received file size %u bytes\r\n", (uint16_t)ota_image_position);
    			  uart_flush();
    			  break;
    		  default:
    			  break;
    		  }
    	  } else if(characteristic == gattdb_ota_data)
    	  {
    		  if(ota_in_progress)
    		  {
    			  bootloader_writeStorage(0,//use slot 0
    					  ota_image_position,
						  evt->data.evt_gatt_server_user_write_request.value.data,
						  evt->data.evt_gatt_server_user_write_request.value.len);
    			  ota_image_position+=evt->data.evt_gatt_server_user_write_request.value.len;
    		  }
    	  }
    	  gecko_cmd_gatt_server_send_user_write_response(connection,characteristic,0);
      }
   	  break;
    //break;
      default:
      break;
   }//switch (BGLIB_MSG_ID(evt->header))
  }//while(1)
}

void bcnSetupAdvBeaconing(void)
{

  /* This function sets up a custom advertisement package according to iBeacon specifications.
   * The advertisement package is 30 bytes long. See the iBeacon specification for further details.
   */

  static struct
  {
    uint8_t flagsLen; /* Length of the Flags field. */
    uint8_t flagsType; /* Type of the Flags field. */
    uint8_t flags; /* Flags field. */
    uint8_t mandataLen; /* Length of the Manufacturer Data field. */
    uint8_t mandataType; /* Type of the Manufacturer Data field. */
    uint8_t compId[2]; /* Company ID field. */
    uint8_t beacType[2]; /* Beacon Type field. */
    uint8_t uuid[16]; /* 128-bit Universally Unique Identifier (UUID). The UUID is an identifier for the company using the beacon*/
    uint8_t majNum[2]; /* Beacon major number. Used to group related beacons. */
    uint8_t minNum[2]; /* Beacon minor number. Used to specify individual beacons within a group.*/
    uint8_t txPower; /* The Beacon's measured RSSI at 1 meter distance in dBm. See the iBeacon specification for measurement guidelines. */
  } bcnBeaconAdvData = {
  /* Flag bits - See Bluetooth 4.0 Core Specification , Volume 3, Appendix C, 18.1 for more details on flags. */
  2, /* length  */
  0x01, /* type */
  0x04 | 0x02, /* Flags: LE General Discoverable Mode, BR/EDR is disabled. */

  /* Manufacturer specific data */
  26, /* length of field*/
  0xFF, /* type of field */

  /* The first two data octets shall contain a company identifier code from
   * the Assigned Numbers - Company Identifiers document */
  /* 0x004C = Apple */
  {UINT16_TO_BYTES(0x004C)},

  /* Beacon type */
  /* 0x0215 is iBeacon */
  {UINT16_TO_BYTE1(0x0215), UINT16_TO_BYTE0(0x0215)},

  /* 128 bit / 16 byte UUID */
  {0xE2, 0xC5, 0x6D, 0xB5, 0xDF, 0xFB, 0x48, 0xD2, 0xB0, 0x60, 0xD0, 0xF5, 0xA7, 0x10, 0x96, 0xE0},

  /* Beacon major number */
  /* Set to 34987 and converted to correct format */
  {UINT16_TO_BYTE1(34987), UINT16_TO_BYTE0(34987)},

  /* Beacon minor number */
  /* Set as 1025 and converted to correct format */
  {UINT16_TO_BYTE1(1025), UINT16_TO_BYTE0(1025)},

  /* The Beacon's measured RSSI at 1 meter distance in dBm */
  /* 0xC3 is -61dBm */
  0xC3};

  //
  uint8_t len = sizeof(bcnBeaconAdvData);
  uint8_t *pData = (uint8_t*)(&bcnBeaconAdvData);

  /* Set custom advertising data */

  gecko_cmd_le_gap_bt5_set_adv_data(HANDLE_IBEACON, 0, len, pData);

  /* Set 8 dBm Transmit Power */
  gecko_cmd_le_gap_set_advertise_tx_power(HANDLE_IBEACON, 75);

  /* Set advertising parameters. 200ms (320/1.6) advertisement interval. All channels used.
   * The first two parameters are minimum and maximum advertising interval, both in
   * units of (milliseconds * 1.6).  */
  gecko_cmd_le_gap_set_advertise_timing(HANDLE_IBEACON, 1600, 1600, 0, 0);

  /* Start advertising in user mode */
  gecko_cmd_le_gap_start_advertising(HANDLE_IBEACON, le_gap_user_data, le_gap_non_connectable);

}

static void bootMessage(struct gecko_msg_system_boot_evt_t *bootevt)
{
	bd_addr local_addr;
	int i;

	printLog("stack version: %u.%u.%u\r\n", bootevt->major, bootevt->minor, bootevt->patch);
	local_addr = gecko_cmd_system_get_bt_address()->address;

	printLog("local BT device address: ");
	for(i=0;i<5;i++)
	{
		printLog("%2.2x:", local_addr.addr[5-i]);
	}
	printLog("%2.2x\r\n", local_addr.addr[0]);

}
