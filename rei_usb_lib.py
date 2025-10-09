# 
# Project Name:
# ----------------------
# USB20F SSI Bridge Lib
#
# Project Description:
# ----------------------
# This module is intended to be example code and a utility library
# to support REIndustries USB bridge testing and development.
#
# The Python logger module is always enabled for text file logging
# and the console message log can be disabled or enabled by user.
#
# C:\Users\<username>\AppData\Local\Programs\Python
#
# TODO:
# ----------------------
# 1. Add option to allow caller to pass SN in string format
#	for appropriate methods. Right now you can't select based
#	on SN.
#
# 2. Set default VID/PID to 0x0451/0x0309
#
# 3. Get rid of ability to write to self.ep_data_in/out as they
#	are pointers and you can't actually modify their contents
#	since they are not arrays.
#
# 4. Add a return value for dump reg space that is all reg read
#	data. Also add option for this call to print to screen with
#	verbose -v flag or not. So user can just save returned reg
#	data or have it print to screen or both.
# 	
#
# 5. Might want to rename this lib to rei_usb20f_lib or something
#	specific to the full speed bridge.
#
# 6. read_int1 funct needs some sort of local ep_size so other
#	read functions that change self.EP_SIZE don't mess up INT
#	64 byte endpoints - these never change size like BULK ones
#	do. I tried making a 'local' variable in the func but it
# 	didn't work. Need to read up on how to properly create local
#	func variables in Python.
#
# ----------------------------------------------------------------
# Disclaimer:
# ----------------------------------------------------------------
# This library is provided strictly as example code. There is no
# expected reliablity of operation from RisingEdgeIndustries and 
# this source code is not to be sold or represented as a 3'd party
# solution for commercial use. The below code is development code
# for example use only supporting customers as they test the bridge
# products from RisingEdgeIndustries. Nothing in this file is allowed
# to be modified or sold in any way. No code below is released with 
# the intention or expectation of reliable operation.
#
# Packing this module with any 3d part code can only be done with 
# the inclusion of this disclaimer and no modifications.
# ----------------------------------------------------------------

import sys
import ctypes as ct
import libusb as usb
import time
import logging
from logging.handlers import QueueHandler, QueueListener
from USB_SSI_Libs import LoggingUtils_USB20F



#------------------------------------------------------------
# Name: USB_Device():
#
# Description:
#   This class encapsulates basic functionality of the libusb
#	Python library needed for accessing core functionality of
#	the USB20F-SSI bridge.
#
# Parameters:
#	quiet: EN/DIS print log messages to console
#	name: Name of calling python module
#
#------------------------------------------------------------
class USB20F_Device(object):
	def __init__(self, quiet=False, name="Unknown"):
		# class parameters
		self.NAME = name + "(rei_usb_lib)"
		self.DESCRIPTION = ""
		self.EP_TIMEOUT = 250 #mS
		self.EP_SIZE = 64
		self.bulk_transferred = ct.POINTER(ct.c_int)()
		self.bulk_transferred.contents = ct.c_int(0)

		# USB Endpoint buffers setup before usb xfer calls
		self.EP_SIZE = 64
		self._EP_INT0_IN = 0x81
		self._EP_INT0_OUT = 0x01
		self._EP_INT1_IN = 0x82
		self._EP_INT1_OUT = 0x02
		self._EP_BULK_IN = 0x83
		self._EP_BULK_OUT = 0x03

		self.EPOUT_ACTIVE = self._EP_INT0_OUT
		self.EPIN_ACTIVE = self._EP_INT0_IN
		
		self.ep_data_out = (ct.c_ubyte*(self.EP_SIZE))()
		self.ep_data_in = (ct.c_ubyte*(self.EP_SIZE))()		

		# return status
		self.r = 0

		# setup register addresses
		self.CR1_ADDR = 0x00000000
		self.CR2_ADDR = 0x00000004
		self.SR1_ADDR = 0x00000008
		self.SR2_ADDR = 0x0000000C
		self.ASR_ADDR = 0x00000010
		self.SKEY_ADDR = 0x00000014
		self.USBBLKSR_ADDR = 0x00000018
		self.USBINT0SR_ADDR = 0x0000001C
		self.USBINT1SR_ADDR = 0x00000020
		self.USBFBRXSR_ADDR = 0x00000024
		self.USBFBTXSR_ADDR = 0x00000028
		self.USBBLKRXFC_ADDR = 0x0000002C
		self.USBIF0RXFC_ADDR = 0x00000030
		self.USBIF1RXFC_ADDR = 0x00000034
		self.NVMEMSR_ADDR = 0x00000038
		self.SSITXFC_ADDR = 0x0000003C
		self.SSITXEC_ADDR = 0x00000040
		self.SSIRXFC_ADDR = 0x00000044
		self.SSIRXEC_ADDR = 0x00000048
		self.SCRTCH1_ADDR = 0x0000004C
		self.SCRTCH2_ADDR = 0x00000050
		self.SCRTCH3_ADDR = 0x00000054
		self.SCRTCH4_ADDR = 0x00000058
		self.SSITXLGSTS_ADDR = 0x0000005C
		self.SSIRXLGSTS_ADDR = 0x00000060
		self.SIRXFSSCNT_ADDR = 0x00000064
		self.CTRTXDATA0_ADDR = 0x00000068
		self.CTRTXDATA1_ADDR = 0x0000006C
		self.CTRTXDATA2_ADDR = 0x00000070
		self.CTRTXDATA3_ADDR = 0x00000074
		self.CTRTXDATA4_ADDR = 0x00000078
		self.CTRTXDATA5_ADDR = 0x0000007C
		self.CTRTXDATA6_ADDR = 0x00000080
		self.CTRTXDATA7_ADDR = 0x00000084
		self.CTRRXDATA0_ADDR = 0x000000A8
		self.CTRRXDATA1_ADDR = 0x000000AC
		self.CTRRXDATA2_ADDR = 0x000000B0
		self.CTRRXDATA3_ADDR = 0x000000B4
		self.CTRRXDATA4_ADDR = 0x000000B8
		self.CTRRXDATA5_ADDR = 0x000000BC
		self.CTRRXDATA6_ADDR = 0x000000C0
		self.CTRRXDATA7_ADDR = 0x000000C4
		self.CTRMODECR_ADDR = 0x000000E8



		# setup logging
		self.log = LoggingUtils_USB20F.LogClass(self.NAME, quiet)
		self.log.write("INFO", f"{self.NAME} logger starting up!")		






	#------------------------------------------------------------
	# Name: open_usb():
	#
	# Description:
	#   Open a USB link targeted at a specific VID and PID.
	#	this function call searches all USB devices until
	#	the target device is found.
	#	The intended use is to access the usb device handle
	#	object in the lib, but it is also returned for other
	#	unforseen use cases.
	#
	# Parameters:
	#	VID: hex vid value
	#	PID: hex pid value
	#
	# Return:
	#	- returns tuple with (<pass/fail flag>, <error_code or data>)
	#	Success: (0, usb_dev_handle)
	#	Failure: (1, <error code>)
	#
	#------------------------------------------------------------
	def open_usb(self, vid=0x1cbf, pid=0x0007):
		self.log.write("DEBUG", "--> Enter open_usb()")
		#
		# callback vars
		#
		self.dev = None
		self.dev_found  = False
		self.sn_string = (ct.c_ubyte* 18)()	# sn string
		self.pd_string = (ct.c_ubyte* 30)()	# product string
		self.mf_string = (ct.c_ubyte* 26)()	# manf string
		self.device_configuration = ct.POINTER(ct.c_int)()	
		self.dev_handle = ct.POINTER(usb.device_handle)() # creates device handle (not device obj)
		self.desc = None
		self.r = None
		self.vid = vid
		self.pid = pid

		#
		# inits
		#
		self.device_configuration.contents = ct.c_int(0)

		# open usb device
		self.r = usb.init(None)
		if self.r < 0:
			self.log.write("ERROR", f'usb init failure: {self.r}')
			return (1, 1)

		self.devs = ct.POINTER(ct.POINTER(usb.device))() # creates device structure
		cnt = usb.get_device_list(None, ct.byref(self.devs))

		if cnt < 0:
			self.log.write("ERROR", f'get device list failure: {cnt}')
			return (1, 2)

		self.log.write("INFO", '\n')
		self.log.write("INFO", "/* Getting USB device list */")

		# find device with matching VID/PID
		i = 0
		while self.devs[i]:
			self.dev = self.devs[i]

			self.desc = usb.device_descriptor()
			self.r = usb.get_device_descriptor(self.dev, ct.byref(self.desc))

			if self.r < 0:
				self.log.write("ERROR", f'failed to get device descriptor: {self.r}')
				return (1, 3)

			self.log.write("INFO", "{:04x}:{:04x} (bus {:d}, device {:d})".format(
				  self.desc.idVendor, self.desc.idProduct, 
				  usb.get_bus_number(self.dev), usb.get_device_address(self.dev)))


			if(self.desc.idVendor == self.vid) and (self.desc.idProduct == self.pid):
				self.log.write("INFO", '\n')
				self.log.write("INFO", '/* Descriptor Inforamtion */')
				self.log.write("INFO", f"{'bLength: ':.<30}{f'{self.desc.bLength:#02x}':.>20}")
				self.log.write("INFO", f"{'bDescriptorType: ':.<30}{f'{self.desc.bDescriptorType:#02x}':.>20}")
				self.log.write("INFO", f"{'bcdUSB: ':.<30}{f'{self.desc.bcdUSB:#04x}':.>20}")
				self.log.write("INFO", f"{'bDeviceClass: ':.<30}{f'{self.desc.bDeviceClass:#02x}':.>20}")
				self.log.write("INFO", f"{'bDeviceSubClass: ':.<30}{f'{self.desc.bDeviceSubClass:#02x}':.>20}")
				self.log.write("INFO", f"{'bDeviceProtocol: ':.<30}{f'{self.desc.bDeviceProtocol:#02x}':.>20}")
				self.log.write("INFO", f"{'bMaxPacketSize0: ':.<30}{f'{self.desc.bMaxPacketSize0:#02x}':.>20}")
				self.log.write("INFO", f"{'idVendor: ':.<30}{f'{self.desc.idVendor:#02x}':.>20}")
				self.log.write("INFO", f"{'idProduct: ':.<30}{f'{self.desc.idProduct:#02x}':.>20}")
				self.log.write("INFO", f"{'bcdDevice: ':.<30}{f'{self.desc.bcdDevice:#02x}':.>20}")
				self.log.write("INFO", f"{'iManufacturer: ':.<30}{f'{self.desc.iManufacturer:#02x}':.>20}")
				self.log.write("INFO", f"{'iProduct: ':.<30}{f'{self.desc.iProduct:#02x}':.>20}")
				self.log.write("INFO", f"{'iSerialNumber: ':.<30}{f'{self.desc.iSerialNumber:#02x}':.>20}")
				self.log.write("INFO", f"{'bNumConfigurations: ':.<30}{f'{self.desc.bNumConfigurations:#02x}':.>20}")				
				self.dev_found  = True		
				break

			i += 1


		#
		# open device if matching vid/pid was found
		#
		if(self.dev_found  == True):
			self.r = usb.open(self.dev, self.dev_handle)
			if self.r < 0:
				self.log.write("ERROR", f"ret val: {self.r} - {usb.strerror(self.r)}")
				self.log.write("ERROR", "failed to open device!")
				return (1, 4)


			# DEBUG: Get ep size info and configuration
			self.r = usb.get_string_descriptor(self.dev_handle, self.desc.iSerialNumber, 0x409, self.sn_string, 18)
			self.r = usb.get_string_descriptor(self.dev_handle, self.desc.iProduct, 0x409, self.pd_string, 30)
			self.r = usb.get_string_descriptor(self.dev_handle, self.desc.iManufacturer, 0x409, self.mf_string, 26)
			
			self.sn_string_d = bytes(self.sn_string)[2:].decode("utf-16") # type - string
			self.pd_string_d = bytes(self.pd_string)[2:].decode("utf-16") # type - string
			self.mf_string_d = bytes(self.mf_string)[2:].decode("utf-16") # type - string		

			# utf-16 decoding
			# skip first two bytes b/c they are USB protocol stuff not SN
			# - don't really need the below for loop for manual decoding anymore
			self.log.write("INFO", '\n')
			self.log.write("INFO", "/* String descriptor info */")
			self.log.write("INFO", f"{'Manufacturer Description: ':.<30}{self.mf_string_d:.>20}")		
			self.log.write("INFO", f"{'Product Description: ':.<30}{self.pd_string_d:.>20}")
			self.log.write("INFO", f"{'Serial Number: ':.<30}{self.sn_string_d:.>20}")

			if self.r < 0:
				self.log.write("ERROR", f"ret val: {self.r} - {usb.strerror(self.r)}")
				self.log.write("ERROR", "failed to open device", file=sys.stdout)
				return (1, 5)

			# get device info for debugging
			self.log.write("INFO", '\n')
			self.log.write("INFO", "/* Endpoint Sizes */")
			self.ep_size = usb.get_max_packet_size(self.dev, 0x01)
			self.log.write("INFO", f"ep_out_size: {self.ep_size}")
			self.ep_size = usb.get_max_packet_size(self.dev, 0x81)
			self.log.write("INFO", f"ep_in_size: {self.ep_size}")
			self.r = usb.get_configuration(self.dev_handle, self.device_configuration)
			self.log.write("INFO", f"r: {self.r}, configuration: {self.device_configuration.contents}")

			# success - return usb device handle
			return (0, self.dev_handle)

		# ERROR: Failed to find vid/pid
		else:
			self.log.write("ERROR", f'ERROR: failed to find vid: {self.vid}, pid: {self.vid}')
			return (1, 6)

		self.log.write("DEBUG", "<-- Exit open_usb()")





	#------------------------------------------------------------
	# Name: dump_descriptors():
	#
	# Description:
	#   This method prints all USB descriptor information for the 
	#	bridge USB interface.
	#
	# Parameters:
	#	<TBD>
	#
	# Return:
	#	- returns tuple with (<pass/fail flag>, <error_code or data>)
	#	Success: (0, 0)
	#	Failure: (1, <error code>)
	#
	#------------------------------------------------------------
	def dump_descriptors(self, vid=0x1cbf, pid=0x0007):
		self.log.write("DEBUG", "--> Enter open_usb()")
		#
		# callback vars
		#
		#self.dev = None
		#self.sn_string = (ct.c_ubyte* 18)()	# sn string
		#self.pd_string = (ct.c_ubyte* 30)()	# product string
		#self.mf_string = (ct.c_ubyte* 26)()	# manf string
		#self.device_configuration = ct.POINTER(ct.c_int)()	
		#self.dev_handle = ct.POINTER(usb.device_handle)() # creates device handle (not device obj)
		#self.desc = None
		#self.r = None
		#self.vid = vid
		#self.pid = pid


		#
		# inits
		#
		#self.device_configuration.contents = ct.c_int(0)

		# open usb device
		#self.r = usb.init(None)
		#if self.r < 0:
		#	self.log.write("ERROR", f'usb init failure: {self.r}')
		#	return (1, 1)

		#self.devs = ct.POINTER(ct.POINTER(usb.device))() # creates device structure
		#cnt = usb.get_device_list(None, ct.byref(self.devs))

		#if cnt < 0:
		#	self.log.write("ERROR", f'get device list failure: {cnt}')
		#	return (1, 2)

		#self.log.write("INFO", '\n')
		#self.log.write("INFO", "/* Getting USB device list */")

		self.desc = usb.device_descriptor()
		self.r = usb.get_device_descriptor(self.dev, ct.byref(self.desc))

		if self.r < 0:
			self.log.write("ERROR", f'failed to get device descriptor: {self.r}')
			return (1, 3)

		self.log.write("INFO", "{:04x}:{:04x} (bus {:d}, device {:d})".format(
			  self.desc.idVendor, self.desc.idProduct, 
			  usb.get_bus_number(self.dev), usb.get_device_address(self.dev)))


		if(self.desc.idVendor == self.vid) and (self.desc.idProduct == self.pid):
			print('\n')
			print('/* Descriptor Inforamtion */')
			print(f"{'bLength: ':.<30}{f'{self.desc.bLength:#02x}':.>20}")
			print(f"{'bDescriptorType: ':.<30}{f'{self.desc.bDescriptorType:#02x}':.>20}")
			print(f"{'bcdUSB: ':.<30}{f'{self.desc.bcdUSB:#04x}':.>20}")
			print(f"{'bDeviceClass: ':.<30}{f'{self.desc.bDeviceClass:#02x}':.>20}")
			print(f"{'bDeviceSubClass: ':.<30}{f'{self.desc.bDeviceSubClass:#02x}':.>20}")
			print(f"{'bDeviceProtocol: ':.<30}{f'{self.desc.bDeviceProtocol:#02x}':.>20}")
			print(f"{'bMaxPacketSize0: ':.<30}{f'{self.desc.bMaxPacketSize0:#02x}':.>20}")
			print(f"{'idVendor: ':.<30}{f'{self.desc.idVendor:#02x}':.>20}")
			print(f"{'idProduct: ':.<30}{f'{self.desc.idProduct:#02x}':.>20}")
			print(f"{'bcdDevice: ':.<30}{f'{self.desc.bcdDevice:#02x}':.>20}")
			print(f"{'iManufacturer: ':.<30}{f'{self.desc.iManufacturer:#02x}':.>20}")
			print(f"{'iProduct: ':.<30}{f'{self.desc.iProduct:#02x}':.>20}")
			print(f"{'iSerialNumber: ':.<30}{f'{self.desc.iSerialNumber:#02x}':.>20}")
			print(f"{'bNumConfigurations: ':.<30}{f'{self.desc.bNumConfigurations:#02x}':.>20}")				
			#self.log.write("INFO", '\n')
			#self.log.write("INFO", '/* Descriptor Inforamtion */')
			#self.log.write("INFO", f"{'bLength: ':.<30}{f'{self.desc.bLength:#02x}':.>20}")
			#self.log.write("INFO", f"{'bDescriptorType: ':.<30}{f'{self.desc.bDescriptorType:#02x}':.>20}")
			#self.log.write("INFO", f"{'bcdUSB: ':.<30}{f'{self.desc.bcdUSB:#04x}':.>20}")
			#self.log.write("INFO", f"{'bDeviceClass: ':.<30}{f'{self.desc.bDeviceClass:#02x}':.>20}")
			#self.log.write("INFO", f"{'bDeviceSubClass: ':.<30}{f'{self.desc.bDeviceSubClass:#02x}':.>20}")
			#self.log.write("INFO", f"{'bDeviceProtocol: ':.<30}{f'{self.desc.bDeviceProtocol:#02x}':.>20}")
			#self.log.write("INFO", f"{'bMaxPacketSize0: ':.<30}{f'{self.desc.bMaxPacketSize0:#02x}':.>20}")
			#self.log.write("INFO", f"{'idVendor: ':.<30}{f'{self.desc.idVendor:#02x}':.>20}")
			#self.log.write("INFO", f"{'idProduct: ':.<30}{f'{self.desc.idProduct:#02x}':.>20}")
			#self.log.write("INFO", f"{'bcdDevice: ':.<30}{f'{self.desc.bcdDevice:#02x}':.>20}")
			#self.log.write("INFO", f"{'iManufacturer: ':.<30}{f'{self.desc.iManufacturer:#02x}':.>20}")
			#self.log.write("INFO", f"{'iProduct: ':.<30}{f'{self.desc.iProduct:#02x}':.>20}")
			#self.log.write("INFO", f"{'iSerialNumber: ':.<30}{f'{self.desc.iSerialNumber:#02x}':.>20}")
			#self.log.write("INFO", f"{'bNumConfigurations: ':.<30}{f'{self.desc.bNumConfigurations:#02x}':.>20}")
			return (0, 0)
		else:
			print(f'ERROR: Cannot find VID or PID expected, VID={vid}, PID={pid}, found VID={self.desc.idVendor}, PID={self.desc.idProduct}')
			return (1, 1)






	#------------------------------------------------------------
	#
	# Name: write_InternalReg():
	#
	# Description:
	#   Setup and execute a register write to the usb bridge
	#	using the INT0 command/control interface. This function 
	#	will auto read the response received from the bridge
	#	when a register access operation is performed. Each
	#	access has an associated status response from the USB
	#	bridge. 
	#
	# Parameters:
	#	address: 32-bit hex value for register address
	#	mask: 32-bit hex value for data mask
	#	data: 32-bit value to be written to register
	#
	# Return:
	#	- returns tuple with (<pass/fail flag>, <error_code or data>)
	#	Success: (0, <received response>)
	#	Failure: (1, <error information>)
	#
	#------------------------------------------------------------
	def write_InternalReg(self, address, mask, data):
		self.log.write("DEBUG", "--> Enter write_InternalReg()")

		self.EPOUT_ACTIVE = self._EP_INT0_OUT
		self.EPIN_ACTIVE = self._EP_INT0_IN
		# send single packet
		self.EP_SIZE = 64
		# create new buffers
		self.ep_data_out = (ct.c_ubyte*(self.EP_SIZE))()
		self.ep_data_in = (ct.c_ubyte*(self.EP_SIZE))()

		# --------------------------------------
		# Setup first write packet
		# --------------------------------------	

		# setup rd/wr command
		self.ep_data_out[0] = 0x42 & 0xFF
		# setup address data
		self.ep_data_out[1] = address & 0xFF
		self.ep_data_out[2] = (address >> 8) & 0xFF
		self.ep_data_out[3] = (address >> 16) & 0xFF
		self.ep_data_out[4] = (address >> 24) & 0xFF
		# setup mask data
		self.ep_data_out[5] = mask & 0xFF
		self.ep_data_out[6] = (mask >> 8) & 0xFF
		self.ep_data_out[7] = (mask >> 16) & 0xFF
		self.ep_data_out[8] = (mask >> 24) & 0xFF
		# setup data
		self.ep_data_out[9] = data & 0xFF
		self.ep_data_out[10] = (data >> 8) & 0xFF
		self.ep_data_out[11] = (data >> 16) & 0xFF
		self.ep_data_out[12] = (data >> 24) & 0xFF

		usb.claim_interface(self.dev_handle, 0)

		# --------------------------------------
		# Handle Transmit Case
		# --------------------------------------
		self.log.write("INFO", f"INT1 TX, EPIN_ACTIVE: {hex(self.EPIN_ACTIVE)}, len: {len(self.ep_data_in)}")

		r = usb.bulk_transfer(self.dev_handle, self.EPOUT_ACTIVE, self.ep_data_out, 
								self.EP_SIZE, self.bulk_transferred, self.EP_TIMEOUT)

		self.log.write("INFO", f"Write xfer {self.bulk_transferred.contents} bytes!")

		if (r < 0):
			self.log.write("ERROR", f"Total bytes transferred <{self.bulk_transferred.contents}> bytes!")
			self.log.write("ERROR", f"Expected to xfer <{self.EPIN_ACTIVE}> bytes!")
			self.log.write("ERROR", f"bulk_transfer() ret code <{r}> <{usb.error_name(r)}> bytes!")
			return (1, r)
		else:	
			self.log.write("INFO", f"Sent {self.bulk_transferred.contents} bytes!")


		# --------------------------------------
		# Handle Receive Case
		# --------------------------------------
		self.log.write("INFO", f"INT1 RX, EPIN_ACTIVE: {hex(self.EPIN_ACTIVE)}, len: {len(self.ep_data_in)}")

		# send test data
		r = usb.bulk_transfer(self.dev_handle, self.EPIN_ACTIVE, self.ep_data_in, 
								self.EP_SIZE, self.bulk_transferred, self.EP_TIMEOUT)	

		if (r < 0):
			self.log.write("ERROR", f"Total bytes transferred <{self.bulk_transferred.contents}> bytes!")
			self.log.write("ERROR", f"Expected to xfer <{self.EPIN_ACTIVE}> bytes!")
			self.log.write("ERROR", f"bulk_transfer() ret code <{r}> <{usb.error_name(r)}> bytes!")
			return (1, r)
		else:	
			self.log.write("INFO", f"Read xfer {self.bulk_transferred.contents} bytes!")

		self.log.writeUSBPacket("INFO", self.ep_data_in)


		usb.release_interface(self.dev_handle, 0)

		self.log.write("DEBUG", "<-- Exit write_InternalReg()")
		return (0, list(self.ep_data_in))






	#------------------------------------------------------------
	#
	# Name: read_InternalReg():
	#
	# Description:
	#   Read internal register (INT0 interface) and return error
	#	or register read result. 
	#
	# Parameters:
	#	address: 32-bit hex value for register address
	#
	# Return:
	#	- returns tuple with (<pass/fail flag>, <error_code or data>)
	#	Success: (0, (reg hex data, raw usb packet))
	#	Failure: (1, <error information>)
	#
	#------------------------------------------------------------
	def read_InternalReg(self, address):
		self.log.write("DEBUG", "--> Enter read_InternalReg()")

		self.EPOUT_ACTIVE = self._EP_INT0_OUT
		self.EPIN_ACTIVE = self._EP_INT0_IN
		# send single packet
		self.EP_SIZE = 64
		# create new buffers
		self.ep_data_out = (ct.c_ubyte*(self.EP_SIZE))()
		self.ep_data_in = (ct.c_ubyte*(self.EP_SIZE))()

		# --------------------------------------
		# Setup first write packet
		# --------------------------------------	

		# setup rd/wr command
		self.ep_data_out[0] = 0x24 & 0xFF
		# setup address data
		self.ep_data_out[1] = address & 0xFF
		self.ep_data_out[2] = (address >> 8) & 0xFF
		self.ep_data_out[3] = (address >> 16) & 0xFF
		self.ep_data_out[4] = (address >> 24) & 0xFF


		usb.claim_interface(self.dev_handle, 0)

		# --------------------------------------
		# Handle Transmit Case
		# --------------------------------------
		self.log.write("INFO", f"INT1 TX, EPIN_ACTIVE: {hex(self.EPIN_ACTIVE)}, len: {len(self.ep_data_in)}")

		r = usb.bulk_transfer(self.dev_handle, self.EPOUT_ACTIVE, self.ep_data_out, 
								self.EP_SIZE, self.bulk_transferred, self.EP_TIMEOUT)

		self.log.write("INFO", f"Write xfer {self.bulk_transferred.contents} bytes!")

		if (r < 0):
			self.log.write("ERROR", f"Total bytes transferred <{self.bulk_transferred.contents}> bytes!")
			self.log.write("ERROR", f"Expected to xfer <{self.EPIN_ACTIVE}> bytes!")
			self.log.write("ERROR", f"bulk_transfer() ret code <{r}> <{usb.error_name(r)}> bytes!")
			return (1, r)
		else:	
			self.log.write("INFO", f"Sent {self.bulk_transferred.contents} bytes!")


		# --------------------------------------
		# Handle Receive Case
		# --------------------------------------
		self.log.write("INFO", f"INT1 RX, EPIN_ACTIVE: {hex(self.EPIN_ACTIVE)}, len: {len(self.ep_data_in)}")

		# send test data
		r = usb.bulk_transfer(self.dev_handle, self.EPIN_ACTIVE, self.ep_data_in, 
								self.EP_SIZE, self.bulk_transferred, self.EP_TIMEOUT)	

		if (r < 0):
			self.log.write("ERROR", f"Total bytes transferred <{self.bulk_transferred.contents}> bytes!")
			self.log.write("ERROR", f"Expected to xfer <{self.EPIN_ACTIVE}> bytes!")
			self.log.write("ERROR", f"bulk_transfer() ret code <{r}> <{usb.error_name(r)}> bytes!")
			return (1, r)
		else:	
			self.log.write("INFO", f"Read xfer {self.bulk_transferred.contents} bytes!")

		self.log.writeUSBPacket("INFO", self.ep_data_in)

		# parse return value
		hex_value = self.ep_data_in[2]
		hex_value += (self.ep_data_in[3] << 8)
		hex_value += (self.ep_data_in[4] << 16)
		hex_value += (self.ep_data_in[5] << 24)


		usb.release_interface(self.dev_handle, 0)

		self.log.write("DEBUG", "<-- Exit read_InternalReg()")
		return (0, (f"0x{hex_value:08x}", list(self.ep_data_in)))





	#------------------------------------------------------------
	#
	# Name: read_int1():
	#
	# Description:
	#    read 64 byte USB packet from interrupt interface 1.
	#
	# Parameters:
	#	timeout: Amount of time in mS to wait for packet to be
	#		received
	#
	# Return:
	#	- returns tuple with (<pass/fail flag>, <error_code or data>)
	#	Success: (0, <data received>)
	#	Failure: (1, <error information>)
	#
	#------------------------------------------------------------
	def read_int1(self, timeout=250):
		self.log.write("DEBUG", "--> Enter read_int1()")

		self.EPOUT_ACTIVE = self._EP_INT1_OUT
		self.EPIN_ACTIVE = self._EP_INT1_IN
		self.local_EP_SIZE = 64
		self.ep_data_out = (ct.c_ubyte*(self.local_EP_SIZE))()
		self.ep_data_in = (ct.c_ubyte*(self.local_EP_SIZE))()

		usb.claim_interface(self.dev_handle, 1)


		# --------------------------------------
		# Handle Receive Case
		# --------------------------------------
		self.log.write("INFO", "-------- INT1 Report -----------")
		self.log.write("INFO", f"EP1IN_SIZE: {self.local_EP_SIZE}, len: {len(self.ep_data_in)}")

		# receive data
		r = usb.bulk_transfer(self.dev_handle, self.EPIN_ACTIVE, self.ep_data_in, 
								self.local_EP_SIZE, self.bulk_transferred, timeout)	

		if (r < 0):
			self.log.write("ERROR", f"Total bytes transferred <{self.bulk_transferred.contents}> bytes!")
			self.log.write("ERROR", f"Endpoint <{hex(self.EPIN_ACTIVE)}> bytes!")
			self.log.write("ERROR", f"bulk_transfer() ret code <{r}> <{usb.error_name(r)}> bytes!")
			return (1, r)
		else:
			self.log.write("INFO", f"Read xfer {self.bulk_transferred.contents} bytes!")


		self.log.writeUSBPacket("INFO", self.ep_data_in)

		usb.release_interface(self.dev_handle, 1)

		self.log.write("DEBUG", "<-- Exit read_int1()")
		return (0, list(self.ep_data_in))






	#------------------------------------------------------------
	#
	# Name: write_int1():
	#
	# Description:
	#    write 64 byte USB packet to interrupt interface 1.
	#
	# Parameters:
	#	data: data to be sent. Must be in 64 byte blocks.
	#	timeout: Amount of time in mS to wait for packet to be
	#		transmitted
	#
	# Return:
	#	- returns tuple with (<pass/fail flag>, <error_code>)
	#	Success: (0, <0 - no data>)
	#	Failure: (1, <error information>)
	#
	#------------------------------------------------------------
	def write_int1(self, data=False, timeout=250):
		self.log.write("DEBUG", "--> Enter write_int1()")

		self.EPOUT_ACTIVE = self._EP_INT1_OUT
		self.EPIN_ACTIVE = self._EP_INT1_IN
		self.EP_SIZE = 64
		self.ep_data_out = (ct.c_ubyte*(self.EP_SIZE))()
		self.ep_data_in = (ct.c_ubyte*(self.EP_SIZE))()

		usb.claim_interface(self.dev_handle, 1)


		# --------------------------------------
		# Handle Transmist Case
		# --------------------------------------
		self.log.write("INFO", "-------- INT1 Report -----------")

		# make sure data is of type 'list'
		if(isinstance(data, list)):
			pass
		else:
			self.log.write("ERROR", "Data is not of type list")


		# send data over int 1 when payload is passed into function
		if(data != False):

			# check to ensure data is in 64 byte blocks!
			if(len(data) % 64):
				# error - data isn't in blocks of 64 bytes
				self.log.write("ERROR", "Data passed to function is not in 64 byte blocks!")
				return (1, 100)

			self.EP_SIZE = len(data)
			data_s = (ct.c_ubyte*len(data))(*data)

			self.log.write("INFO", f"EP1IN_SIZE: {self.EP_SIZE}, len: {len(data_s)}, timeout: {timeout}")

			# send data
			r = usb.bulk_transfer(self.dev_handle, self.EPOUT_ACTIVE, data_s, 
									self.EP_SIZE, self.bulk_transferred, timeout)	

			if (r < 0):
				self.log.write("ERROR", f"Total bytes transferred <{self.bulk_transferred.contents}> bytes!")
				self.log.write("ERROR", f"Endpoint <{hex(self.EPOUT_ACTIVE)}> bytes!")
				self.log.write("ERROR", f"bulk_transfer() ret code <{r}> <{usb.error_name(r)}> bytes!")
				return (1, r)
			else:
				self.log.write("INFO", f"Write xfer {self.bulk_transferred.contents} bytes!")


		# send data over int 1 when payload isn't passed into function
		else:
			# error - no data was passed in to function 
			return (1, 200)



		
		usb.release_interface(self.dev_handle, 1)

		self.log.write("DEBUG", "<-- Exit write_int1()")
		return (0, 0)






	#------------------------------------------------------------
	#
	# Name: send_bulk():
	#
	# Description:
	#	Send a bulk data transfer. Data size must be in increments
	#	of 64 byte blocks.
	#
	# Parameters:
	#	data: data payload to be transmitted over USB link via
	#	BULK interface (interface 3).
	#	
	#
	# Return:
	#	- returns tuple with (<pass/fail flag>, <error_code or data>)
	#	Success: (0, 0) {no data returned}
	#	Failure: (1, <error information>)
	#
	#------------------------------------------------------------
	def send_bulk(self, data=False, timeout=250, verbose=False, log=False):
		self.log.write("DEBUG", "--> Enter send_bulk()")

		self.EPOUT_ACTIVE = self._EP_BULK_OUT
		self.EPIN_ACTIVE = self._EP_BULK_IN
		self.EP_TIMEOUT = timeout


		usb.claim_interface(self.dev_handle, 2)
		
		if(data != False):
			# ensure data in 64 byte blocks
			if(len(data)%64):
				self.log.write("ERROR", f"Data passed to send_bulk() not 64 byte blocks, mod result <{len(data) % 64}>!")
				return (1, 1)

			self.EP_SIZE = len(data)
			data_s = (ct.c_ubyte*len(data))(*data)

			# send bulk data
			r = usb.bulk_transfer(self.dev_handle, self.EPOUT_ACTIVE, data_s, 
									self.EP_SIZE, self.bulk_transferred, self.EP_TIMEOUT)
			# error check
			if (r < 0):
				self.log.write("ERROR", f"ERROR: Total bytes transferred <{self.bulk_transferred.contents}> bytes!")
				self.log.write("ERROR", f"ERROR: Expected to xfer <{self.EP_SIZE}> bytes!")
				self.log.write("ERROR", f"ERROR: bulk_transfer() ret code <{r}> <{usb.error_name(r)}> bytes!")
				return (1, r)
			else:	
				self.log.write("INFO", f"transferred {self.bulk_transferred.contents} bytes!")

			
		else:
			self.EP_SIZE = 64
			# send bulk data
			r = usb.bulk_transfer(self.dev_handle, self.EPOUT_ACTIVE, self.ep_data_out, 
									self.EP_SIZE, self.bulk_transferred, self.EP_TIMEOUT)		

			# error check
			if (r < 0):
				self.log.write("ERROR", f"ERROR: Total bytes transferred <{self.bulk_transferred.contents}> bytes!")
				self.log.write("ERROR", f"ERROR: Expected to xfer <{self.EP_SIZE}> bytes!")
				self.log.write("ERROR", f"ERROR: bulk_transfer() ret code <{r}> <{usb.error_name(r)}> bytes!")
				return (1, r)
			else:	
				self.log.write("INFO", f"transferred {self.bulk_transferred.contents} bytes!")

		usb.release_interface(self.dev_handle, 2)

		self.log.write("DEBUG", "<-- Exit send_bulk()")
		return (0, 0)







	#------------------------------------------------------------
	#
	# Name: rec_bulk():
	#
	# Description:
	#   Receive a bulk data transfer. Endpoint size must be in
	#	64 byte block increments.
	#
	# Parameters:
	#	timeout: timeout for reception in mS
	#	ep_size: endpoint size/data transfer size (must be 64 byte
	#		increments). Data is transferred over interface 3.
	#	
	#
	# Return:
	#	- returns tuple with (<pass/fail flag>, <error_code or data>)
	#	Success: (0, received data)
	#	Failure: (1, error flag)
	#
	#------------------------------------------------------------
	def rec_bulk(self, timeout=250, ep_size=64):
		self.log.write("DEBUG", "--> Enter rec_bulk()")

		self.EPOUT_ACTIVE = self._EP_BULK_OUT
		self.EPIN_ACTIVE = self._EP_BULK_IN
		self.EP_TIMEOUT = timeout

		# adjust endpoint size and buffer if needed
		if(ep_size != 64):
			self.EP_SIZE_BULK = ep_size
			self.ep_data_in = (ct.c_ubyte*(self.EP_SIZE_BULK))()
		else:
			self.EP_SIZE_BULK = 64
			self.ep_data_in = (ct.c_ubyte*(self.EP_SIZE_BULK))()

		usb.claim_interface(self.dev_handle, 2)
		

		# read bulk data
		r = usb.bulk_transfer(self.dev_handle, self.EPIN_ACTIVE, self.ep_data_in, 
									self.EP_SIZE_BULK, self.bulk_transferred, self.EP_TIMEOUT)	
		# error check
		if (r < 0):
			self.log.write("ERROR", f"ERROR: Total bytes transferred <{self.bulk_transferred.contents}> bytes!")
			self.log.write("ERROR", f"ERROR: Expected to xfer <{self.EP_SIZE_BULK}> bytes!")
			self.log.write("ERROR", f"ERROR: bulk_transfer() ret code <{r}> <{usb.error_name(r)}> bytes!")
			return (1, r)
		else:
			self.log.write("INFO", f"Received {self.bulk_transferred.contents} bytes!")	


		usb.release_interface(self.dev_handle, 2)

		self.log.write("DEBUG", "<-- Exit rec_bulk()")

		return (0, list(self.ep_data_in))






	#------------------------------------------------------------
	#
	# Name: close_usb():
	#
	# Description:
	#    Close USB connection for self.dev_handle
	#
	# Parameters:
	#	
	#	
	#
	# Return:
	#	NA 
	#
	#------------------------------------------------------------
	def close_usb(self):
		self.log.write("DEBUG", "--> Enter close_usb()")

		

		self.log.write("DEBUG", "<-- Exit close_usb()")
		self.log.shutdown_logging()

		usb.close(self.dev_handle)





	#------------------------------------------------------------
	#
	# Name: dump_regspace():
	#
	# Description:
	#   Dump register space to console by reading each register,
	#	check for read error and parse result if error free.
	#
	# Parameters:
	#	None
	#	
	#
	# Return:
	#	NA
	#
	#------------------------------------------------------------
	# self.log.write("INFO", f"{'bLength: ':.<30}{f'{self.desc.bLength:#02x}':.>20}")
	def dump_regspace(self):

		# read CR1
		r = self.read_InternalReg(self.CR1_ADDR)
		# check for error
		if(r[0]):
			print(f"ERROR: libusb ret code <{r[1]}> <{usb.error_name(r[1])}> bytes!")			
		else:
			print(f"Address: {self.CR1_ADDR:#08x}, Value: {r[1][0]}")

		# read CR2
		r = self.read_InternalReg(self.CR2_ADDR)
		# check for error
		if(r[0]):
			print(f"ERROR: libusb ret code <{r[1]}> <{usb.error_name(r[1])}> bytes!")			
		else:
			print(f"Address: {self.CR2_ADDR:#08x}, Value: {r[1][0]}")

		# read SR1
		r = self.read_InternalReg(self.SR1_ADDR)
		# check for error
		if(r[0]):
			print(f"ERROR: libusb ret code <{r[1]}> <{usb.error_name(r[1])}> bytes!")			
		else:
			print(f"Address: {self.SR1_ADDR:#08x}, Value: {r[1][0]}")

		# read SR2
		r = self.read_InternalReg(self.SR2_ADDR)
		# check for error
		if(r[0]):
			print(f"ERROR: libusb ret code <{r[1]}> <{usb.error_name(r[1])}> bytes!")			
		else:
			print(f"Address: {self.SR2_ADDR:#08x}, Value: {r[1][0]}")

		# read ASR
		r = self.read_InternalReg(self.ASR_ADDR)
		# check for error
		if(r[0]):
			print(f"ERROR: libusb ret code <{r[1]}> <{usb.error_name(r[1])}> bytes!")			
		else:
			print(f"Address: {self.ASR_ADDR:#08x}, Value: {r[1][0]}")

		# read SKEY
		r = self.read_InternalReg(self.SKEY_ADDR)
		# check for error
		if(r[0]):
			print(f"ERROR: libusb ret code <{r[1]}> <{usb.error_name(r[1])}> bytes!")			
		else:
			print(f"Address: {self.SKEY_ADDR:#08x}, Value: {r[1][0]}")

		# read USBBLKSR
		r = self.read_InternalReg(self.USBBLKSR_ADDR)
		# check for error
		if(r[0]):
			print(f"ERROR: libusb ret code <{r[1]}> <{usb.error_name(r[1])}> bytes!")			
		else:
			print(f"Address: {self.USBBLKSR_ADDR:#08x}, Value: {r[1][0]}")

		# read USBINT0SR
		r = self.read_InternalReg(self.USBINT0SR_ADDR)
		# check for error
		if(r[0]):
			print(f"ERROR: libusb ret code <{r[1]}> <{usb.error_name(r[1])}> bytes!")			
		else:
			print(f"Address: {self.USBINT0SR_ADDR:#08x}, Value: {r[1][0]}")

		# read USBINT1SR
		r = self.read_InternalReg(self.USBINT1SR_ADDR)
		# check for error
		if(r[0]):
			print(f"ERROR: libusb ret code <{r[1]}> <{usb.error_name(r[1])}> bytes!")			
		else:
			print(f"Address: {self.USBINT1SR_ADDR:#08x}, Value: {r[1][0]}")

		# read USBFBRXSR
		r = self.read_InternalReg(self.USBFBRXSR_ADDR)
		# check for error
		if(r[0]):
			print(f"ERROR: libusb ret code <{r[1]}> <{usb.error_name(r[1])}> bytes!")			
		else:
			print(f"Address: {self.USBFBRXSR_ADDR:#08x}, Value: {r[1][0]}")

		# read USBFBTXSR
		r = self.read_InternalReg(self.USBFBTXSR_ADDR)
		# check for error
		if(r[0]):
			print(f"ERROR: libusb ret code <{r[1]}> <{usb.error_name(r[1])}> bytes!")			
		else:
			print(f"Address: {self.USBFBTXSR_ADDR:#08x}, Value: {r[1][0]}")

		# read USBBLKRXFC
		r = self.read_InternalReg(self.USBBLKRXFC_ADDR)
		# check for error
		if(r[0]):
			print(f"ERROR: libusb ret code <{r[1]}> <{usb.error_name(r[1])}> bytes!")			
		else:
			print(f"Address: {self.USBBLKRXFC_ADDR:#08x}, Value: {r[1][0]}")

		# read USBIF0RXFC
		r = self.read_InternalReg(self.USBIF0RXFC_ADDR)
		# check for error
		if(r[0]):
			print(f"ERROR: libusb ret code <{r[1]}> <{usb.error_name(r[1])}> bytes!")			
		else:
			print(f"Address: {self.USBIF0RXFC_ADDR:#08x}, Value: {r[1][0]}")

		# read USBIF1RXFC
		r = self.read_InternalReg(self.USBIF1RXFC_ADDR)
		# check for error
		if(r[0]):
			print(f"ERROR: libusb ret code <{r[1]}> <{usb.error_name(r[1])}> bytes!")			
		else:
			print(f"Address: {self.USBIF1RXFC_ADDR:#08x}, Value: {r[1][0]}")

		# read NVMEMSR
		r = self.read_InternalReg(self.NVMEMSR_ADDR)
		# check for error
		if(r[0]):
			print(f"ERROR: libusb ret code <{r[1]}> <{usb.error_name(r[1])}> bytes!")			
		else:
			print(f"Address: {self.NVMEMSR_ADDR:#08x}, Value: {r[1][0]}")

		# read SSITXFC
		r = self.read_InternalReg(self.SSITXFC_ADDR)
		# check for error
		if(r[0]):
			print(f"ERROR: libusb ret code <{r[1]}> <{usb.error_name(r[1])}> bytes!")			
		else:
			print(f"Address: {self.SSITXFC_ADDR:#08x}, Value: {r[1][0]}")

		# read SSITXEC
		r = self.read_InternalReg(self.SSITXEC_ADDR)
		# check for error
		if(r[0]):
			print(f"ERROR: libusb ret code <{r[1]}> <{usb.error_name(r[1])}> bytes!")			
		else:
			print(f"Address: {self.SSITXEC_ADDR:#08x}, Value: {r[1][0]}")

		# read SSIRXFC
		r = self.read_InternalReg(self.SSIRXFC_ADDR)
		# check for error
		if(r[0]):
			print(f"ERROR: libusb ret code <{r[1]}> <{usb.error_name(r[1])}> bytes!")			
		else:
			print(f"Address: {self.SSIRXFC_ADDR:#08x}, Value: {r[1][0]}")

		# read SSIRXEC
		r = self.read_InternalReg(self.SSIRXEC_ADDR)
		# check for error
		if(r[0]):
			print(f"ERROR: libusb ret code <{r[1]}> <{usb.error_name(r[1])}> bytes!")			
		else:
			print(f"Address: {self.SSIRXEC_ADDR:#08x}, Value: {r[1][0]}")
		
		# read SCRTCH1
		r = self.read_InternalReg(self.SCRTCH1_ADDR)
		# check for error
		if(r[0]):
			print(f"ERROR: libusb ret code <{r[1]}> <{usb.error_name(r[1])}> bytes!")			
		else:
			print(f"Address: {self.SCRTCH1_ADDR:#08x}, Value: {r[1][0]}")

		# read SCRTCH2
		r = self.read_InternalReg(self.SCRTCH2_ADDR)
		# check for error
		if(r[0]):
			print(f"ERROR: libusb ret code <{r[1]}> <{usb.error_name(r[1])}> bytes!")			
		else:
			print(f"Address: {self.SCRTCH2_ADDR:#08x}, Value: {r[1][0]}")

		# read SCRTCH3
		r = self.read_InternalReg(self.SCRTCH3_ADDR)
		# check for error
		if(r[0]):
			print(f"ERROR: libusb ret code <{r[1]}> <{usb.error_name(r[1])}> bytes!")			
		else:
			print(f"Address: {self.SCRTCH3_ADDR:#08x}, Value: {r[1][0]}")			

		# read SCRTCH4
		r = self.read_InternalReg(self.SCRTCH4_ADDR)
		# check for error
		if(r[0]):
			print(f"ERROR: libusb ret code <{r[1]}> <{usb.error_name(r[1])}> bytes!")			
		else:
			print(f"Address: {self.SCRTCH4_ADDR:#08x}, Value: {r[1][0]}")			

		# read SSITXLGSTS
		r = self.read_InternalReg(self.SSITXLGSTS_ADDR)
		# check for error
		if(r[0]):
			print(f"ERROR: libusb ret code <{r[1]}> <{usb.error_name(r[1])}> bytes!")			
		else:
			print(f"Address: {self.SSITXLGSTS_ADDR:#08x}, Value: {r[1][0]}")

		# read SSIRXLGSTS
		r = self.read_InternalReg(self.SSIRXLGSTS_ADDR)
		# check for error
		if(r[0]):
			print(f"ERROR: libusb ret code <{r[1]}> <{usb.error_name(r[1])}> bytes!")			
		else:
			print(f"Address: {self.SSIRXLGSTS_ADDR:#08x}, Value: {r[1][0]}")

		# read SIRXFSSCNT
		r = self.read_InternalReg(self.SIRXFSSCNT_ADDR)
		# check for error
		if(r[0]):
			print(f"ERROR: libusb ret code <{r[1]}> <{usb.error_name(r[1])}> bytes!")			
		else:
			print(f"Address: {self.SIRXFSSCNT_ADDR:#08x}, Value: {r[1][0]}")

		# read CTRTXDATA0
		r = self.read_InternalReg(self.CTRTXDATA0_ADDR)
		# check for error
		if(r[0]):
			print(f"ERROR: libusb ret code <{r[1]}> <{usb.error_name(r[1])}> bytes!")			
		else:
			print(f"Address: {self.CTRTXDATA0_ADDR:#08x}, Value: {r[1][0]}")

		# read CTRTXDATA1
		r = self.read_InternalReg(self.CTRTXDATA1_ADDR)
		# check for error
		if(r[0]):
			print(f"ERROR: libusb ret code <{r[1]}> <{usb.error_name(r[1])}> bytes!")			
		else:
			print(f"Address: {self.CTRTXDATA1_ADDR:#08x}, Value: {r[1][0]}")

		# read CTRTXDATA2
		r = self.read_InternalReg(self.CTRTXDATA2_ADDR)
		# check for error
		if(r[0]):
			print(f"ERROR: libusb ret code <{r[1]}> <{usb.error_name(r[1])}> bytes!")			
		else:
			print(f"Address: {self.CTRTXDATA2_ADDR:#08x}, Value: {r[1][0]}")

		# read CTRTXDATA3
		r = self.read_InternalReg(self.CTRTXDATA3_ADDR)
		# check for error
		if(r[0]):
			print(f"ERROR: libusb ret code <{r[1]}> <{usb.error_name(r[1])}> bytes!")			
		else:
			print(f"Address: {self.CTRTXDATA3_ADDR:#08x}, Value: {r[1][0]}")

		# read CTRTXDATA4
		r = self.read_InternalReg(self.CTRTXDATA4_ADDR)
		# check for error
		if(r[0]):
			print(f"ERROR: libusb ret code <{r[1]}> <{usb.error_name(r[1])}> bytes!")			
		else:
			print(f"Address: {self.CTRTXDATA4_ADDR:#08x}, Value: {r[1][0]}")

		# read CTRTXDATA5
		r = self.read_InternalReg(self.CTRTXDATA5_ADDR)
		# check for error
		if(r[0]):
			print(f"ERROR: libusb ret code <{r[1]}> <{usb.error_name(r[1])}> bytes!")			
		else:
			print(f"Address: {self.CTRTXDATA5_ADDR:#08x}, Value: {r[1][0]}")

		# read CTRTXDATA6
		r = self.read_InternalReg(self.CTRTXDATA6_ADDR)
		# check for error
		if(r[0]):
			print(f"ERROR: libusb ret code <{r[1]}> <{usb.error_name(r[1])}> bytes!")			
		else:
			print(f"Address: {self.CTRTXDATA6_ADDR:#08x}, Value: {r[1][0]}")

		# read CTRTXDATA7
		r = self.read_InternalReg(self.CTRTXDATA7_ADDR)
		# check for error
		if(r[0]):
			print(f"ERROR: libusb ret code <{r[1]}> <{usb.error_name(r[1])}> bytes!")			
		else:
			print(f"Address: {self.CTRTXDATA7_ADDR:#08x}, Value: {r[1][0]}")

		# read CTRRXDATA0
		r = self.read_InternalReg(self.CTRRXDATA0_ADDR)
		# check for error
		if(r[0]):
			print(f"ERROR: libusb ret code <{r[1]}> <{usb.error_name(r[1])}> bytes!")			
		else:
			print(f"Address: {self.CTRRXDATA0_ADDR:#08x}, Value: {r[1][0]}")

		# read CTRRXDATA1
		r = self.read_InternalReg(self.CTRRXDATA1_ADDR)
		# check for error
		if(r[0]):
			print(f"ERROR: libusb ret code <{r[1]}> <{usb.error_name(r[1])}> bytes!")			
		else:
			print(f"Address: {self.CTRRXDATA1_ADDR:#08x}, Value: {r[1][0]}")

		# read CTRRXDATA2
		r = self.read_InternalReg(self.CTRRXDATA2_ADDR)
		# check for error
		if(r[0]):
			print(f"ERROR: libusb ret code <{r[1]}> <{usb.error_name(r[1])}> bytes!")			
		else:
			print(f"Address: {self.CTRRXDATA2_ADDR:#08x}, Value: {r[1][0]}")

		# read CTRRXDATA3
		r = self.read_InternalReg(self.CTRRXDATA3_ADDR)
		# check for error
		if(r[0]):
			print(f"ERROR: libusb ret code <{r[1]}> <{usb.error_name(r[1])}> bytes!")			
		else:
			print(f"Address: {self.CTRRXDATA3_ADDR:#08x}, Value: {r[1][0]}")

		# read CTRRXDATA4
		r = self.read_InternalReg(self.CTRRXDATA4_ADDR)
		# check for error
		if(r[0]):
			print(f"ERROR: libusb ret code <{r[1]}> <{usb.error_name(r[1])}> bytes!")			
		else:
			print(f"Address: {self.CTRRXDATA4_ADDR:#08x}, Value: {r[1][0]}")

		# read CTRRXDATA5
		r = self.read_InternalReg(self.CTRRXDATA5_ADDR)
		# check for error
		if(r[0]):
			print(f"ERROR: libusb ret code <{r[1]}> <{usb.error_name(r[1])}> bytes!")			
		else:
			print(f"Address: {self.CTRRXDATA5_ADDR:#08x}, Value: {r[1][0]}")

		# read CTRRXDATA6
		r = self.read_InternalReg(self.CTRRXDATA6_ADDR)
		# check for error
		if(r[0]):
			print(f"ERROR: libusb ret code <{r[1]}> <{usb.error_name(r[1])}> bytes!")			
		else:
			print(f"Address: {self.CTRRXDATA6_ADDR:#08x}, Value: {r[1][0]}")

		# read CTRRXDATA7
		r = self.read_InternalReg(self.CTRRXDATA7_ADDR)
		# check for error
		if(r[0]):
			print(f"ERROR: libusb ret code <{r[1]}> <{usb.error_name(r[1])}> bytes!")			
		else:
			print(f"Address: {self.CTRRXDATA7_ADDR:#08x}, Value: {r[1][0]}")

		# read CTRMODECR
		r = self.read_InternalReg(self.CTRMODECR_ADDR)
		# check for error
		if(r[0]):
			print(f"ERROR: libusb ret code <{r[1]}> <{usb.error_name(r[1])}> bytes!")			
		else:
			print(f"Address: {self.CTRMODECR_ADDR:#08x}, Value: {r[1][0]}")

#------------------------------------------------------------
#
# Name: reg_print():
#
# Description:
#   Print formated register detailed bit information. This is
#	for the reg_access.py module.
#
# Parameters:
#	Register ADDR
#	
#
# Return:
#	NA
#	Just prints string to console.
#------------------------------------------------------------
def reg_print(self, addr):
	if(addr == 0):
		print("reg 0")