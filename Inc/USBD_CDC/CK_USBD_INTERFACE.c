
#include "USBD_CDC/usbd_core.h"
#include "USBD_CDC/usbd_desc.h"
#include "USBD_CDC/usbd_cdc.h"
#include "USBD_CDC/usbd_cdc_if.h"
#include "USBD_CDC/CK_USBD_BUFFER.h"
#include "USBD_CDC/CK_USBD_INTERFACE.h"

#define BUFFER_SIZE		512

/*
 *  In this mode the transfer data is written to transfer buffer directly
 *  Write more than BUFFER_SIZE will be ignored.
 *  In this way memcpy time is not needed. Data is ready to transfer without second operation.
 */
#define FAST_TRANSFER

USBD_HandleTypeDef hUsbDeviceFS; // USB Device Core handle declaration

circularBuffer_t usbd_transfer_cb;

circularBuffer_t usbd_receive_cb;

uint8_t copy_buffer[BUFFER_SIZE];

uint32_t copy_length;

void CK_USBD_Init(void){

	// Init Device Library, add supported class and start the library
	if (USBD_Init(&hUsbDeviceFS, &FS_Desc, DEVICE_FS) != USBD_OK){
		//Error_Handler();
	}
	if (USBD_RegisterClass(&hUsbDeviceFS, &USBD_CDC) != USBD_OK){
		//Error_Handler();
	}
	if (USBD_CDC_RegisterInterface(&hUsbDeviceFS, &USBD_Interface_fops_FS) != USBD_OK){
		//Error_Handler();
	}
	if (USBD_Start(&hUsbDeviceFS) != USBD_OK){
		//Error_Handler();
	}

	CK_USBD_BUFFER_Init(&usbd_transfer_cb, BUFFER_SIZE);

	CK_USBD_BUFFER_Init(&usbd_receive_cb, BUFFER_SIZE);
}

void CK_USBD_Start(void){

	USBD_Start(&hUsbDeviceFS);
}

void CK_USBD_Stop(void){
	USBD_Stop(&hUsbDeviceFS);
}

int CK_USBD_Transmit(void){

#if defined(FAST_TRANSFER)

	// When busy it does not print but since data is written to buffer
	// next CK_USBD_Transmit() transfers all.
	// CK_USBD_Transmit() can be updated periodically with a timer interrupt.
	if(CDC_Transmit_FS(copy_buffer, copy_length) == USBD_OK){
		copy_length = 0;
		return 1; // OK
	}
#else

	CK_USBD_BUFFER_GetBuffer(&usbd_transfer_cb, copy_buffer, &copy_length);

	if(copy_length != 0){
		if(CDC_Transmit_FS(copy_buffer, copy_length) == USBD_OK){

			return 1; // OK
		}
	}

#endif
	return 0;

}

int CK_USBD_ReadData(uint8_t* data){

	// I did not implemented a method for returning whole received buffer.
	// Reading data one byte at a time
	if(!CK_USBD_BUFFER_IsBufferEmpty(&usbd_receive_cb)){
		CK_USBD_BUFFER_BufferRead(&usbd_receive_cb, data);
		return 1;
	}

	return 0; // Data is not available
}

int CK_USBD_WriteRxCircularBuffer(uint8_t* Buf, uint32_t* Len){

	// Later add FAST_TRANSFER mode for reception as well where
	// array should be copied to circular buffer directly once.

	uint32_t receivedLength = *Len; // point to the received length address which is the length number.

	for(int i = 0; i < receivedLength; i++){
		if(!CK_USBD_BUFFER_IsBufferFull(&usbd_receive_cb)){
			uint8_t data = Buf[i];
			CK_USBD_BUFFER_BufferWrite(&usbd_receive_cb, data);
		}
		else{
			return 0; // Full, could not write
		}
	}

	return 1; // OK
}

int CK_USBD_WriteTxCircularBuffer(uint8_t data){

#if defined(FAST_TRANSFER)

	if(copy_length <= BUFFER_SIZE){
		copy_buffer[copy_length++] = data;
		return 1; // OK
	}

#else

	if(!CK_USBD_BUFFER_IsBufferFull(&usbd_transfer_cb)){
		CK_USBD_BUFFER_BufferWrite(&usbd_transfer_cb, data);
		return 1; // OK
	}

#endif

	return 0; // Full, could not write

}

void CK_USBD_IntPrint(int32_t num){
	if(num < 0){
		CK_USBD_WriteTxCircularBuffer((uint8_t)'-');
		num *= -1;
	}
	int32_t tmp[10];
	int numOfDigits = 0;
	int limit = 10;

	for(int i=0; i < limit; i++){
		if(num >= 10){
			tmp[i] = (int32_t)num % 10;
			num = num - tmp[i];
			num = num / 10;
		}
		else{
			tmp[i] = num;
			numOfDigits = i;
			i = limit;
		}
	}
	for(int i = numOfDigits; i >= 0; i--){
		CK_USBD_WriteTxCircularBuffer((uint8_t)tmp[i] + 48); //for ASCII code of number
	}
}

void CK_USBD_IntPrintln(int32_t num){
	if(num < 0){
		CK_USBD_WriteTxCircularBuffer((uint8_t)'-');
		num *= -1;
	}
	int32_t tmp[10];
	int numOfDigits = 0;
	int limit = 10;

	for(int i = 0; i < limit; i++){
		if(num >= 10){
			tmp[i] = (int32_t)num % 10;
			num = num - tmp[i];
			num = num / 10;
		}
		else{
			tmp[i] = num;
			numOfDigits = i;
			i = limit;
		}
	}
	for(int i=numOfDigits; i >= 0; i--){
		CK_USBD_WriteTxCircularBuffer((uint8_t)tmp[i] + 48); //for ASCII code of number
	}

	CK_USBD_WriteTxCircularBuffer((uint8_t)'\n'); //for ASCII line feed
	CK_USBD_WriteTxCircularBuffer((uint8_t)'\r'); //for ASCII carriage return

}

void CK_USBD_FloatPrintln(float num){
	int flag = 0;
	num = num * 100; // 2 digit after comma
	int intnum = (int)num;
	const int limit = 10; // 10 digit number max
	if(intnum < 0){
		CK_USBD_WriteTxCircularBuffer((uint8_t)'-');
		intnum *= -1;
	}
	if(intnum < 10){ // 0.00
		flag = 1;
	}
	else if(intnum >= 10 && intnum <= 99){ // 0.01 to 0.99
		flag = 2;
	}

	int tmp[limit];
	int numOfDigits = 0;
	for(int i = 0; i < limit; i++){
		if(intnum>=10){
			tmp[i] = (int)intnum % 10;
			intnum = intnum - tmp[i];
			intnum = intnum / 10;
		}
		else{
			tmp[i] = intnum;
			numOfDigits = i; // get num of digits
			i = limit; // end loop
		}
	}
	if(flag==1){ // make 0.00
		tmp[1] = 0;tmp[2] = 0;
		numOfDigits = 2;
	}
	if(flag==2){ // add 0 for 0.01 to 0.99 numbers
		tmp[++numOfDigits] = 0;
	}

	for(int i=numOfDigits; i >= 0; i--){
		if(i>1){
			CK_USBD_WriteTxCircularBuffer((uint8_t)tmp[i] + 48); //for ASCII code of number
		}
		else if(i==1){
			CK_USBD_WriteTxCircularBuffer((uint8_t)46); //for ASCII '.'
			CK_USBD_WriteTxCircularBuffer((uint8_t)tmp[i] + 48); //for ASCII code of number
		}
		else{
			CK_USBD_WriteTxCircularBuffer((uint8_t)tmp[i] + 48); //for ASCII code of number
		}
	}
	CK_USBD_WriteTxCircularBuffer((uint8_t)'\n'); //for ASCII line feed
	CK_USBD_WriteTxCircularBuffer((uint8_t)'\r'); //for ASCII carriage return

}

void CK_USBD_FloatPrint(float num){
	int flag = 0;
	num = num * 100; // 2 digit after comma
	int intnum = (int)num;
	const int limit = 10; // 10 digit number max
	if(intnum < 0){
		CK_USBD_WriteTxCircularBuffer((uint8_t)'-');
		intnum *= -1;
	}
	if(intnum < 10){ // 0.00
		flag = 1;
	}
	else if(intnum >= 10 && intnum <= 99){ // 0.01 to 0.99
		flag = 2;
	}

	int tmp[limit];
	int numOfDigits = 0;
	for(int i = 0; i < limit; i++){
		if(intnum >= 10){
			tmp[i] = (int)intnum % 10;
			intnum = intnum - tmp[i];
			intnum = intnum / 10;
		}
		else{
			tmp[i] = intnum;
			numOfDigits = i; // get num of digits
			i = limit; // end loop
		}
	}
	if(flag == 1){ // make 0.00
		tmp[1] = 0;tmp[2] = 0;
		numOfDigits = 2;
	}
	if(flag == 2){ // add 0 for 0.01 to 0.99 numbers
		tmp[++numOfDigits] = 0;
	}

	for(int i = numOfDigits; i >= 0; i--){
		if(i > 1){
			CK_USBD_WriteTxCircularBuffer((uint8_t)tmp[i] + 48); //for ASCII code of number
		}
		else if(i == 1){
			CK_USBD_WriteTxCircularBuffer((uint8_t)46); //for ASCII '.'
			CK_USBD_WriteTxCircularBuffer((uint8_t)tmp[i] + 48); //for ASCII code of number
		}
		else{
			CK_USBD_WriteTxCircularBuffer((uint8_t)tmp[i] + 48); //for ASCII code of number
		}
	}

}

void CK_USBD_StringPrintln(const char str[]){
	int size = strlen(str);
	for(int i = 0; i < size; i++){
		CK_USBD_WriteTxCircularBuffer((uint8_t)str[i]);
	}
	CK_USBD_WriteTxCircularBuffer((uint8_t)'\n'); //for ASCII line feed
	CK_USBD_WriteTxCircularBuffer((uint8_t)'\r'); //for ASCII carriage return

}

void CK_USBD_StringPrint(const char str[]){
	int size = strlen(str);
	for(int i = 0; i < size; i++){
		CK_USBD_WriteTxCircularBuffer((uint8_t)str[i]);
	}

}




