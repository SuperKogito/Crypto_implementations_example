/*
 * This file contains an example of how a Contiki program using various
 * looks Cryptography APIs looks like.
 *
 * The program gives and example implementations of the following:
 * TI aes-128 (ecb mode) + TI sha-256
 * Mbedtls aes128 (cbc mode) + Mbedtls sha256
 * Contiki aes-128 (ecb mode)
 */
#include "contiki.h"
#include "contiki-net.h"

/*
 * Cryptography APIs headers
 */
#include "lib/aes-128.h"
#include "lib/mbedtls/aes.h"
#include "lib/ti/TI_SHA_256.h"
#include "lib/mbedtls/sha256.h"
#include "lib/ti/TI_aes_128_encr_only.h"

/**
 * Define used functions
 */

/**
 * [Print uint8_t arrays]
 * @method uint8_print
 * @param  hint        [explanation message]
 * @param  data        [data buffer]
 * @param  size        [size of data to print]
 */
void uint8_print(char *hint, uint8_t *data, int size){
								printf(hint);
								for(uint8_t i = 0; i < size; i+=4) {
																printf("%02X%02X%02X%02X ", data[i], data[i+1], data[i+2], data[i+3]);
								}
								printf("\r\n");
}

/**
 * [Print test header message]
 * @method custom_print
 * @param  hint         [header message]
 */
void custom_print(char *hint){
								printf("\r\n---------------------------------------------------------------------------------------------\r\n");
								printf(hint);
								printf("\r\n---------------------------------------------------------------------------------------------\r\n");
}

/**
 * [Copy the content of buffer one in buffer two at a specific position]
 * @method shifted_buffer1_to_buffer2
 * @param  src                        [source buffer to copy from]
 * @param  dst                        [destination buffer to copy to]
 * @param  src_start_pos              [start position of copying]
 * @param  src_end_pos                [end position of copying]
 * @param  dst_start_pos              [start position of pasting]
 * @param  dst_end_pos                [end position of pasting]
 */
void shifted_buffer1_to_buffer2(uint8_t* src, uint8_t* dst, int src_start_pos, int src_end_pos, int dst_start_pos, int dst_end_pos){
								int i = src_start_pos;
								for (int j = dst_start_pos; j < dst_end_pos; j++) {
																dst[j] = src[i];
																i++;
								}
}

/**
 * [convert uint32_t buffer to uint8_t]
 * @method uint32_to_uint8
 * @param  src             [source uint32_t buffer]
 * @param  dst             [destination uint8_t buffer]
 * @param  size            [size of source buffer]
 */
void uint32_to_uint8(uint32_t* src, uint8_t* dst, int size){
								int k = 0;
								for(int j=0; j<size; j+=4) {
																dst[j+0] = (src[k] >> 24);
																dst[j+1] = (src[k] >> 16);
																dst[j+2] = (src[k] >> 8);
																dst[j+3] = (src[k] >> 0);
																k++;
								}
}

/**
 * [convert uint8_t buffer to uint32_t]
 * @method uint8_to_uint32
 * @param  src             [source uint8_t buffer]
 * @param  dst             [destination uint32_t buffer]
 * @param  size            [uint8_t buffer size]
 */
void uint8_to_uint32(uint8_t* src, uint32_t* dst, int size){
								int k = 0;
								for(int j=0; j<size; j+=4) {
																dst[k] = src[j];
																dst[k] = (dst[k] << 8) + src[j+1];
																dst[k] = (dst[k] << 8) + src[j+2];
																dst[k] = (dst[k] << 8) + src[j+3];
																k++;
								}
}

/**
 * [Call TI aes-128 ecb mode]
 * @method ti_aes
 * @param  TI_aes128_input [plain data input]
 * @param  ti_key          [aes key]
 */
void ti_aes(uint8_t* TI_aes128_input, uint8_t* ti_key){
								custom_print((char*) "TEST OF TI AES-128 (ECB mode) API");
								uint8_t TI_aes128_output[32] = {0};

								// Define variables
								int block_size = 16;
								uint8_t uint8_temporary_buffer[block_size];
								uint8_t uint8_temporary_key[block_size];

								// Encrypt data block by block (32 bits = 2 blocks) using ECB mode
								for (int i = 0; i < 32; i+= block_size) {

																// Extract the 16 bytes chunk and converting it to adequate processing type
																shifted_buffer1_to_buffer2(TI_aes128_input, uint8_temporary_buffer, i, i+16, 0, 16);
																shifted_buffer1_to_buffer2(ti_key, uint8_temporary_key, 0, 16, 0, 16);

																unsigned char* char_temporary_buffer = (unsigned char*) uint8_temporary_buffer;
																unsigned char* char_temporary_key = (unsigned char*) uint8_temporary_key;

																// Encrypt chunk using aes-128 in ECB mode
																aes_encrypt(char_temporary_buffer, char_temporary_key);

																// Re-converting of encrypted chunk and parsing of output
																uint8_t* uint8_temp_buffer = (uint8_t*) char_temporary_buffer;
																shifted_buffer1_to_buffer2(uint8_temp_buffer, TI_aes128_output, 0, 16, i, i+16);
								}

								// Prints
								uint8_print((char*)"TI-aes128 key   :", ti_key,  16);
								uint8_print((char*)"TI-aes128 input :", TI_aes128_input,  32);
								uint8_print((char*)"TI-aes128 output:", TI_aes128_output, 32);
}

/**
 * [Call TI sha-256]
 * @method ti_sha256
 * @param  TI_sha256_input [hashing input data]
 */
void ti_sha256(uint8_t* TI_sha256_input){
								custom_print((char*) "TEST OF TI SHA-256 API");
								uint8_t TI_sha256_output[32] = {0};

								// Define variables
								uint32_t hashing_input_buffer[32];
								uint32_t hashing_output_buffer[8];
								uint64_t data_size_in_bits = 0x100;

								// Convert uint8_t variable to uint32_t	+ Hash data + Convert uint32_t variable to uint8_t
								uint8_to_uint32(TI_sha256_input, hashing_input_buffer, 32);
								SHA_256(hashing_input_buffer, data_size_in_bits, hashing_output_buffer, 1);
								uint32_to_uint8(hashing_output_buffer, TI_sha256_output, 32);

								// Prints
								uint8_print((char*)"TI-sha256 input :", TI_sha256_input,  32);
								uint8_print((char*)"TI-sha256 output:", TI_sha256_output, 32);
}

/**
 * [Call mbedtls aes-128 cbc mode]
 * @method mbed_aes128
 * @param  Mbedtls_aes128_input [plain input data]
 * @param  uint8_aes128_key     [aes key]
 * @param  uint8_aes128_iv      [aes cbc initialization vector]
 */
void mbed_aes128(uint8_t* Mbedtls_aes128_input, uint8_t* uint8_aes128_key, uint8_t* uint8_aes128_iv){
								custom_print((char*) "TEST OF MBEDTLS AES-128 (CBC mode) API");
								mbedtls_aes_context aes;

								uint8_t temp_iv[16] = {0};
								shifted_buffer1_to_buffer2(uint8_aes128_iv, temp_iv, 0, 16, 0, 16);
								const unsigned char* input = (const unsigned char*) Mbedtls_aes128_input;
								const unsigned char* mbed_key = (const unsigned char*) uint8_aes128_key;
								unsigned char* iv = (unsigned char*) temp_iv;
								unsigned char output[32] = {0};

								// Set key and encrypt
								mbedtls_aes_setkey_enc( &aes, mbed_key, 16*8 );
								mbedtls_aes_crypt_cbc(  &aes, MBEDTLS_AES_ENCRYPT, sizeof(input)*8, iv, input, output );

								// Casting and padding into the uint8_t returned buffer
								uint8_t* Mbedtls_aes128_output = (uint8_t*) output;

								// Prints
								uint8_print((char*)"Mbedtls-aes128 key   :", uint8_aes128_key,  16);
								uint8_print((char*)"Mbedtls-aes128 iv    :", uint8_aes128_iv, 16);
								uint8_print((char*)"Mbedtls-aes128 input :", Mbedtls_aes128_input,  32);
								uint8_print((char*)"Mbedtls-aes128 output:", Mbedtls_aes128_output, 32);
}

/**
 * [Call mbedtls sha-256]
 * @method mbed_sha256
 * @param  Mbedtls_sha256_input [hashing function input data]
 */
void mbed_sha256(uint8_t *Mbedtls_sha256_input){
								custom_print((char*) "TEST OF MBEDTLS SHA-256 API");
								uint8_t Mbedtls_sha256_output[32];
								const unsigned char* sha_input = (const unsigned char*) Mbedtls_sha256_input;
								unsigned char sha_output[32] = {0};

								// Hashing the encrypted data using the mbedtls_sha256
								size_t input_length = sizeof(sha_input)*8;
								mbedtls_sha256(sha_input, input_length, sha_output, 0);

								// Casting and padding into the uint8_t returned buffer
								uint8_t* temp_hashed = (uint8_t*) sha_output;
								shifted_buffer1_to_buffer2(temp_hashed, Mbedtls_sha256_output, 0, 32, 0, 32);

								// Prints
								uint8_print((char*)"Mbedtls-sha256 input :", Mbedtls_sha256_input,  32);
								uint8_print((char*)"Mbedtls-sha256 output:", Mbedtls_sha256_output, 32);
}

/**
 * [Call contik aes-128 ecb mode]
 * @method contiki_aes128
 * @param  plain_data     [plain input data]
 * @param  aes_key        [aes key]
 */
void contiki_aes128(uint8_t* plain_data, uint8_t* aes_key){
								custom_print((char*) "TEST OF CONTIKI AES-128 (ECB mode) API");
								int block_size = 16;

								// Set key
								uint8_t ecb_key[16];
								uint8_t aes_output[32];
								uint8_t temporary_buffer[block_size];
								shifted_buffer1_to_buffer2(aes_key, ecb_key, 0, 16, 0, 16);
								AES_128.set_key(ecb_key);

								// Encrypt data block by block (32 bits = 2 blocks) using ECB mode
								for (int i = 0; i < 32; i+= block_size) {
																shifted_buffer1_to_buffer2(plain_data, temporary_buffer, i, i+16, 0, 16);
																AES_128.encrypt(temporary_buffer);
																shifted_buffer1_to_buffer2(temporary_buffer, aes_output, 0, 16, i, i+16);
								}
								uint8_print((char*)"Contiki aes-128 key   :", aes_key,  16);
								uint8_print((char*)"Contiki aes-128 input :", plain_data,  32);
								uint8_print((char*)"Contiki aes-128 output:", aes_output, 32);
}
/*
 * All Contiki programs must have a process, and we declare it here.
 */
PROCESS(test_process, "Test process");

/*
 * Here we implement the process. The process is run whenever an event
 * occurs, and the parameters "ev" and "data" will we set to the event
 * type and any data that may be passed along with the event.
 */
PROCESS_THREAD(test_process, ev, data)
{
								PROCESS_BEGIN();
								custom_print("PROCESS STARTED");

								// Define variables
								static uint8_t plain_data[32] = {0x49, 0x74, 0x20, 0x69, 0x73, 0x20, 0x61, 0x6e, 0x20, 0x69, 0x6d, 0x70, 0x6c, 0x65, 0x6d, 0x65,
									 															 0x6e, 0x74, 0x61, 0x74, 0x69, 0x6f, 0x6e, 0x20, 0x65, 0x78, 0x61, 0x6d, 0x70, 0x6c, 0x65, 0x21};

								static uint8_t aes_key[16]    = {0x21, 0x54, 0x68, 0x69, 0x73, 0x2d, 0x49, 0x73, 0x2d, 0x41, 0x6e, 0x2d, 0x4b, 0x65, 0x79, 0x21};
								static uint8_t uint8_iv[16]   = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f};

								// TI aes128 & sha256
								ti_aes(plain_data, aes_key);
								ti_sha256(plain_data);

								// MbedTls aes128 & sha256
								mbed_aes128(plain_data, aes_key, uint8_iv);
								mbed_sha256(plain_data);

								// Contiki aes128
								contiki_aes128(plain_data, aes_key);

								custom_print("PROCESS ENDED");
								PROCESS_END();
}

AUTOSTART_PROCESSES(&test_process);
