#ifndef _DATALOGGER_H
#define _DATALOGGER_H

#include "Arduino.h"
#include "DataFlash.h"
#include "FastSerial.h"
#include <string.h>

/* Magic numbers */
#define FILE_HEADER_BYTE1       0x30  // File header byte 1
#define FILE_HEADER_BYTE2       0x29  // File header byte 2
#define HEAD_BYTE1              0x46  // Data packet byte 1
#define HEAD_BYTE2              0x57  // Data packet byte 2
#define TAIL_BYTE               0x56  // Data packet tail byte
#define ID_BYTE_START           0x8A  // Base data packet ID
#define BLOCK_FORMAT_HEAD_BYTE1 0xF1  // Block format packet byte 1
#define BLOCK_FORMAT_HEAD_BYTE2 0xF2  // Block format packet byte 2
#define BLOCK_FORMAT_TAIL_BYTE  0xF3  // Block format packet tail byte

/* Option bits for data recovery */
#define OPT_DUMP_TO_TEXT       0x10  // Request text data
#define OPT_DUMP_TO_BINARY     0x20  // Request binary data

/* Additional information */
#define MAX_VECTOR_LEN   100    // max vector length
#define DELIMITER       ','     // delimiter for text data when requested
#define TOTAL_PAGES     8192    // total number of memory pages
#define MAX_PACKET_SIZE 1024    // only used to stop the data dump routine from
                                // searching all 4 mb, i.e. stopping after it
                                // has read MAX_PACKET_SIZE*MAX_BLOCKS bytes and 
                                // did not find what it was looking for.

/* #define DATALOGGER_DEBUG */
#ifdef DATALOGGER_DEBUG
#define DEBUG(x) (_pdebug->println(x))
#else
#define DEBUG(x) ((void) 0)
#endif

class DataLogger
{
 public:
  DataLogger();
  DataLogger(FastSerial* pdebug);
  ~DataLogger();
  void register_block(const char* name, int32_t len, int32_t precision);
  void write_float_array(float* data, int32_t len);
  bool check_for_data();
  bool is_writeable();
  void enable_write();
  void disable_write();
  void erase();
  void dump_to_serial(FastSerial* ps);
  void init();
  void clear_file_header();

 private:
  DataFlash_APM2  DataFlash;



  bool _move_to_next_packet(uint8_t byte1, uint8_t byte2, int32_t maxreads);
  void _print_array(FastSerial* ps,    \
		    float* buffer,   \
		    int32_t len,       \
		    int32_t precision);

  bool _write_enabled;
  bool _init;
  uint32_t _total_memory_bytes;
  FastSerial* _pdebug;

};

#endif // DATALOGGER_H
