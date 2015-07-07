#include "DataLogger.h"

DataLogger::DataLogger(void)
{
  // initialize counters, pointers, etc
  _total_memory_bytes = 0;
  _init = false;
  _write_enabled = false;
  _pdebug = 0;
}

DataLogger::DataLogger(FastSerial* pdebug)
{
  // initialize counters, pointers, etc
  _total_memory_bytes = 0;
  _init = false;
  _write_enabled = false;
  _pdebug = pdebug;
}

DataLogger::~DataLogger(void)
{
  // nothing to clean up
}

void DataLogger::erase()
{
  if (_write_enabled && _init)
    {
      DataFlash.EraseAll(&delay);
    }
}

void DataLogger::enable_write()
{
  _write_enabled = true;
}

void DataLogger::disable_write()
{
  _write_enabled = false;
}

bool DataLogger::is_writeable()
{
  return _write_enabled;
}


void DataLogger::register_block(const char* name, int32_t len, int32_t precision)
{
  int32_t i;
  int32_t slen;
  int32_t buflen;

  if (_write_enabled && _init) // do nothing if we aren't authorized to write
    {
      // data to write
      buflen = len*4;
      slen = strlen(name);

      // write file header
      DataFlash.StartWrite(1);
      DataFlash.WriteByte(FILE_HEADER_BYTE1);
      DataFlash.WriteByte(FILE_HEADER_BYTE2);

      DEBUG("Going to write this precision: ");
      DEBUG(precision);

      // write format data
      DataFlash.WriteByte(BLOCK_FORMAT_HEAD_BYTE1); // block type packet header
      DataFlash.WriteByte(BLOCK_FORMAT_HEAD_BYTE2); // block type packet header
      DataFlash.WriteLong(len);                     // Num of elements (size of mux)
      DataFlash.WriteLong(buflen);                  // Total length of block write
      DataFlash.WriteLong(precision);               // Requested precision of text output
      DataFlash.WriteLong(slen);                    // Size of column label string
      for (i=0;i<slen;i++)
	DataFlash.WriteByte(name[i]);               // Write the label string. 
                                           // Note the lack of terminator 
                                           // here. Comma sep list

      DataFlash.WriteByte(BLOCK_FORMAT_TAIL_BYTE);

      // force a write to flash (I am assuming that's what this function does)
      //DataFlash.FinishWrite();
    }
}

void DataLogger::write_float_array(float* data, int32_t len)
{
  int32_t i;
  int32_t idata;

  DEBUG("In write_float_array");  
  if (_write_enabled && _init)
    {
      DEBUG("Actually writing this many bytes: ");
      DEBUG(len);

      DataFlash.WriteByte(HEAD_BYTE1);
      DataFlash.WriteByte(HEAD_BYTE2);


      for (i=0;i<len;i++)
	{
	  DataFlash.WriteFloat(data[i]);
	  //idata = (int32_t)data;
	  //DataFlash.WriteLong(i);
	}

      DataFlash.WriteByte(TAIL_BYTE);
      
      // DataFlash.FinishWrite();
    }
}

void DataLogger::init()
{
  DataFlash.Init();
  _total_memory_bytes = (uint32_t)DataFlash.df_PageSize * TOTAL_PAGES;
  _init = true;
}

bool DataLogger::check_for_data()
{
  unsigned char b1;
  unsigned char b2;
  
  DataFlash.StartRead(1);
  b1 = DataFlash.ReadByte();
  b2 = DataFlash.ReadByte();

  if ((b1==FILE_HEADER_BYTE1) && (b2==FILE_HEADER_BYTE2))
    {
      return true;
    }

  return false;
}

// WARNING: This may lead to corrupt data sets! Use only for debugging or if sure
// that the new dataset will be larger than the old one!
void DataLogger::clear_file_header()
{
  DataFlash.StartWrite(1);
  DataFlash.WriteByte(0x00);
  DataFlash.WriteByte(0x00);
}

void DataLogger::dump_to_serial(FastSerial* ps)
{
  unsigned char h1, h2, h3;
  int32_t i, j;
  int32_t maxreads;
  int32_t len;
  int32_t slen;
  uint8_t* colstr;
  float* buffer;
  //float buffer[3];

  int32_t buflen;
  int32_t precision;

  bool packetvalid;


  /* Read file header */
  DataFlash.StartRead(1);
  h1 = DataFlash.ReadByte(); // read and discard file header bytes
  h2 = DataFlash.ReadByte(); // read and discard file header bytes

  /* Read format packet */
  h1         = DataFlash.ReadByte(); // block type packet header
  h2         = DataFlash.ReadByte(); // block type packet header
  len        = DataFlash.ReadLong(); // Num of elements (size of mux)
  buflen     = DataFlash.ReadLong(); // Total length of block write
  precision  = DataFlash.ReadLong(); // Requested precision of text output
  slen       = DataFlash.ReadLong(); // Length of column name string

  DEBUG(h1);
  DEBUG(h2);
  DEBUG(len);
  DEBUG(buflen);
  DEBUG(precision);
  DEBUG(slen);
  
  /* Allocate memory to hold data vector */
  buffer = new float[len];

  // Allocate memory to hold column string and read it
  colstr = new uint8_t[slen+1];
  for (i=0;i<slen;i++)
    colstr[i] = DataFlash.ReadByte();
  colstr[slen] = '\0'; // add terminator
  
  h3 = DataFlash.ReadByte();

  if (h3 != BLOCK_FORMAT_TAIL_BYTE)
    {
      // aww crap
      DEBUG("aww, snap");
      delete [] colstr;
      delete [] buffer;
      return;
    }

  // print column names
  ps->println((const char*)colstr);
  delete [] colstr;

  // search for data blocks
  maxreads = (buflen+3)*2; // buffer plus head/tails bytes times two for good measure
  while (_move_to_next_packet(HEAD_BYTE1,HEAD_BYTE2,maxreads))
    {
      // found a packet and are already past the header, read it
     
      for (i=0;i<len;i++)
	{
	  buffer[i] = DataFlash.ReadFloat();
	  DEBUG(buffer[i]);
	}

      // read footer
      h3 = DataFlash.ReadByte();
      if (h3 == TAIL_BYTE)
	{
	  _print_array(ps, buffer, len, precision);
	}
      else
	{
	  DEBUG("bad tails");
	}
    }

    delete [] buffer;
}

bool DataLogger::_move_to_next_packet(uint8_t byte1, uint8_t byte2, int32_t maxreadbytes)
{
  int32_t idx=0;
  uint8_t h1,h2;
  DEBUG("Entering move to next packet");

  while (idx<maxreadbytes)
    {
      h1 = DataFlash.ReadByte();
      idx++;
      if (h1 == byte1)
	{
	  h2 = DataFlash.ReadByte();
	  idx++;
	  if (h2 == byte2)
	    {
	      // found it
	      DEBUG("Found packet.");
	      return true;
	    }
	}
    }

  DEBUG("Did not find packet.");
  return false;
}
	    
      




void DataLogger::_print_array(FastSerial* ps,     \
			      float* buffer,    \
			      int32_t len,        \
			      int32_t precision)
{
  uint8_t i;
  
  ps->print(buffer[0],precision);  
  for (i=1;i<len;i++)
    {
      ps->print(DELIMITER);
      ps->print(buffer[i],precision);
    }
  ps->println(" ");
}



