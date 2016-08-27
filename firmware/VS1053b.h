/*
 * vs1053b.h
 *
 *  Created on: Jul 25, 2016
 *      Author: aaron
 */

#ifndef LIBRARIES_VS1053B_H_
#define LIBRARIES_VS1053B_H_

#ifdef __cplusplus

#include "application.h"
#include "FatFs/FatFs.h"
#include "FatFs/trampoline.h"
#include <CException/CException.h>

#ifndef DEBUGLOG
#define DEBUGLOG(x, y, ...)
#endif

//VS10xx SCI Registers
#define SCI_MODE 0x00
#define SCI_STATUS 0x01
#define SCI_BASS 0x02
#define SCI_CLOCKF 0x03
#define SCI_DECODE_TIME 0x04
#define SCI_AUDATA 0x05
#define SCI_WRAM 0x06
#define SCI_WRAMADDR 0x07
#define SCI_HDAT0 0x08
#define SCI_HDAT1 0x09
#define SCI_AIADDR 0x0A
#define SCI_VOL 0x0B
#define SCI_AICTRL0 0x0C
#define SCI_AICTRL1 0x0D
#define SCI_AICTRL2 0x0E
#define SCI_AICTRL3 0x0F

extern const unsigned short vs1053b_patches[];
extern const size_t vs1053b_patch_len;

template<class T>
void vPlayAudioTask(void* arg) {
	T* VS1053b = (T*)arg;
	for(;;)
		VS1053b->playInternal();
	DEBUGLOG(TRACE, "AUDIO: player thread ending");
	END_THREAD();
}

template <typename PinType>
class VS1053b {
private:
	FIL _file;
	uint8_t _buffer[512];
	SPIClass* _spi;
	std::mutex* _spiMutex;
	uint16_t _dreq;
	uint16_t _ncs;
	uint16_t _ndcs;
	PinType _nrst;
	uint32_t _clock;

	bool _started;
	volatile bool _stopRequested;
	String _filename;

	std::mutex playerMutex;
	os_queue_t _dreqState;
	os_thread_t _playerThread;
	os_queue_t _playTrigger;

	void beginSPI() {
		if(_spiMutex)
			_spiMutex->lock();

		_spi->begin(SPI_MODE_MASTER, _ncs);
		_spi->setClockSpeed(_clock, HZ);
		_spi->setBitOrder(MSBFIRST);
		_spi->setDataMode(SPI_MODE0);

	}

	void endSPI() {
		if(_spiMutex)
			_spiMutex->unlock();
	}
	void loadCode(const unsigned short * plugin, size_t len)  {
	  unsigned int i = 0;

	  DEBUGLOG(DebugLevel_Trace, "Loading VS1053 Patches");
	  while (i<len/sizeof(plugin[0])) {
		unsigned short addr, n, val;
		addr = plugin[i++];
		n = plugin[i++];
		if (n & 0x8000U) { /* RLE run, replicate n samples */
		  n &= 0x7FFF;
		  val = plugin[i++];
		  while (n--) {
			writeRegister(addr, val);
		  }
		} else {           /* Copy run, copy n samples */
		  while (n--) {
			val = plugin[i++];
			writeRegister(addr, val);
		  }
		}
	  }

	  DEBUGLOG(DebugLevel_Trace, "VS1053 Patches loaded");
	}

	void writeRegister(unsigned char addressbyte, unsigned char highbyte, unsigned char lowbyte) {
	  waitBusy(); //Wait for DREQ to go high indicating IC is available
	  beginSPI();
	  {
		  digitalWrite(_ncs, LOW); //Select control
		  //SCI consists of instruction byte, address byte, and 16-bit data word.
		  _spi->transfer(0x02); //Write instruction
		  _spi->transfer(addressbyte);
		  _spi->transfer(highbyte);
		  _spi->transfer(lowbyte);
		  waitBusy(); //Wait for DREQ to go high indicating command is complete
		  digitalWrite(_ncs, HIGH); //Deselect Control
	  }
	  endSPI();
	}

	void writeRegister(unsigned char addressbyte, const unsigned short word) {
		uint8_t highbyte = word >> 8;
		uint8_t lowbyte = word & 0x00ff;
		writeRegister(addressbyte, highbyte, lowbyte);
	}

	unsigned int readRegister (unsigned char addressbyte) {
	  waitBusy(); //Wait for DREQ to go high indicating IC is available
	  char response1, response2;
	  beginSPI();
	  {
		  digitalWrite(_ncs, LOW); //Select control

		  //SCI consists of instruction byte, address byte, and 16-bit data word.
		  _spi->transfer(0x03);  //Read instruction
		  _spi->transfer(addressbyte);

		  response1 = _spi->transfer(0xFF); //Read the first byte
		  waitBusy(); //Wait for DREQ to go high indicating command is complete
		  response2 = _spi->transfer(0xFF); //Read the second byte
		  waitBusy(); //Wait for DREQ to go high indicating command is complete

		  digitalWrite(_ncs, HIGH); //Deselect Control
	  }
	  endSPI();

	  int resultvalue = response1 << 8;
	  resultvalue |= response2;
	  return resultvalue;
	}

	void waitBusy() {

#if PLATFORM_THREADING && defined(SYSTEM_VERSION_060RC1)
		bool result;
		while(busy())
			os_queue_take(_dreqState, (void*)&result, 1000, nullptr);
#endif

		while(busy());
	}

	bool busy() {
		return digitalRead(_dreq) == LOW;
	}

	int streamBufferFillWords(void) {
	  uint16_t wrp, rdp;
	  int16_t res;
	  /* For FLAC files, stream buffer is larger */
	  int16_t bufSize = (readRegister(SCI_HDAT1) == 0x664C) ? 0x1800 : 0x400;
	  writeRegister(SCI_WRAMADDR, 0x5A7D);
	  wrp = readRegister(SCI_WRAM);
	  rdp = readRegister(SCI_WRAM);
	  res = wrp-rdp;
	  if (res < 0) {
		return res + bufSize;
	  }
	  return res;
	}

	int streamBufferFreeWords(void) {
	  /* For FLAC files, stream buffer is larger */
	  int16_t bufSize = (readRegister(SCI_HDAT1) == 0x664C) ? 0x1800 : 0x400;
	  int16_t res = bufSize - streamBufferFillWords();
	  if (res < 2) {
		return 0;
	  }
	  return res-2;
	}

	int audioBufferFillWords(void)  {
	  uint16_t wrp, rdp;
	  writeRegister(SCI_WRAMADDR, 0x5A80);
	  wrp = readRegister(SCI_WRAM);
	  rdp = readRegister(SCI_WRAM);
	  return (wrp-rdp) & 4095;
	}

	int audioBufferFreeWords(void) {
	  int16_t res = 4096 - audioBufferFillWords();
	  if (res < 2) {
		return 0;
	  }
	  return res-2;
	}

	uint16_t audioBufferUnderflow(void) {
	  uint16_t uFlow;
	  writeRegister(SCI_WRAMADDR, 0x5A82);
	  uFlow = readRegister(SCI_WRAM);
	  if (uFlow) {
		writeRegister(SCI_WRAMADDR, 0x5A82);
		writeRegister(SCI_WRAM, 0, 0); /* Clear */
	  }
	  return uFlow;
	}

	void playInternal() {

		//pump semaphore
		bool shouldPlay;
		os_queue_take(_playTrigger, &shouldPlay, CONCURRENT_WAIT_FOREVER, nullptr);

		ATOMIC_BLOCK()
		{
			shouldPlay = _filename.length() > 0;
		}

		if(shouldPlay)
		{
			//open file
			FRESULT result = f_open(&_file, _filename.c_str(), FA_READ | FA_OPEN_EXISTING);
			if (result != FR_OK) { //Open the file in read mode.
				DEBUGLOG(DebugLevel_Error, "AUDIO: Failed to open %s (%d)", _filename.c_str(), result);
			}
			else
			{
				DEBUGLOG(DebugLevel_Trace, "AUDIO: Track open: %s", _filename.c_str());

				UINT bytes_read = 0;
				UINT total_bytes_read = 0;

				while(1) {
					if(_stopRequested)
					{
						DEBUGLOG(DebugLevel_Verbose, "AUDIO: stop requested");
						break;
					}
					UINT available = streamBufferFreeWords() * 2;

					if(available >= sizeof(_buffer) / 2) {
						UINT numberOfBytesToRead = available;
						if(numberOfBytesToRead > sizeof(_buffer))
							numberOfBytesToRead = sizeof(_buffer);
						result = f_read(&_file, _buffer, numberOfBytesToRead, &bytes_read);
						total_bytes_read += bytes_read;
						DEBUGLOG(DebugLevel_VVVDeepTrace, "AUDIO: %d available buffer, %d tried to read, %d read (total %d)", available, numberOfBytesToRead, bytes_read, total_bytes_read);
						if(result != FR_OK || bytes_read == 0) {
							DEBUGLOG(DebugLevel_Trace, "AUDIO: read finished with code %d", result);
							break;
						}

						waitBusy();

						//save SCI clock speed
						unsigned clock = _clock;
						//should allow up to CLKI/4, so at 5x, 15.28 MHz
						_clock = 10000000;
						beginSPI();
						{
							digitalWrite(_ndcs, LOW); //Select Data
							__ff_spi_send_dma(*_spi, _buffer, bytes_read);
							waitBusy();
							digitalWrite(_ndcs, HIGH); //Deselect Data
							//restore clock
							_clock = clock; //8000000;
						}
						endSPI();
					}
					delay(1);
				}

				f_close(&_file);
			}

			DEBUGLOG(DebugLevel_Verbose, "AUDIO: finished playing");

			ATOMIC_BLOCK()
			{
				_stopRequested = false;
				_filename = "";
			}
		}
	}

	friend void vPlayAudioTask<VS1053b<PinType>>(void* arg);

	void dreqChange()
	{
#if PLATFORM_THREADING && (defined(SYSTEM_VERSION_060RC1) || defined(SYSTEM_VERSION_060))
		if(!busy()) {
			os_queue_put(_dreqState, (void*)true, 0, nullptr);
		}
#endif
	}

	void startup() {
#if PLATFORM_THREADING && (defined(SYSTEM_VERSION_060RC1) || defined(SYSTEM_VERSION_060))
		os_queue_create(&_dreqState, sizeof(HIGH), 1, nullptr);
		os_queue_create(&_playTrigger, sizeof(true), 1, nullptr);
#endif


	  uint8_t state = LOW;

	  pinMode(_dreq, INPUT);
	  pinMode(_ncs, OUTPUT);
	  pinMode(_ndcs, OUTPUT);
	  pinMode(_nrst, OUTPUT);

	  digitalWrite(_ncs, HIGH); //Deselect Control
	  digitalWrite(_ndcs, HIGH); //Deselect Data
	  digitalWrite(_nrst, LOW); //Put VS1053 into hardware reset

	  DEBUGLOG(DebugLevel_Verbose, "Set up Vs1053b");

	  delay(100);

	  digitalWrite(*_nrst, HIGH); //Bring up VS1053

	  int n;
	  for(n = 0; n < 10 && state == LOW; n++)
	  {
		  state = digitalRead(_dreq);
		  DEBUGLOG(DebugLevel_Trace, "DREQ: %d", state);
		  delay(100);
	  }

	  if(n >= 10)
		  LOG(ERROR, "VS1053b initialization error - DREQ never went high");

	  attachInterrupt(_dreq, &VS1053b<PinType>::dreqChange, this, CHANGE, 0, 0);

	  setVolume(40, 40);

	  state = digitalRead(_dreq);
	  DEBUGLOG(DebugLevel_Trace, "DREQ: %d", state);

	  //Let's check the status of the VS1053
	  int MP3Mode = readRegister(SCI_MODE);
	  int MP3Status = readRegister(SCI_STATUS);
	  int MP3Clock = readRegister(SCI_CLOCKF);

	  DEBUGLOG(DebugLevel_Trace, "SCI_Mode (0x4800) = 0x%4x", MP3Mode);
	  DEBUGLOG(DebugLevel_Trace, "SCI_Status (0x48) = 0x%4x", MP3Status);
	  int vsVersion = (MP3Status >> 4) & 0x000F; //Mask out only the four version bits
	  DEBUGLOG(DebugLevel_Trace, "VS Version (VS1053 is 4) = %d", vsVersion); //The 1053B should respond with 4. VS1001 = 0, VS1011 = 1, VS1002 = 2, VS1003 = 3
	  DEBUGLOG(DebugLevel_Trace, "SCI_ClockF = 0x%4x", MP3Clock);

	  //Now that we have the VS1053 up and running, increase the internal clock multiplier and up our SPI rate
	  writeRegister(SCI_CLOCKF, 0xe0, 0x00); //Set multiplier to 3.0x (0x60) or 5.0x (0xe0)

	  //From page 12 of datasheet, max SCI reads are CLKI/7. Input clock is 12.288MHz.
	  //Internal clock multiplier is now 3x: 36.684 MHz
	  //Therefore, max SPI speed is 5MHz. 4MHz will be safe.
	  //Internal clock multiplier is now 5x: 61.14 MHz
	  //Therefore, max SPI speed is 8.7 MHz. 8 MHz will be safe.
	  _clock = 800000;

	  MP3Clock = readRegister(SCI_CLOCKF);
	  DEBUGLOG(DebugLevel_Trace, "SCI_ClockF = 0x%2x", MP3Clock);


	  //load the patches plugin for VS1053b
	  loadCode(vs1053b_patches, vs1053b_patch_len);
	  waitBusy();


	  DEBUGLOG(DebugLevel_DeepTrace, "AUDIO: launching player thread");
	  os_thread_create(nullptr, "Audio", 3, vPlayAudioTask<VS1053b<PinType>>, this, 2048);

	  _started = true;
	}
public:
	VS1053b() :
		_spi(nullptr),
		_spiMutex(nullptr),
		_dreq(0),
		_ncs(0),
		_ndcs(0),
		_clock(1000000),
		_started(false),
		_stopRequested(false),
		_filename(""),
		_dreqState(nullptr),
		_playerThread(nullptr),
		_playTrigger(nullptr)
	{}

	void begin(SPIClass& spi, const uint16_t dreq, const uint16_t ncs, const uint16_t ndcs, PinType nrst, std::mutex& mutex) {
		_spi = &spi;
		_dreq = dreq;
		_ncs = ncs;
		_ndcs = ndcs;
		_nrst = nrst;
		_spiMutex = &mutex;

		startup();
	}
	void setVolume(unsigned char leftchannel, unsigned char rightchannel) {
	  writeRegister(SCI_VOL, leftchannel, rightchannel);
	}

	void setVolume(uint8_t level) {
		level = map(255-level, 0, 255, 0, 100);
		setVolume(level, level);
	}

	bool started() { return _started; };
	bool play(const char* filename, size_t bufferSize = 512)  {
		if(started())
		{
			//stop play if in progress
			for(bool myStart = false; !myStart;) {
				ATOMIC_BLOCK() {
					if(_filename == nullptr) {
						_filename = filename;
						myStart = true;
					} else {
						if(!_stopRequested)
							stop();
					}
				}

				if(!myStart)
					delay(1);
			}

			os_queue_put(_playTrigger, (void*)false, CONCURRENT_WAIT_FOREVER, nullptr);
			return true;
		}
		else
			return false;
	}
	void stop() { _stopRequested = true; }

};

#endif

#endif /* LIBRARIES_VS1053B_H_ */
