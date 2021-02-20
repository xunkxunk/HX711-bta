#ifndef HX711_MULTI_h
#define HX711_MULTI_h

#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

class HX711BTA
{
	private:
		byte PD_SCK;	// Power Down and Serial Clock Input Pin
		byte COUNT;		// The number of channels to read
		byte *DOUT;		// Serial Data Output Pin
		byte GAIN;		// amplification factor
		
		bool OK[8];
		long miniA[8];
		long maxiA[8];
		long miniB[8];
		long maxiB[8];
		long TOLERANCE_A;
		long TOLERANCE_B;
		int MOY ;
		int ITERATION ;
		
		bool debugEnabled; //print debug messages?

		long *OFFSETS;	// used for tare weight
		float SCALE;	// used to return weight in grams, kg, ounces, whatever
		void calcul(long *results = NULL, int coefError = 1000, int coefCpt = 7, int coefMoy = 30);
		
	public:
		// define clock and data pin, channel, and gain factor
		// channel selection is made by passing the appropriate gain: 128 or 64 for channel A, 32 for channel B
		// count: the number of channels
		// dout: an array of pin numbers, of length 'count', one entry per channel
		HX711BTA(int count, byte *dout, byte pd_sck, byte gain = 128, byte PIN_RELAIS = 7);

		virtual ~HX711BTA();

		//returns the number of channels
		byte get_count();

		// check if HX711 is ready
		// from the datasheet: When output data is not ready for retrieval, digital output pin DOUT is high. Serial clock
		// input PD_SCK should be low. When DOUT goes to low, it indicates data is ready for retrieval.
		bool wait_ready_timeout(unsigned long timeout, unsigned long delay_ms);
		bool is_ready();

		// set the gain factor; takes effect only after a call to read()
		// channel A can be set for a 128 or 64 gain; channel B has a fixed 32 gain
		// depending on the parameter, the channel is also set to either A or B
		void serialOk();
		void setMiniA(long val, int pos = 0);
		void setMaxiA(long val, int pos = 0);
		void setMiniB(long val, int pos = 0);
		void setMaxiB(long val, int pos = 0);
		long getMiniA(int pos = 0);
		long getMiniB(int pos = 0);
		long getMaxiB(int pos = 0);
		long getMaxiA(int pos = 0);
		void setToleranceA(long tolerance = 3000);
		void setToleranceB(long tolerance = 1000);
		void setMoyenne(int moy = 30);
		void setIteration(int iteration = 5);
		
		void printOk(byte gain = 128) ;
		void defineOk();
		void set_gain(byte gain = 128);

		void set_dout(int count, byte *dout);

		// waits for the chip to be ready and returns a reading
		void read(long *result = NULL);

		// same as read, but does not offset the values according to the tare
		void readRaw(long *result = NULL);

		// set the OFFSET value for tare weight
		// times: how many times to read the tare value
		// returns true iff the offsets have been reset for the scale during this call.
		// tolerance: the maximum deviation of samples, above which to reject the attempt to tare. (if set to 0, ignored)
		bool tare(byte times = 10, uint16_t tolerance = 0);
		
		void sendRawDataB(long *poidsB = NULL, int count = NULL, byte *dout = NULL);
		void sendRawData(long *poidsA = NULL, int count = NULL, byte *dout = NULL);
		void ab(long *resultat = NULL, int count = NULL, byte *dout = NULL, String channel = "");
		void etalonnage();
		// puts the chip into power down mode
		void power_down();

		// wakes up the chip after power down mode
		void power_up();

		void setDebugEnable(bool debugEnable = true);
		
		void reboot();
		
		String lire();
};

#endif /* HX711_MULTI_h */
