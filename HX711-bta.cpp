#include <Arduino.h>
#include <HX711-bta.h>

HX711BTA::HX711BTA(int count, byte *dout, byte pd_sck, byte gain, byte PIN_RELAIS) {
	PD_SCK 	= pd_sck;
	DOUT 	= dout; //TODO - make the input of dout to the function a const, or otherwise copy the values for local storage
	COUNT   = count;

	debugEnabled = false;

	pinMode(PD_SCK, OUTPUT);
	pinMode(PIN_RELAIS,OUTPUT);
	digitalWrite(PIN_RELAIS,HIGH);
   
   
	for (int i=0; i<count; i++) {
		pinMode(DOUT[i], INPUT);
	}
	//set_gain(gain);
	printOk(gain);

	OFFSETS = (long *) malloc(COUNT*sizeof(long));

	for (int i=0; i<COUNT; ++i) {
		OFFSETS[i] = 0;
	}
}

void HX711BTA::serialOk() {
	Serial.print("OK : ");
	for (int i= 0 ; i < COUNT ; ++i) 
	{
	Serial.print(OK[i]);
	Serial.print(" ");
}
}


void HX711BTA::setMiniA(long val,int pos) {
	miniA[pos] = val ;
}

void HX711BTA::setMiniB(long val,int pos) {
	miniB[pos] = val ;
}

void HX711BTA::setMaxiA(long val,int pos) {
	maxiA[pos] = val ;
}

void HX711BTA::setMaxiB(long val,int pos) {
	maxiB[pos] = val ;
}

long HX711BTA::getMaxiB(int pos) {
	return maxiB[pos] ;
}

long HX711BTA::getMiniB(int pos) {
	return miniB[pos] ;
}

long HX711BTA::getMiniA(int pos) {
	return miniA[pos] ;
}

long HX711BTA::getMaxiA(int pos) {
	return maxiA[pos] ;
}


void HX711BTA::defineOk() {
	
	digitalWrite(PD_SCK, LOW);
	
	for (int i = 0; i<COUNT; ++i) {
		OK[i] = false; 
		unsigned long millisStarted = millis();
		while (millis() - millisStarted < 2000) {
			if (digitalRead(DOUT[i]) == LOW) {
				OK[i] = true ; break ; 
			}
			delay(50);
		}
	}
	
}


void HX711BTA::printOk(byte gain) {
	
	switch (gain) {
		case 128:		// channel A, gain factor 128
			GAIN = 1;
			break;
		case 64:		// channel A, gain factor 64
			GAIN = 3;
			break;
		case 32:		// channel B, gain factor 32
			GAIN = 2;
			break;
	}
	
	defineOk();
	
	
	read();
	
}

void HX711BTA::set_dout(int count, byte *dout) {
	
	DOUT = dout ; 
	COUNT = count ; 
	
}

HX711BTA::~HX711BTA() {
	free(OFFSETS);
}

bool HX711BTA::wait_ready_timeout(unsigned long timeout, unsigned long delay_ms) {
	// Wait for the chip to become ready until timeout.
	// https://github.com/bogde/HX711/pull/96
	unsigned long millisStarted = millis();
	while (millis() - millisStarted < timeout) {
		if (is_ready()) {
			return true;
		}
		delay(delay_ms);
	}
	return false;
}

bool HX711BTA::is_ready() { 
	bool bresult = true;
	for (int i = 0; i<COUNT; ++i) {
		if (OK[i]) {
			if (digitalRead(DOUT[i]) == HIGH) {
				bresult = false;
			}
		}
	}
	return bresult;
}

void HX711BTA::set_gain(byte gain) {

	switch (gain) {
		case 128:		// channel A, gain factor 128
			GAIN = 1;
			break;
		case 64:		// channel A, gain factor 64
			GAIN = 3;
			break;
		case 32:		// channel B, gain factor 32
			GAIN = 2;
			break;
	}

	digitalWrite(PD_SCK, LOW);
	read(); //a read is needed to get gain setting to come into effect. (for the next read)
}

byte HX711BTA::get_count() {
	return COUNT;
}

//TODO: write a function / state variable that 'learns' (stores/tracks) the expected noise figure from the cells, and automatically selects a reasonable 'tolerance' for tare.
//		i.e. a 'best recently seen stability'. Keep it up-to-date automatically by updating it with every read. (reads will probably need to be time-aware)

bool HX711BTA::tare(byte times, uint16_t tolerance) {
	//TODO: change this implementation to use a smarter read strategy. 
	//		right now samples are read 'times' times, but only the last is used (the multiple samples)
	//		are just being used to verify the spread is < tolerance.
	//		
	//		This should be done by creating a smarter multiple-reads function which returns a struct with values and metadata (number of good samples, standard deviation, etc.) 
	int i,j;

	long values[COUNT];

	long minValues[COUNT];
	long maxValues[COUNT];

	for (i=0; i<COUNT; ++i) {
		minValues[i]=0x7FFFFFFF;
		maxValues[i]=0x80000000;

		//OFFSETS[i]=0; //<--removed this line, so that a failed tare does not undo previous tare
	}

	for (i=0; i<times; ++i) {
		readRaw(values);
		for (j=0; j<COUNT; ++j) {
			if (values[j]<minValues[j]) {
				minValues[j]=values[j];
			}	
			if (values[j]>maxValues[j]) {
				maxValues[j]=values[j];
			} 
		}		
	}

	if (tolerance!=0 && times>1) {
		for (i=0; i<COUNT; ++i) {
			if (abs(maxValues[i]-minValues[i])>tolerance) {
				//one of the cells fluctuated more than the allowed tolerance, reject tare attempt;
				if (debugEnabled) {
					Serial.print("Rejecting tare: (");
					Serial.print(i);
					Serial.print(") ");
					Serial.println(abs(maxValues[i]-minValues[i]));
				}
				return false;
			}
		}
	}

	//set the offsets
	for (i=0; i<COUNT; ++i) {
		OFFSETS[i] = values[i];
	}
	return true;

}


void HX711BTA::calcul(long *results, int coefError, int coefCpt, int coefMoy) {
    
    
    double moyenne[COUNT] ;
	double moyenneS[COUNT] ;
    long minValues[COUNT];
    long maxValues[COUNT]; 
   
    
    int error = 10000 ;
    int errorS = 20000 ; 
    int cpt = 0 ; 
    
    // Tant que l'erreur est supérieur à 100 et que le compteur n'est pas arrivée à 7, on continu
    // en meme, on prend la valeur avec l'erreur la plus faible dans la variable moyenneS ; 
    while (error > coefError & cpt < coefCpt) {
    
        // initialise les variables
        for (int i=0; i < COUNT ; ++i) {
          minValues[i]=0x7FFFFFFF;
          maxValues[i]=0x80000000;
          moyenne[i] = 0 ;
        }
    
        // Calcul des moyennes 
         for (int i = 0 ; i < coefMoy ; i++) {
            readRaw(results);      
            delay(2);
            for (int j = 0 ; j < COUNT ; ++j) {
              moyenne[j] += results[j]   ;
              if (results[j] > maxValues[j]) { maxValues[j] = results[j] ; }
              if (results[j] < minValues[j]) { minValues[j] = results[j] ;  }
            }
        } 
    
        // Calcul de l'erreur
        error = 0 ; 
        for (int e = 0  ; e < COUNT ; ++e) { 
          error += abs(maxValues[e] - minValues[e])/ COUNT ;
        }
    
        // Test l'erreur et stock dans moyenneS
        if (error < errorS) {
          for (int e = 0  ; e < COUNT ; ++e) { 
            moyenneS[e] = moyenne[e] ; 
          }
          errorS = error ; 
        }
        cpt++ ;        
    }
	
         for (int i = 0; i < COUNT ; ++i) {
          results[i] = moyenneS[i] / coefMoy ;
        }
 
}


void HX711BTA::sendRawData(long *poidsA, int count, byte *dout) {
    
	set_dout(count, dout);
    set_gain(128);
	
	long resultat[COUNT];
    //scales.readRaw(results);
    calcul(resultat, TOLERANCE_A,ITERATION,MOY);
    
    for (int i = 0; i < COUNT ; ++i) {

        //results[i] = r[i];
        long raw =  resultat[i];
        if (resultat[i] < miniA[i]) {
          resultat[i] = miniA[i] ;
        }
        if (resultat[i] > maxiA[i]) {
          resultat[i] = maxiA[i] ;
        }
        
        double haut = (resultat[i] - miniA[i]) ;
        double bas = maxiA[i] - miniA[i] ;
        double inter = haut/bas * 4095 ;
        poidsA[i] =  inter ; // CHANNEL A (128) ;  map(resultat[i], miniA[i], maxiA[i], 0, 4096) ; //
    //Serial.println(SDISPLAY);
    //    #if SDISPLAY
          Serial.print(" A: ");
          Serial.print(i); 
          Serial.print("Brut: ");
          Serial.print(raw);
          Serial.print(" haut: ");
          Serial.print(haut);     
          Serial.print(" bas: ");
          Serial.print(bas);     
          Serial.print(" : ");
          Serial.print( resultat[i]);
          Serial.print(" P : ");
          Serial.println( poidsA[i]);
          //Serial.print( (i != scales.get_count() - 1) ? "\t" : "\n");
     //   #endif
    }

  }


void HX711BTA::sendRawDataB(long *poidsB, int count, byte *dout) {

	set_dout(count, dout);
    set_gain(32);
	
    long resultsB[COUNT];
	
    calcul(resultsB, TOLERANCE_B,ITERATION,MOY);
    
    for (int i = 0; i < COUNT ; ++i) {

        //resultsB[i] = r[i]; 
        long raw =  resultsB[i];
        if (resultsB[i] < miniB[i]) {
          resultsB[i] = miniB[i] ;
        }
        if (resultsB[i] > maxiB[i]) {
          resultsB[i] = maxiB[i] ;
        }
        double haut = (resultsB[i] - miniB[i]) ;
        double bas = maxiB[i] - miniB[i] ;
        double inter = haut/bas * 4095 ;
        poidsB[i] = inter ; 
        
        #if SDISPLAY
          Serial.print(" B : ");
          Serial.print(i);
          Serial.print("Brut : ");
          Serial.print(raw);
          
          Serial.print(" - ");
          Serial.print( resultsB[i]);
          Serial.print(" P : ");
          Serial.println( poidsB[i]);
          //Serial.print( (i != scales.get_count() - 1) ? "\t" : "\n");
         #endif
    }

  }

void HX711BTA::ab(long *resultat, int count, byte *dout, String channel){
	
	if (channel = "A") {
		set_dout(count, dout);
		set_gain(128);	
		//long resultat[COUNT];    
		calcul(resultat, TOLERANCE_A,ITERATION,MOY);
		
	} else if (channel = "B") {
		set_dout(count, dout);
		set_gain(32);	
		//long resultat[COUNT];    
		calcul(resultat, TOLERANCE_B,ITERATION,MOY);
		
	}
}

void HX711BTA::setToleranceA(long tolerance) {
	TOLERANCE_A = tolerance ; 
	
}

void HX711BTA::setToleranceB(long tolerance) {
	TOLERANCE_B = tolerance ; 
	
}

void HX711BTA::setMoyenne(int moy) {
	MOY = moy ; 	
}

void HX711BTA::setIteration(int iteration) {
	ITERATION = iteration ; 	
}

//reads from all cahnnels and sets the values into the passed long array pointer (which must have at least 'count' cells allocated)
//if you are only reading to toggle the line, and not to get values (such as in the case of setting gains) you can pass NULL.
void HX711BTA::read(long *result) {
    
    readRaw(result);
    
    // Datasheet indicates the value is returned as a two's complement value, so 'stretch' the 24th bit to fit into 32 bits. 
	if (NULL!=result) {
		for (int j = 0; j < COUNT; ++j) {
		    result[j] -= OFFSETS[j];   	
		}
	}
}


void HX711BTA::readRaw(long *result) {
	int i,j;
	// wait for all the chips to become ready
	//while (!is_ready());
	long basis[24] = {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
	/*for (int u = 0 ; u < 24 ; ++u) {
		Serial.print(u);
		Serial.print(" ");
		Serial.println(basis[u]);
	}*/
	if (!wait_ready_timeout(2000,10)) defineOk();

	// pulse the clock pin 24 times to read the data
	for (i = 0; i < 24; ++i) {
		digitalWrite(PD_SCK, HIGH);
		if (NULL!=result) {
			for (j = 0; j < COUNT; ++j) {
				if (OK[j]) {
					bitWrite(result[j], 23-i, digitalRead(DOUT[j]));
				} else {
					bitWrite(result[j], 23-i, basis[i]);
					//Serial.print(i);
					//Serial.print(" ");
					//Serial.print(j);
					//Serial.print(" ");
					//Serial.println(basis[j]);
					
					
				}
			}
		}
		digitalWrite(PD_SCK, LOW);
	}
   
	// set the channel and the gain factor for the next reading using the clock pin
	for (i = 0; i < GAIN; ++i) {
		digitalWrite(PD_SCK, HIGH);
		digitalWrite(PD_SCK, LOW);
	}

    // Datasheet indicates the value is returned as a two's complement value, so 'stretch' the 24th bit to fit into 32 bits. 
    if (NULL!=result) {
	    for (j = 0; j < COUNT; ++j) {
	    	if ( ( result[j] & 0x00800000 ) ) {
	    		result[j] |= 0xFF000000;
	    	} else {
	    		result[j] &= 0x00FFFFFF; //required in lieu of re-setting the value to zero before shifting bits in.
	    	}
	    } 

    }
}

void HX711BTA::setDebugEnable(bool debugEnable) {
	debugEnabled = debugEnable;
}

void HX711BTA::power_down() {
	digitalWrite(PD_SCK, LOW);
	digitalWrite(PD_SCK, HIGH);
}

void HX711BTA::power_up() {
	digitalWrite(PD_SCK, LOW);
}

void HX711BTA::reboot() {
   NVIC_SystemReset();
   while (1) ;
}

String HX711BTA::lire()
{
// Read serial input:
  String inString = "" ; 
  while (inString == "") {
    while (Serial.available() > 0) {
      int inChar = Serial.read();
      //if (isDigit(inChar)) {
      if (inChar != '\n') {
        // convert the incoming byte to a char and add it to the string:
        inString += (char)inChar;
      }
      // if you get a newline, print the string, then the string's value:
      if (inChar == '\n') {
        //Serial.print("Value:");
        //Serial.println(inString.toInt());
        //Serial.print("String: ");
        //Serial.println(inString);
        // clear the string for new input:
        //inString = "";
      }
    }
  
  }
  return inString;
}

void HX711BTA::etalonnage() {
	long vide[1] ;
	vide[0] = 0 ;
	
    //long maxiVide = -3000000;
    //long miniVide = 3000000;
    long maxi;
    long mini ;
    float error ; 
    String inString ;
    String channel;
    int pinb ;
    byte TOUTS[1] = {0};
    int TCH = 1 ; 
    double moyenne = 0 ;
    double moyenne0[1] = {0} ;
    double moyenne40;
    int poids40;
	int poids0;
    double inter ;
	double inter0;
	double inter1;
	int PMAXI = 0 ;

	//Serial.begin(BAUD);
	//while(!Serial) {}; 
	
	while (1) {
	  //vide = 0 ;
	  channel = "" ;
	  
	  while (vide[0] == 0) {
		
		Serial.println("-- Valeur du Poids maxi --");
		do {
			Serial.print("Entrer la poids ? ");
			lire();
			PMAXI = inString.toInt() ;		
		} while (PMAXI <= 0) ; 		
		Serial.println(PMAXI);
		
		Serial.println("-- Balance à vide --");
		do {
			Serial.print("Entrer la Channel : A/B ? ");
			lire();
			channel = inString ;		
		} while (channel != "A" || channel != "B") ; 
		Serial.println(channel);
		
		do {
			Serial.print("N° de PIN [0-5]: ?");
			lire();
			pinb = inString.toInt();
		} while (pinb < 0 && pinb > 5) ; 		
		Serial.println(pinb);
		
		TOUTS[0] = pinb ; 
		
		moyenne = 0 ;
		
		//long vide[1];
		ab(vide, 1 , TOUTS , channel);
		
	  }
	  
		Serial.print("Valeur brut à vide : ");
		Serial.println(vide[0]);
	
		Serial.println("Balance avec le poids tare"); 
		do  {
			Serial.println("Entrer 0");
			lire();      
			poids0 = inString.toInt();
			
		} while (poids0 != 0) ;  
		        
			 
		long moyenne0[1];
		ab(moyenne0, 1 , TOUTS , channel);
		Serial.print("Valeur brut à 0kg : ");
		Serial.println(moyenne0[0]);
				  
		Serial.println("Poser sur la balance un poids important");
		do  {
			Serial.print("Entrer la valeur du poids : ");
			lire();      
			poids40 = inString.toInt();
		} while (poids40 <= 0) ; 
		
		Serial.print(poids40);
		Serial.println(" kg");
		
		if (poids40 > 0) {
			
			long moyenne40[1];
			ab(moyenne40, 1 , TOUTS , channel);
			Serial.print("Valeur brut à 0kg : ");
			Serial.println(moyenne40[0]);
		
			double moyenne140 = (moyenne40[0] - moyenne0[0]) * PMAXI / poids40 + moyenne0[0]  ;
			maxi = moyenne140 ;
			mini = vide[0] ; 

			double haut = (moyenne40[0] - mini) ;
			double bas = maxi - mini ;
			inter = haut/bas * 4095 ;
						  
			Serial.print("valeur brute mini à vide : ");
			Serial.println(mini);
			Serial.print("Valeur brute maxi à ");
			Serial.print(PMAXI);
			Serial.print(" kg : ");
			Serial.println(maxi);
			Serial.print("Valeur pour ") ; 
			Serial.print(poids40);
			Serial.print("kg :  ");
			Serial.println(inter);			
			Serial.println("");
			
			haut = (moyenne0[0] - mini) ;
			bas = maxi - mini ;
			inter0 = haut/bas * 4095 ;

			Serial.print("Valeur pour ") ; 
			Serial.print(poids0);
			Serial.print("kg : ");
			Serial.println(inter0);

			double alpha = (inter - inter0) / poids40 ; 
			double beta = - inter0 * poids40 / (inter - inter0)  ; 
			Serial.print("1 / alpha : ") ;
			Serial.println(alpha);
			Serial.print("Beta : ");
			Serial.println(beta);
			
				   
		  }
	   while (inString != "next") {
		   Serial.println("Entrer une nouvelle valeur de poids après avoir poser le poids");
		  lire();      
		  long px = inString.toInt();

		  if (px > 0) {
			
			long mx[1];
			ab(mx, 1 , TOUTS , channel);
					
			double haut = (mx[0] - mini) ;
			double bas = maxi - mini ;
			inter1 = haut/bas * 4095 ;
			
			Serial.print("Valeur 12bit pour ") ; 
			Serial.print(px);
			Serial.print("kg :  ");
			Serial.println(inter1);			
			
			
			
			double alpha = (inter - inter1) / (poids40 - px) ; 
			double beta = px - inter1 * (poids40 - px) / (inter - inter1)  ; 
			Serial.print("1 / alpha : ") ;
			Serial.println(alpha);
			Serial.print("Beta : ");
			Serial.println(beta);
			Serial.println("");
			/*
			if (moy(channel, pinb)) {
				double mx = moyenne / MOYENNE ; 
				double haut = (mx - mini) ;
				double bas = maxi - mini ;
				double inter1 = haut/bas * 4095 ;
				Serial.print("Valeur pour ") ; 
				Serial.print(px);
				Serial.print("kg : ");
				Serial.println(inter1);
				
				Serial.print(" Erreur de mesure : ");
				Serial.println(error);
				Serial.println("");

				double alpha = (inter - inter1) / (poids40 - px)  ; 
				double beta = px -  inter1 * (poids40 - px) / (inter - inter1) ; 
				Serial.print("1 / alpha : ") ;
				Serial.println(alpha);
				Serial.print("Beta : ");
				Serial.println(beta);
				Serial.println("");
				}
				*/
		  }
		}
	 
	  Serial.println("");
	  Serial.println("----------- Nouvel étalonnage ---------------");
}
}
  
