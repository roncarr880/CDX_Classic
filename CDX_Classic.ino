/*  CDX_Classic - A cw and digital modes radio using some classic qrp circuits.  Processor is an Arduino Nano.
 *   
 *  by KE1MU
 *  
 *  
 *  vfo is clock 0 using PLLB, high side vfo
 *  bfo is clock 1 using PLLA
 *  transmit uses clock 2
 *  
 *  Timer2
 *    D3 is audio out pwm, D11 is transmitter bias.
 *    TCCR2A  10 10 0011 for fast pwm, 1000 0011 enables ouput pin A, 0010 0011 enables pin B, A is bias, B is audio
 *    TCCR2B  0x01 for fastest clock
 *    TIMSK2  0x01 for overflow interrutps for audio
 *    Load OCR2A for bias value, OCR2B for audio
 *    
 *  Turning clocks on and off or reseting Si5351 causes TX glitch.  Caused by capacitive coupling of the clock?
 *    Fixed by keying the driver circuit.
 *    
 */

#define SWAP_TAP_DTAP 1         // swap the button tap and dtap, tap selects, dtap changes now 

#define SI5351 0x60     // I2C address
#define CLOCK_FREQ 25000045L
//  starting addresses of phase lock loop registers
#define PLLA 26
#define PLLB 34

#include <OLED1306_Basic.h>
#include "clocks_20m.h"         // full si5351 init from clockbuilder desktop

#define ROW0   0
#define ROW1   8
#define ROW2  16
#define ROW3  24
#define ROW4  32
#define ROW5  40
#define ROW6  48
#define ROW7  56

  /* switch states */
#define IDLE_ 0
#define ARM  1
#define DARM 2
#define DONE 3
#define TAP  4
#define DTAP 5
#define LONGP 6


#define I2TBUFSIZE 64              // size power of 2.  max 256 as using 8 bit index
#define I2RBUFSIZE 2               // set to expected #of reads, power of 2.  Won't be doing any reads in this program.
#define I2INT_ENABLED 0            // 0 for polling in loop or timer, 1 for TWI interrupts

#define CW 0
#define DIGI 1
#define USB  2
#define LSB  3
#define CW_OFFSET 650

#define T2_BIAS 0x80
#define T2_AUDIO 0x20

#define BFO (9000000+400)        // bfo on the high side of the xtal filter ( set first, lsb? )
#define BW  -2400                // band width of the xtal filter, bfo on low side is BFO - BW
                                 // this works backwards from what I expected  

//  I2C buffers and indexes
unsigned int i2buf[I2TBUFSIZE];   // writes
uint8_t i2rbuf[I2RBUFSIZE];       // reads
volatile uint8_t i2in,i2out;
volatile uint8_t i2rin,i2rout;
volatile uint8_t i2done = 1;

OLED1306  LCD;
extern unsigned char SmallFont[];
extern unsigned char MediumNumbers[];
extern unsigned char BigNumbers[];

// run tone detection when in DIGI mode
#define FSKON   (TIMSK1 = 1 << ICIE1)
#define FSKOFF  (TIMSK1 = 0)
volatile uint16_t raw_tone;
volatile uint8_t tone_flag;

uint32_t _P1;                  // base transmit si5351 solution from which tone offsets are calculated 
uint32_t _P2;
uint32_t _P3;

uint32_t freq;
uint32_t bfo;
uint8_t  mode;
uint8_t  band;
int16_t   step_ = 1000;
uint8_t   stp = 2;
uint8_t  gain;

struct BANDSTACK {
   uint32_t   freq;
   uint16_t   r_div;      // rx vfo = freq + bfo
   uint16_t   t_div;      // tx vfo = freq
   uint8_t    mode;
   uint8_t    filt_id;    // low pass filter ID for band selection, two bands per board, 4 boards possible, + ls bit is relay state
   uint8_t    cw_pwr;     // pwm bias drive for modes
   uint8_t    digi_pwr;
};

#define NUMBANDS 8
struct BANDSTACK bandstack[NUMBANDS] = {
  {  3928000, 54, 220, LSB, 0,  52,  25},
  {  7250000, 44, 122,LSB, 1, 101,  50},     //100
  { 10110000, 36,  86, CW, 2, 142,  70},     //140
  { 14100000, 30,  62, CW, 3, 201, 100},     //200
  { 18100000, 26,  50, DIGI, 7, 255, 120},   //240  was id 6 but rx band switch works better with this setting
  { 21100000, 24,  40, CW, 7, 255, 200},
  { 24900000, 20,  36, CW, 7, 255, 255},
  { 28100000, 18,  30, CW, 7, 255, 255}
};

// pin definitions
#define ENC_A 4
#define ENC_B 5
#define ENC_SW 8
#define BAND_ID0 A1
#define BAND_ID1 A2
#define DIT_PIN 9
#define DAH_PIN 10
#define TXENABLE 12     // wired to reset on stack
#define BAND_SW 2       // wired to 3.3 on stack
#define BIAS  11
#define AUDIO 3

#define DIT 1
#define DAH 2
uint8_t kmode;
uint8_t s_tone;
uint8_t kspeed = 14;

const char msg1[] PROGMEM = "CDX Classic";

uint8_t transmitting;
int tdown;

uint8_t ad_ref = 0x40;        // 0x40 5 volt, 0xc0 is 1.1 volt ( +13db )
uint8_t vol = 30;
uint8_t svol = 20;
uint8_t sstate[1];            // button switch state array

#define FREQ 0
#define VOLUME 1
#define SVOLUME 2
#define KSPEED 3
uint8_t enc_user;

int db_counter;               // !!! debug


void setup() {

   Serial.begin(38400);

   pinMode( ENC_A, INPUT_PULLUP ); 
   pinMode( ENC_B, INPUT_PULLUP ); 
   pinMode( ENC_SW, INPUT_PULLUP ); 
   pinMode( BAND_ID0, INPUT_PULLUP ); 
   pinMode( BAND_ID1, INPUT_PULLUP ); 
   pinMode( DIT_PIN, INPUT_PULLUP ); 
   pinMode( DAH_PIN, INPUT_PULLUP ); 
   pinMode( TXENABLE, OUTPUT ); 
   pinMode( BAND_SW, OUTPUT );
   pinMode( BIAS, OUTPUT );
   pinMode( AUDIO, OUTPUT );        
   digitalWrite( TXENABLE, LOW );
   digitalWrite( BAND_SW, LOW );
   digitalWrite( BIAS, LOW );

   band = read_filter_id(); 
   freq = bandstack[band].freq;
   
   i2init();
   LCD.InitLCD();
   LCD.setFont(SmallFont);
   LCD.clrScr();
   p_msg( msg1,0 );

   si5351_init();
   set_timer2( T2_AUDIO );
   set_bfo();

  // digi mode setup
   pinMode(7,INPUT);                // comparator reference
   TCCR1A = 0;                      // normal mode timer 1
   TCCR1B = 0x81;                   // noise cancel bit, divide by 1 prescale, 246 hz lower edge of xmit tone
   //TCCR1B = 0xc1;                 // rising clock on timer, don't think it really matters
   ACSR = (1<<ACIC);
   // FSKON;                           // enable capture interrupt !!! testing if enable here

   qsy(0);
   analogRead( A0 );                // init analog system
   if( bandstack[band].filt_id & 1 ) digitalWrite(BAND_SW,HIGH);
   else digitalWrite(BAND_SW,LOW);

   
}

void loop() {
static uint32_t tm;
int8_t t;
static int sec;
  
  i2poll();

  t = encoder();
  if( t ){
    switch( enc_user ){
      case FREQ:  qsy( t * step_ );  break;
      case VOLUME:
         vol += t;
         vol = constrain( vol, 0, 63 );
         status_display2();  
      break;
      case SVOLUME:
         svol += t;
         svol = constrain( svol, 0, 63 );
         status_display2();
      break;
      case KSPEED:
         kspeed += t;
         kspeed = constrain( kspeed, 10, 25 );
         status_display2();
      break;
    }
  }

  t = switches();
  if( t > DONE ){
     button( t );
     sstate[0] = DONE;
  }

  if( tm != millis() ){            // run once per millisecond functions
     tm = millis();
     
     if( mode == CW ){
        keyer();
     }
     if( mode == DIGI ){
       vox_check();
       if( transmitting ) send_tone();
     }

     if( tdown && s_tone == 0 ){       // semi break in delay
        if( --tdown == 0 ) rx();
     }

     ++sec;
     if( sec == 50 ){
        sec = 0;
        //Serial.print(16); Serial.write(' '); Serial.print(-16); Serial.write(' ');
        noInterrupts();   int val = db_counter; interrupts();
        Serial.println( val );
     }
  }

}


/****************  some CW mode code ******************/


int8_t read_paddles(){
int8_t pdl;

   pdl = 0;
   if( digitalRead( DAH_PIN ) == LOW ) pdl = 2;
   if( digitalRead( DIT_PIN ) == LOW ) pdl += 1;
   
   if( kmode & 4 ){                        // swap paddles 
      pdl <<= 1;
      if( pdl & 4 ) pdl += 1;
      pdl &= 3;                            // wire straight key to ring of plug
   }

   return pdl;
}

void side_tone_on(){
   
   if( mode == CW ) s_tone = 1;        // this function also keys tx during DIGI mode, but no side tone           
   if( transmitting == 0 ){
      tx();
      i2cd( SI5351,3,0b11111011);
      i2flush();
   }
   OCR2A = bandstack[band].cw_pwr;
   digitalWrite( TXENABLE, HIGH );
   
}

void side_tone_off(){

   s_tone = 0;
   digitalWrite( TXENABLE, LOW );
   OCR2A = 0;
   // delay(1);
   // i2cd(SI5351,3,0xff);           // do we need to key the drive?
   tdown = 400;                      // return to rx between letters
}


/*
void ptt(){                        // ssb PTT or straight key via keyer() function
static uint16_t dbounce;
static int8_t txing;                  // local dupe of variable transmitting.  uses: delayed breakin, wave shaping, practice mode
int8_t pdl;

   pdl = digitalRead( PTT ) ^ 1; 
   dbounce >>= 1;                  // shift bits right, any bit as 1 counts as on, no bits is off, stretches on time slightly
   if( pdl ) dbounce |= 0x400;     // 1ms delay per bit debounce, 0x400 is 15ms, 0x80 is 8ms

   if( mode == CW ){               // straight key mode
      if( txing && dbounce == 0 ) txing = 0, side_tone_off();
      else if( txing == 0 && dbounce ) txing = 1, side_tone_on();
   }
   else{                           // SSB
      if( txing && dbounce == 0 ) txing = 0 , rx();
      else if( txing == 0 && dbounce ) txing = 1, tx(); 
   }
}
*/


// http://cq-cq.eu/DJ5IL_rt007.pdf      all about the history of keyers

#define WEIGHT 200        // extra weight for keyed element

void keyer( ){            // this function is called once every millisecond
static int8_t state;
static int count;
static int8_t cel;           // current element
static int8_t nel;           // next element - memory
static int8_t arm;           // edge triggered memory mask
static int8_t iam;           // level triggered iambic mask
int8_t pdl;


//   if( (kmode & 3) == 0 ){    // straight key mode
//      ptt();
//      return;
//   }
   pdl = read_paddles();
   if( count ) --count;

   switch( state ){
     case 0:                               // idle
        cel = ( nel ) ? nel : pdl;         // get memory or read the paddles
        nel = 0;                           // clear memory
        if( cel == DIT + DAH ) cel = DIT;
        if( cel == 0 ) break;
        iam = (DIT+DAH) ^ cel;
        arm = ( iam ^ pdl ) & iam;         // memory only armed if alternate paddle is not pressed at this time, edge trigger
                                                    // have set up for mode A
        if( (kmode & 3) == 2 ) arm = iam;           // mode B.  Level triggered memory.
        if( (kmode & 3) == 3 ) iam = cel;           // ultimatic mode
        
        count = (1200+WEIGHT)/kspeed;
        if( cel == DAH ) count *= 3;
        state = 1;
        side_tone_on();
     break; 
     case 1:                                  // timing the current element. look for edge of the other paddle
        if( count ) nel = ( nel ) ? nel : pdl & arm;
        else{
           count = 1200/kspeed;
           state = 2;
           side_tone_off();
        }
     break;   
     case 2:                                  // timing the inter-element space
        if( count ) nel = ( nel ) ? nel : pdl & arm;
        else{
           nel = ( nel ) ? nel : pdl & iam;   // sample alternate at end of element and element space
           state = 0;
        }
     break;   
   }
  
}



void tx(){                // change to transmit
uint8_t t_mode;

    t_mode = T2_BIAS;
    if( mode == CW ) t_mode += T2_AUDIO;     // enable sidetone out
    set_timer2( t_mode );
    set_tx_clk();
    transmitting = 1;
}

void rx(){                // change to receive

    set_timer2( T2_AUDIO );
    set_bfo();
    transmitting = 0;
}



/***************************************************/

void set_timer2( uint8_t fun ){

    noInterrupts();
    OCR2A = 0;   OCR2B = 128;            // off values
    if( (fun & T2_AUDIO) == T2_AUDIO ) TIMSK2 = 1;
    else TIMSK2 = 0;

    TCCR2A = fun | 0x3;                  // fast pwm mode
    TCCR2B = ( fun ) ? 0x01 : 0x00;      // clock 63.5 khz or off
    //pinMode( 3, OUTPUT );
    //pinMode( 11, OUTPUT );
    interrupts();
}



#define ADPS 7                      // conversion speed, make as long as possible, a value of 3-4 should work at 63.5k

ISR( TIMER2_OVF_vect ){             // audio.  Rx sample rate 63.5 / 8 == 7.9375
static uint8_t state;
//static int adc;
static int val;
static uint8_t agc_counter;
//static uint8_t agc_recover;
//static int16_t sig_level;
static int agc;
//static int agc2;
//static uint8_t bits;

  // receiver
  ++state;
  state &= 7;
  switch( state ){
    case 0:
      val = ADC - 512;               // read adc and que next conversion
      ADMUX = ad_ref | 0;            // conversion on channel zero 
      ADCSRA = 0xC0 + ADPS;
     // db_counter = val;              // !!! debug
    break;
    case 1:
      if( ++agc_counter == 0 ){
         if( agc < vol ) ++agc;
         if( agc > vol ) --agc;
      }
    break;     
    case 2:   break;
    case 3:   break;
    case 4:   break;
    case 5:   break;
    case 6:   break;
    case 7:                          // write result
       val *= agc;
       val >>= 6;
       OCR2B = constrain(val+128,0,255);
       if( (abs(val)) > 2*(int)vol && agc > 0 ) --agc;     // volume ranges 0 to 63, signal +- 128
    break;
  }
  
}


ISR( TIMER1_CAPT_vect ){
static uint16_t  prev;
uint16_t now;

   now = ICR1;
   raw_tone = now - prev;
   prev = now;
   if( raw_tone > 5000 ) tone_flag = 1;    // else short count
}


uint8_t read_filter_id(){
uint8_t id;
uint8_t band;
uint8_t i;

   // get 1st band that has the matching filter ID of the lowpass filter installed
   band = id = 0;
   if( digitalRead( BAND_ID0 ) == HIGH ) id += 2;
   if( digitalRead( BAND_ID1 ) == HIGH ) id += 4;
   for( i = 0; i < NUMBANDS; ++i ){
      if( id == ( bandstack[i].filt_id & 6 ) ){
         band = i;
         break;
      }
   }
   return band;      
}


void band_change( uint8_t old_band ){

    bandstack[old_band].freq = freq;
    bandstack[old_band].mode = mode;
    freq = bandstack[band].freq;
    set_bfo();
    qsy(0);
    if( bandstack[band].filt_id & 1 ) digitalWrite(BAND_SW,HIGH);
    else digitalWrite(BAND_SW,LOW);

}

// since we have a PLL reset here, set up the vfo also. can use this function for band change also.
void set_bfo(){
uint16_t divi;

  i2cd( SI5351, 3, 0b11111111 );       // clocks off
  i2flush();

  mode = bandstack[band].mode;
  if( mode == DIGI || mode == USB ) bfo = BFO - BW;
  else bfo = BFO;
  
  divi = bandstack[band].r_div;
  si_pll_x( PLLB, freq+bfo, divi);
  si_load_divider( divi, 0, 0 );       // divider for clock 0
  
  si_pll_x( PLLA, bfo, 98 );
  //si_load_divider( 98, 2, 0 );        // keep clock 2 in step with clock 1 but disabled
  si_load_divider( 98, 1, 1 );        // clock 1, reset all
  i2flush();
  i2cd( SI5351, 3, 0b11111100 );      // enable vfo, bfo outputs, assumes rx is active
  i2flush();
  //delayMicroseconds(1500);            // same clock delay needed here?
  if( mode == DIGI ) FSKON;
  else FSKOFF;
}

// si5351 part of changing to tx mode
void set_tx_clk(){
uint32_t f;
uint16_t divi;
uint8_t  drive;

   i2cd( SI5351, 3, 0b11111111 );
   i2flush();
   f = freq;
   if( mode == CW ) f -= CW_OFFSET;
   divi = bandstack[band].t_div;
   si_pll_x( PLLA, f, divi );
   //si_load_divider( divi, 1, 0 );
   si_load_divider( divi, 2, 1 );
   i2flush();

   // see if the si5351 drive effects power out, less drive for 80,40,30 meters
   // !!! put this in band change instead of here
   drive = ( band > 3 ) ? 3 : band;
   i2cd( SI5351, 18, 0x4C + drive );
   
   i2cd( SI5351, 3, 0b11111011 );     // clock 2 on
   i2flush();
   delayMicroseconds(1250);           // clock takes awhile to start up, then add bias

}

void qsy( int val ){

   freq += val;
   display_freq();
   si_pll_x( PLLB, freq+bfo, bandstack[band].r_div);
  
}

void vox_check(){
static uint8_t vox;

   noInterrupts();
   if( tone_flag ){
      vox = 10;
   }
   else if( vox ) --vox;
   interrupts();

   if( vox && transmitting == 0 ) side_tone_on();
   else if( transmitting && vox == 0 && tdown == 0 ) side_tone_off();
  
}

void send_tone(){
float val;
uint16_t raw;
static uint8_t mod;                         // reduce some of the I2C traffic / interrupt latency
static float last_val;
static float err;

   raw = 0;
   if( transmitting == 0 ) return;
   noInterrupts();
     if( tone_flag ){
        raw = raw_tone;
        tone_flag = 0;
     }
   interrupts();
   if( raw != 0 ){
      raw = median( raw );
      ++mod;
      if( mod == 3 ) mod = 0;
      if( mod ) return;                       

      // sending the median of 3 values
      val =  16000000.0 / (float)raw;
      si_tone_offset( val );

      db_counter = val;
      // tone_testing( val );
   }

}

/*********
void tone_testing( float val ){
static float t0,t1,t2,t3;
static uint32_t tm;

  
   if( millis() - tm < 683 ) return;      // wspr baud rate

   t3 = t2; t2 = t1; t1 = t0; t0 = val;

   LCD.clrRow(3); LCD.clrRow(4); LCD.clrRow(5);
   LCD.printNumF( t0, 2, 5, ROW2 );
   LCD.printNumF( t1, 2, 5, ROW3 );
   LCD.printNumF( t2, 2, 5, ROW4 );
   LCD.printNumF( t3, 2, 5, ROW5 );
   LCD.printNumF( t3-t2,2,60,ROW5);
   LCD.printNumF( t2-t1,2,60,ROW4);
   LCD.printNumF(t1-t0,2,60,ROW3);
   tm = millis();  
}
***********/

uint16_t median( uint16_t val ){             // remove outliers
static uint16_t vals[3];
static uint8_t in;
uint8_t j,i,k;                               // low, median, high

   vals[in] = val;
   ++in;
   if( in > 2 ) in = 0;

   j = 0, i = 1, k = 2;                     // pretend they are in the correct order
   if( vals[j] > vals[k] ) k = 0, j = 2;    // swap guess high and low
   if( vals[i] < vals[j] ) i = j;           // is lower than the low guess, pick that one instead
   if( vals[i] > vals[k] ) i = k;           // is higher than the high guess

   return vals[i];
}




// Less flash used by avoiding print string?
void p_msg( const char *ptr, int row ){
char c;

   LCD.clrRow( row );
   LCD.gotoRowCol( row, 0 );
   while( ( c = pgm_read_byte(ptr++) ) ) LCD.putch(c);
   LCD.putch(' ');                      // make sure at least one write after gotoRowCol
  
}

// load the clock builder data
void si5351_init(){
uint8_t reg, data;
int i;

     delay(20);
     for( i = 0; i < 513; ++i ){
        reg = pgm_read_byte( &si5351_reg[i++] );
        data = pgm_read_byte( &si5351_reg[i] );
        if( reg == 255 ) break;         // end file marker
        i2cd( SI5351, reg, data );
        //Serial.print( reg );   Serial.write(' ');
        //Serial.println(data);
     }

        // set some divider registers that will never change
   for(int i = 0; i < 3; ++i ){
     i2cd(SI5351,42+8*i,0);
     i2cd(SI5351,43+8*i,1);
     i2cd(SI5351,47+8*i,0);
     i2cd(SI5351,48+8*i,0);
     i2cd(SI5351,49+8*i,0);
   }

   i2flush();

   i2cd( SI5351, 177, 0xAC );         // PLLA PLLB soft reset
   i2flush();
   delay(10);
   i2cd( SI5351, 177, 0xAC );
   i2flush();
}

void i2cd( unsigned char addr, unsigned char reg, unsigned char dat ){

    i2start( addr );
    i2send(  reg );                    // register or 1st data byte if no registers in device
    i2send(  dat );
    i2stop();
}


// load a new frequency into PLL A or B 
// the output divider is fixed per band in use and not recalculated
void  si_pll_x(unsigned char pll, uint64_t freq, int out_divider ){
 uint64_t a,b,c;
 uint64_t bc128;             // floor 128 * b/c term of equations
 uint64_t pll_freq;
 uint64_t clock_freq = (uint64_t)CLOCK_FREQ;

 uint32_t P1;            // PLL config register P1
 uint32_t P2;            // PLL config register P2
 uint32_t P3;            // PLL config register P3
 uint64_t r;


   // set c such that each b is 1hz change
   c = clock_freq / out_divider;     // max 1048575, for TX min divider of 24 for 25mhz clock, 26 for 27mhz clcok 
   if( c > 1048575 ) c = 1048575;    // vfo on high bands will have dividers less than 24
   
   pll_freq = freq * (uint64_t)out_divider;
   a = pll_freq / (clock_freq);
   r = pll_freq - a * (clock_freq);
   b = ( c * r ) / (clock_freq);
   bc128 =  (128 * r)/ (clock_freq);   // 128*b/c, b = c*r, sub c*r for b,  128*c*r/c,   128*r, div by cfreq, get some fraction of 128
   P1 = 128 * a + bc128 - 512;
   P2 = 128 * b - c * bc128;
   if( P2 >= c ) P2 -= c, ++P1;        // could this happen with truncation of unsigned values? have seen it once I think
   P3 = c;

   // save base solution for generating fractional xmit tone offsets, tx uses PLLA
   if( pll == PLLA ){
       _P1 = P1;  _P2 = P2; _P3 = P3;
   }
   i2cd(SI5351, pll + 0, (P3 & 0x0000FF00) >> 8);
   i2cd(SI5351, pll + 1, (P3 & 0x000000FF));
   i2cd(SI5351, pll + 2, (P1 & 0x00030000) >> 16);
   i2cd(SI5351, pll + 3, (P1 & 0x0000FF00) >> 8);
   i2cd(SI5351, pll + 4, (P1 & 0x000000FF));
   i2cd(SI5351, pll + 5, ((P3 & 0x000F0000) >> 12) | ((P2 & 0x000F0000) >> 16));
   i2cd(SI5351, pll + 6, (P2 & 0x0000FF00) >> 8);
   i2cd(SI5351, pll + 7, (P2 & 0x000000FF));
    
}


// init has loaded other registers with the clock builder values to allow this simplified code to work
void si_load_divider( int val, int clk , int rst){
 
   val = 128 * val - 512;
   i2cd( SI5351, 44+8*clk, (val >> 16 ) & 3 );
   i2cd( SI5351, 45+8*clk, ( val >> 8 ) & 0xff );
   i2cd( SI5351, 46+8*clk, val & 0xff );   
   if( rst ) i2cd( SI5351, 177, 0xA0 );         // PLLA PLLB soft reset needed?
}

void si_tone_offset( float val ){
 // P3 or c has been chosen such that a change of 1 hz changes P2 by 128
 uint32_t P1;            // PLL config register P1
 uint32_t P2;            // PLL config register P2
 uint32_t P3;            // PLL config register P3

   P1 = _P1;  P2 = _P2;  P3 = _P3;    // base value

   P2 += (uint32_t)(128.0 * val);     // add the tone offset, positive only
   while( P2 >= P3 ) P2 -= P3, ++P1;

   // transmitting using PLLA
   i2cd(SI5351, PLLA + 0, (P3 & 0x0000FF00) >> 8);
   i2cd(SI5351, PLLA + 1, (P3 & 0x000000FF));
   i2cd(SI5351, PLLA + 2, (P1 & 0x00030000) >> 16);
   i2cd(SI5351, PLLA + 3, (P1 & 0x0000FF00) >> 8);
   i2cd(SI5351, PLLA + 4, (P1 & 0x000000FF));
   i2cd(SI5351, PLLA + 5, ((P3 & 0x000F0000) >> 12) | ((P2 & 0x000F0000) >> 16));
   i2cd(SI5351, PLLA + 6, (P2 & 0x0000FF00) >> 8);
   i2cd(SI5351, PLLA + 7, (P2 & 0x000000FF));

}


/*****  Non-blocking  I2C  functions, polling or interrupt driven  ******/

/**************
// TWI interrupt version
// if twi interrupts enabled, then need a handler 
ISR(TWI_vect){
  i2poll();
  //if( gi2state == 0 ) i2poll();   // needed to get out of state zero. Fixed this issue I think.
  #ifdef I2STATS
     ++i2ints;
  #endif
}
***************/

void i2init(){
  TWSR = 0;
  TWBR = 12;    //8  500k, 12 400k, 72 100k   for 16 meg clock. ((F_CPU/freq)-16)/2
                //12 500k, 17 400k, 72 125k   for 20 meg clock.  8 625k 6 700k.  4 833k 
  TWDR = 0xFF;       // ?? why
  PRR &= 0x7F;
  TWSR = 1<<TWEN;
  i2done = 1;
}
// use some upper bits in the buffer for control
#define ISTART 0x100
#define ISTOP  0x200


// single buffered, wait for previous transfer to finish if any, queue a start condition
void i2start( unsigned char adr ){
unsigned int dat;
uint8_t t;

  if( i2done == 0 ) i2flush();
  dat = ( adr << 1 ) | ISTART;      // shift the address over and add the start flag
  i2send( dat );
  
}


void i2send( unsigned int data ){   // just save stuff in the buffer
uint8_t  next;
uint8_t  t;

  // check for buffer full
  next = (i2in + 1) & (I2TBUFSIZE-1);
  noInterrupts();
  t = i2out;
  interrupts();
  /***
  if( t == next ){
      polling = 1;          // may need to finish this transfer via polling as transfer is bigger than our buffer
      #ifdef I2STATS
        ++i2polls;
      #endif
  }
  ***/
  
  while( t == next ){       // the buffer is full, call poll to send some of the data. Some OLED writes are bigger than the buffer.
         noInterrupts(); 
         i2poll();                  // wait for i2out to move
         t = i2out;
         interrupts();
  }
  
  i2buf[i2in++] = data;
  i2in &= (I2TBUFSIZE - 1);
}

void i2stop( ){

   i2send( ISTOP );   // que a stop condition
   i2done = 0;
   noInterrupts();    // kick off sending this buffer
   i2poll();
   interrupts();
}


void i2flush(){  // call flush to empty out the buffer, waits on I2C transactions completed 
uint8_t  ex;

  ex = 1;
  while(ex){
     noInterrupts();
     ex = i2poll(); 
     interrupts();
  }
}

/***********    save some flash space, not doing I2C reads for this radio
// queue a read that will complete later
void i2queue_read( unsigned char adr, unsigned char reg, unsigned char qty ){
unsigned int dat;

   i2start( adr );
   i2send( reg );
   // i2stop();     // or repeated start
   dat = ((unsigned int)qty << 10) | ( adr << 1 ) | ISTART | 1;   // a start with the read bit set
   i2send( dat );
   i2stop();        // stop to complete the transaction
}

int i2read_int(){     // returns 2 values in i2c read queue as a signed integer
int data;

      if( i2rout == i2rin ) return 0;
      data = i2rbuf[i2rout++];
      i2rout &= (I2RBUFSIZE-1);
      data <<= 8;
      if( i2rout == i2rin ) return data;
      data |= i2rbuf[i2rout++];
      i2rout &= (I2RBUFSIZE-1);
      return data;
}

uint8_t i2available(){
uint8_t qty;

     qty = i2rin - i2rout;
     if( qty > I2RBUFSIZE ) qty += I2RBUFSIZE;    // some funky unsigned math
     return qty;
}
*********************/

// everything happens here.  Call this from loop or interrupt.
// Only state that does not return immediately is the i2stop.  It does not produce an interrupt.
// modified to take some of the cases out of the switch construct so they always get parsed.
// Fixes the need to double poll when in state 0. 

uint8_t i2poll(){    
static  uint8_t state = 0;
static unsigned int data;
static unsigned int read_qty;
static uint8_t first_read;

   
   switch( state ){
           
      case 1:  // test for start to clear, send saved data which has the device address
         if( (TWCR & (1<<TWINT)) ){
            state = ( data & 1 ) ? 4 : 2;    // read or write pending?
            first_read = 1;
            TWDR = data;
            TWCR = (1<<TWINT) | (1<<TWEN) | (I2INT_ENABLED);
         }
      break;
      case 2:  // test for done
         if( (TWCR & (1<<TWINT)) ){  
            state = 0;
         }
      break;
      case 4:  // non blocking read until count has expired
         if( (TWCR & (1<<TWINT)) ){
            // read data
            if( first_read ) first_read = 0;       // discard the 1st read, need 8 more clocks for real data
            else{
               i2rbuf[i2rin++] = TWDR;
               i2rin &= ( I2RBUFSIZE - 1 );
               if( --read_qty == 0 ) state = 0;    // done
            }
            
            if( read_qty ){                        // any left ?
               if( read_qty > 1 ) TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWEA) | (I2INT_ENABLED);  // not the last read
               else TWCR = (1<<TWINT) | (1<<TWEN) | (I2INT_ENABLED);                            // nack the last read
            }
         }
      break;    
   }

   // always test these conditions
   //      case 0:      // idle state or between characters
   if( state == 0 ){
        if( i2in != i2out ){   // get next character
           data = i2buf[i2out++];
           i2out &= (I2TBUFSIZE - 1 );
         
           if( data & ISTART ){   // start
              if( data & 1 ) read_qty = data >> 10;     // read queued
              data &= 0xff;
           //   while(TWCR & (1<<TWSTO));                                      // wait for previous stop to clear
              TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN) | (I2INT_ENABLED);  // set start condition
              state = 1; 
           }
           else if( data & ISTOP ){  // stop
              // set stop condition
              TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO) | (I2INT_ENABLED);
              state = 3;
              //i2done = 1;
           }
           else{   // just data to send
              TWDR = data;
              TWCR = (1<<TWINT) | (1<<TWEN) | (I2INT_ENABLED);
              state = 2;
           }
        }
        else TWCR = (1<<TWEN);      // stop interrupts, keep enabled
      // break;                     // no break, check for stop completion
   }

 
   if( state == 3 ){     // was once,  merged this processing into state 0, may not need to wait for stop
      //case 3:  // wait here for stop to clear. TWINT does not return to high and no interrupts happen on stop.
      //   if( state != 3 ) break;
         while( 1 ){
            if( (TWCR & (1<<TWSTO)) == 0 ){
               state = 0;
               i2done = 1;           // i2c transaction complete, buffer is free for next one
               break;
            }
         }
      //break;
   }
   
   if( i2in != i2out ) return (state + 8);
   else return state;
}

/*********** end I2C functions  ************/


void display_freq(){
int rem;
uint32_t f;
static uint8_t msg_displayed;                      // write the RIT message only once


  // if( rit_enabled == 0 ) msg_displayed = 0;
   f = freq;
   //if( mode == CW ) f -= CW_OFFSET;      // !!! revisit
   
   rem = f % 1000;

    // display big numbers in the blue area of the screen
    // font widths/height are: small 6 x 8, Medium 12 x 16, Big 14 x 24
    LCD.setFont(BigNumbers);
    LCD.printNumI(f/1000,12,ROW2,5,'/');
    LCD.setFont(MediumNumbers);
    LCD.printNumI(rem,5*14 + 12 + 3,ROW3,3,'0');
    LCD.setFont( SmallFont );                                                        // keep OLED in small text as the default font
   // if( rit_enabled && msg_displayed == 0 ) p_msg( msg2, 6 ), msg_displayed = 1;     // RIT message
   // LCD.gotoRowCol( 3,0 );
   // LCD.putch( band_priv(f));
 
}


int8_t encoder(){         /* read encoder, return 1, 0, or -1 */
static char mod;        /* encoder is divided by 4 because it has detents */
static char dir;        /* need same direction as last time, effective debounce */
static char last;       /* save the previous reading */
char new_;              /* this reading */
char b;

   //if( transmitting ) return 0;
   
   new_ = (digitalRead(ENC_B) << 1 ) | digitalRead(ENC_A);
   if( new_ == last ) return 0;       /* no change */

   b = ( (last << 1) ^ new_ ) & 2;    /* direction 2 or 0 from xor of last shifted and new data */
   last = new_;
   if( b != dir ){
      dir = b;
      return 0;      /* require two in the same direction serves as debounce */
   }
   mod = (mod + 1) & 3;       /* divide by 4 for encoder with detents */
   if( mod != 0 ) return 0;

   return ( (dir == 2 ) ? 1: -1 );   /* swap defines ENC_A, ENC_B if it works backwards */
}


      /* run the switch state machine, generic code for multiple switches even though have only one here */
int8_t switches(){
static uint8_t press_, nopress;
static uint32_t tm;
int  i,j;
int8_t sw;
int8_t s;

   if( tm == millis() ) return 0;      // run once per millisecond
   tm = millis();
   
   sw = ( digitalRead( ENC_SW ) == LOW ) ? 1 : 0;                 
   
   if( sw ) ++press_, nopress = 0;       /* only acting on one switch at a time */
   else ++nopress, press_ = 0;           /* so these simple vars work for all of them */

   /* run the state machine for all switches in a loop */
   for( i = 0, j = 1; i < 1; ++i ){
      s = sstate[i];
      switch(s){
         case DONE:  if( nopress >= 100 ) s = IDLE_;  break;
         case IDLE_:  if( ( j & sw ) && press_ >= 30 ) s = ARM;  break; /* pressed */
         case ARM:
            if( nopress >= 30 ) s = DARM;                      /* it will be a tap or double tap */
            if( press_ >= 240 ) s = LONGP;                     // long press
         break;
         case DARM:
            if( nopress >= 240 )  s = TAP;
            if( press_ >= 30 )    s = DTAP;
         break;
      }
      sstate[i] = s; 
      j <<= 1;
   }
   
   return sstate[0];      // only one switch implemented so can return its value
}

#define NUM_MENU 5
uint8_t *svar[NUM_MENU] =                  // variables to change
{&stp, &band, &mode, &kmode, &gain };
uint8_t smax[NUM_MENU] =                   // max value allowed
{ 4,      8,     4,      4,     2 };


// tap inc value, dtap inc function, long change encoder user.  status_display
void button( int8_t val ){
static uint8_t function;
int i;
uint8_t ob;

  ob = band;

  if( SWAP_TAP_DTAP ){                   // tap selects value, dtap changes value now
    if( val == TAP ) val = DTAP;
    else if( val == DTAP ) val = TAP;
  }

  switch( val ){
    case TAP:
      ++*svar[function];
      if( *svar[function] >= smax[function] ) *svar[function] = 0;
      
      // special case freq step
      if( function == 0 ){
         step_ = 10;  i = stp;
         while( i-- ) step_ *= 10;
      }
      
      // special case weak signal gain       
      if( function == 4 ){
         ad_ref = ( gain ) ? 0xc0 : 0x40;   // 0x40 5 volt, 0xc0 is 1.1 volt ( +13db )
      }

      if( function == 1 ) band_change(ob);
      if( function == 2 ) {                  // mode change
         band_change(band);                  // implement as band change to same band
      }
      
    break;

    case DTAP:
      if( ++function > NUM_MENU-1 ) function = 0;
    break;

    case LONGP:
      if( ++enc_user > 3 ) enc_user = 0;
    break;
  }

  status_display( function );
  
}

const char status_msg[] PROGMEM = "StepBandModeKeyMGain";
const char modes_msg[] PROGMEM = " CW DIGIUSB LSB ";
const char enc_msg[] PROGMEM = "FreqVol SVolKspd";
void status_display( uint8_t f ){
int i,k;

  LCD.clrRow(0);
  LCD.gotoRowCol( 0,0 );
  k = 4*f;                 // index msg
  for( i = 0; i < 4; ++i ) LCD.putch( pgm_read_byte( &status_msg[k++] ) );
  LCD.putch(' ');
  
  switch( f ){
    case 0:  LCD.printNumI( step_, 6*6, 0 ); break;
    case 2:
      k = 4 * mode;
      LCD.gotoRowCol( 0, 6*6 );
      for( i = 0; i < 4; ++i ) LCD.putch( pgm_read_byte( &modes_msg[k++] ) );
      LCD.putch(' ');
    break;  
    default:  LCD.printNumI( *svar[f],6*6,0 ); break;
  }

  status_display2();
  
}

void status_display2(){      // display encoder user status
int i,k;

  LCD.clrRow(1);
  LCD.gotoRowCol( 1,0 );
  k = 4*enc_user;
  for( i = 0; i < 4; ++i ) LCD.putch( pgm_read_byte( &enc_msg[k++] ) );

  switch( enc_user ){
    case 1:
       LCD.printNumI( vol, 6*6, ROW1 );
    break;
    case 2:
       LCD.printNumI( svol, 6*6, ROW1 );
    break;
    case 3:
       LCD.printNumI( kspeed, 6*6,ROW1 );
    break;
  }

  
}
