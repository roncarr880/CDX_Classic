/*  CDX_Classic - A cw and digital modes radio using some classic qrp circuits.  Processor is an Arduino Nano.
 *   
 *  by KE1MU
 *  
 *  
 *  vfo is clock 0 using PLLB, high side vfo
 *  bfo is clock 1 using PLLA
 *  transmit uses clock 1 and 2 with 180 deg phase diff
 */

#define SI5351 0x60     // I2C address
#define CLOCK_FREQ 25001740L
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

#define I2TBUFSIZE 64              // size power of 2.  max 256 as using 8 bit index
#define I2RBUFSIZE 2               // set to expected #of reads, power of 2.  Won't be doing any reads in this program.
#define I2INT_ENABLED 0            // 0 for polling in loop or timer, 1 for TWI interrupts

#define CW 0
#define DIGI 1
#define USB  2
#define LSB  3
#define CW_OFFSET 650

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

struct BANDSTACK {
   uint32_t   freq;
   uint16_t   r_div;      // rx vfo = freq + bfo
   uint16_t   t_div;      // tx vfo = freq
   uint8_t    mode;
   uint8_t    filt_id;    // low pass filter ID for band selection, two bands per board, 4 boards possible, + ls bit is relay state
};

#define NUMBANDS 8
struct BANDSTACK bandstack[NUMBANDS] = {
  {  3600000, 54, 220, CW, 0},
  {  7100000, 44, 122, CW, 1},
  { 10110000, 36,  86, CW, 2},
  { 14100000, 30,  62, CW, 3},
  { 18100000, 26,  50, CW, 4},
  { 21100000, 24,  40, CW, 4},
  { 24900000, 20,  36, CW, 5},
  { 28100000, 18,  30, CW, 5}
};

// pin definitions
#define ENC_A 4
#define ENC_B 5
#define ENC_SW 8
#define BAND_ID0 A1
#define BAND_ID1 A2
#define DIT_PIN 9
#define DAH_PIN 10
#define TXENABLE 12
#define BAND_SW 2

const char msg1[] PROGMEM = "CDX Classic";


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
   digitalWrite( TXENABLE, HIGH );      // active low on hct245, this is disabled
   digitalWrite( BAND_SW, LOW );

   band = read_filter_id();
   freq = bandstack[band].freq;
   
   i2init();
   LCD.InitLCD();
   LCD.setFont(SmallFont);
   LCD.clrScr();
   p_msg( msg1,0 );

   si5351_init();
   set_bfo();

  // digi mode setup
   pinMode(7,INPUT);                // comparator reference
   TCCR1A = 0;                      // normal mode timer 1
   TCCR1B = 0x81;                   // noise cancel bit, divide by 1 prescale, 246 hz lower edge of xmit tone
   //TCCR1B = 0xc1;                 // rising clock on timer, don't think it really matters
   ACSR = (1<<ACIC);
   FSKON;                           // enable capture interrupt !!! testing if enable here

}

void loop() {
  
  i2poll();
  send_tone();


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
   for( i = 0; i < NUMBANDS; ++i ) if( id == bandstack[i].filt_id ) band = i;
   return band;      
}

// since we have a PLL reset here, set up the vfo also. can use this function for band change also.
void set_bfo(){
uint16_t divi;

  i2cd( SI5351, 3, 0b11111111 );       // clocks off

  mode = bandstack[band].mode;
  if( mode == DIGI || mode == USB ) bfo = 9000000;
  else bfo = 9004000;
  
  divi = bandstack[band].r_div;
  freq = bandstack[band].freq;
  si_pll_x( PLLB, freq+bfo, divi);
  si_load_divider( divi, 0, 0 );       // divider for clock 0
  
  si_pll_x( PLLA, bfo, 98 );
  si_load_divider( 98, 2, 0 );        // keep clock 2 in step with clock 1 but disabled
  si_load_divider( 98, 1, 1 );        // clock 1, reset all
  i2cd( SI5351, 3, 0b11111100 );      // enable vfo, bfo outputs, assumes rx is active
}

// si5351 part of changing to tx mode
void set_tx_clk(){
uint32_t f;
uint16_t divi;

   i2cd( SI5351, 3, 0b11111111 );
   f = freq;
   if( mode == CW ) f -= CW_OFFSET;
   divi = bandstack[band].t_div;
   si_pll_x( PLLA, f, divi );
   si_load_divider( divi, 1, 0 );
   si_load_divider( divi, 2, 1 );
   i2cd( SI5351, 3, 0b11111001 );     // clocks 1 and 2 180 degrees phase diff

}

void qsy( int val ){

  
}

void send_tone(){
float val;
uint16_t raw;
static uint8_t mod;                         // reduce some of the I2C traffic / interrupt latency
static float last_val;
static float err;

   raw = 0;
   //if( transmitting == 0 ) return;
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
      
      tone_testing( val );
   }

}

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

     for( i = 0; i < 513; ++i ){
        reg = pgm_read_byte( &si5351_reg[i++] );
        data = pgm_read_byte( &si5351_reg[i] );
        if( reg == 255 ) break;         // end file marker
        i2cd( SI5351, reg, data );
     }
     i2cd( SI5351, 177, 0xAC );         // PLLA PLLB soft reset
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
