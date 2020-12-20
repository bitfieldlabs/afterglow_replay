/***********************************************************************
 *  afterglow replay:
 *      Copyright (c) 2020 bitfield labs
 *
 ***********************************************************************
 *  This file is part of the afterglow pinball LED project:
 *  https://github.com/bitfieldlabs/afterglow_replay
 *
 *  afterglow replay is free software: you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public License as
 *  published by the Free Software Foundation, either version 3 of the
 *  License, or (at your option) any later version.
 *
 *  afterglow replay is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with afterglow replay.
 *  If not, see <http://www.gnu.org/licenses/>.
 ***********************************************************************/
 
//------------------------------------------------------------------------------
/* This code assumes following pin layout:
 *
 *  +----------+---------------+-----------+---------------+--------------+
 *  | Name     | Function      | Nano Pin# | Register, Bit | Mode         |
 *  +----------+---------------+-----------+---------------+--------------+
 *  +----------+---------------+-----------+---------------+--------------+
*/

#include <EEPROM.h>
#include <avr/wdt.h>
#include <avr/boot.h>

//------------------------------------------------------------------------------
// Setup

// Afterglow version number
#define AFTERGLOW_REPLAY_VERSION 100

// Afterglow replay board revision. Currently v1.0.
#define BOARD_REV 10

// turn debug output via serial on/off
#define DEBUG_SERIAL 0

// original matrix update interval [us]
#define ORIG_INT (2000)

// local time interval, config A [us]
#define TTAG_INT_A (250)

// cycles per original interval, config A
#define ORIG_CYCLES_A (ORIG_INT / TTAG_INT_A)

// number of columns in the lamp matrix
#define NUM_COL 8

// number of rows in the lamp matrix
#define NUM_ROW 8

// default glow duration [ms]
#define DEFAULT_GLOWDUR 180

// glow duration scaling in the configuration
#define GLOWDUR_CFG_SCALE 10

// default maximum lamp brightness 0-7
#define DEFAULT_BRIGHTNESS 7

// afterglow LED glow duration [ms]
#define AFTERGLOW_LED_DUR (2000)

// current supervision on pin A0
#define CURR_MEAS_PIN A0

// test mode setup
#define TEST_MODE_NUMMODES 7    // number of test modes
#define TEST_MODE_DUR 8         // test duration per mode [s]
#define TESTMODE_INT (500)      // test mode lamp switch interval [ms]
#define TESTMODE_CYCLES_A ((uint32_t)TESTMODE_INT * 1000UL / (uint32_t)TTAG_INT_A) // number of cycles per testmode interval, config A

// enable lamp replay in test mode
#define REPLAY_ENABLED

#ifdef REPLAY_ENABLED
// Replay time scale [us]
#define REPLAY_TTAG_SCALE 16000

// Replay record
typedef struct AG_LAMP_SWITCH_s
{
    uint16_t col : 3;    // lamp column
    uint16_t row : 3;    // lamp row
    uint16_t dttag : 10; // delta time tag [16ms] to the last event
} AG_LAMP_SWITCH_t;

// Replay logic
byte replay(void);

// Number of replay records
int numReplays(void);
#endif // REPLAY_ENABLED


//------------------------------------------------------------------------------
// global variables

// Lamp matrix 'memory'
static uint16_t sMatrixState[NUM_COL][NUM_ROW];
    
// local time
static uint32_t sTtag = 0;

// interrupt runtime counters [cycles]
static uint16_t sLastIntTime = 0;
static uint16_t sMaxIntTime = 0;

// remember the last column and row samples
static byte sLastColMask = 0;
static byte sLastRowMask = 0;

#if DEBUG_SERIAL
static byte sLastOutColMask = 0;
static byte sLastOutRowMask = 0;
static uint32_t sBadColCounter = 0;
static uint32_t sBadColOrderCounter = 0;
static byte sLastBadCol = 0;
static byte sLastGoodCol = 0;
static int sMaxCurr = 0;
static int sLastCurr = 0;
#endif

// afterglow configuration data definition
typedef struct AFTERGLOW_CFG_s
{
    uint16_t version;                         // afterglow version of the configuration
    uint16_t res;                             // reserved bytes
    uint8_t lampGlowDur[NUM_COL][NUM_ROW];    // Lamp matrix glow duration configuration [ms * GLOWDUR_CFG_SCALE]
    uint8_t lampBrightness[NUM_COL][NUM_ROW]; // Lamp matrix maximum brightness configuration (0-7)
    uint32_t crc;                             // data checksum
} AFTERGLOW_CFG_t;

// afterglow configuration
static AFTERGLOW_CFG_t sCfg;

// precalculated glow steps for each lamp
static uint16_t sGlowSteps[NUM_COL][NUM_ROW];

// precalculated maximum subcycle for lamp activation (brightness)
static byte sMaxSubcycle[NUM_COL][NUM_ROW];


//------------------------------------------------------------------------------
void setup()
{
    noInterrupts(); // disable all interrupts

    // setup the timers
    timerSetup();

    // I/O pin setup
    DDRB |= B00000011;
    DDRC |= B00001111;
    DDRD |= B11111110;
    
    // keep OE high for now
    PORTC |= B00001000;

    // Configure the ADC clock to 1MHz by setting the prescaler to 16.
    // This should allow for fast analog pin sampling without much loss of precision.
    // defines for setting and clearing register bits.
    _SFR_BYTE(ADCSRA) |= _BV(ADPS2);
    _SFR_BYTE(ADCSRA) &= ~_BV(ADPS1);
    _SFR_BYTE(ADCSRA) &= ~_BV(ADPS0);

    // initialize the data
    memset(sMatrixState, 0, sizeof(sMatrixState));

    // set default configuration
    setDefaultCfg();

    // Apply the configuration
    // This will prepare all values for the interrupt handlers.
    applyCfg();

    // enable serial output at 115200 baudrate
#if DEBUG_SERIAL
    Serial.begin(115200);
    Serial.print("afterglow replay v");
    Serial.print(AFTERGLOW_REPLAY_VERSION);
    Serial.println(" (c) 2018 morbid cornflakes");
    // check the extended fuse for brown out detection level
    uint8_t efuse = boot_lock_fuse_bits_get(GET_EXTENDED_FUSE_BITS);
    Serial.println("-----------------------------------------------");
    uint8_t bodBits = (efuse & 0x7);
    Serial.print("efuse BOD ");
    Serial.println((bodBits == 0x07) ? "OFF" : (bodBits == 0x04) ? "4.3V" : (bodBits == 0x05) ? "2.7V" : "1.8V");
#ifdef REPLAY_ENABLED
    Serial.print("Replay Table Size: ");
    Serial.println(numReplays());
#endif
    Serial.print("CFG from ");
    Serial.print(cfgLoaded ? "EEPROM" : "DEFAULT");
    if (err)
    {
        Serial.print(" err ");
        Serial.print(err);
    }
    Serial.println("");
#endif

    // enable all interrupts
    interrupts();

    // enable a strict 15ms watchdog
    wdt_enable(WDTO_15MS);
}

//------------------------------------------------------------------------------
void timerSetup(void)
{
    // Use Timer1 to create an interrupt every TTAG_INT us.
    // This will be the heartbeat of our realtime task.
    TCCR1A = 0;
    TCCR1B = 0;
    // set compare match register for TTAG_INT us increments
    // prescaler is at 1, so counting real clock cycles
    OCR1A = (TTAG_INT_A * 16);   // [16MHz clock cycles]
    // turn on CTC mode
    TCCR1B |= (1 << WGM12);
    // Set CS10 bit so timer runs at clock speed
    TCCR1B |= (1 << CS10);  
    // enable timer compare interrupt
    TIMSK1 |= (1 << OCIE1A);
}

//------------------------------------------------------------------------------
void start()
{
    // enable the timer compare interrupt
    TIMSK1 |= (1 << OCIE1A);

    // enable a strict 15ms watchdog
    wdt_enable(WDTO_15MS);
}

//------------------------------------------------------------------------------
void stop()
{
    // disable the watchdog
    wdt_disable();

    // disable the timer compare interrupt
    TIMSK1 &= ~(1 << OCIE1A);

    // pull OE high to disable all outputs
    PORTC |= B00001000;
}

//------------------------------------------------------------------------------
// Timer1 interrupt handler
// This is the realtime task heartbeat. All the magic happens here.
ISR(TIMER1_COMPA_vect)
{   
    // time is running
    uint16_t startCnt = TCNT1;
    sTtag++;

    // kick the dog
    wdt_reset();

    // afterglow mode
    driveLampMatrix();

    // replay mode
    uint16_t inData = testModeInput();

    byte inColMask = (inData >> 8); // LSB is col 0, MSB is col 7
    byte inRowMask = ~(byte)inData; // high means OFF, LSB is row 0, MSB is row 7

    // evaluate the column reading
    // only one bit should be set as only one column can be active at a time
    uint32_t inCol = NUM_COL;
    switch (inColMask)
    {
        case 0x01: inCol = 0; break;
        case 0x02: inCol = 1; break;
        case 0x04: inCol = 2; break;
        case 0x08: inCol = 3; break;
        case 0x10: inCol = 4; break;
        case 0x20: inCol = 5; break;
        case 0x40: inCol = 6; break;
        case 0x80: inCol = 7; break;
        default:
        {
            // This may happen if the sample is taken in between column transition.
            // Depending on the pinball ROM version the duration of this transition varies.
            // On a Whitewater with Home ROM LH6 (contains anti ghosting updates) this
            // gap was measured to be around 30us long.
            // Machines with anti-ghosting firmware will show a gap with no column enabled
            // for a while during the transition while older firmwares might have two
            // columns enabled at the same time due to slow transistor deactivation. Both
            // cases are caught here.
            // See also https://emmytech.com/arcade/led_ghost_busting/index.html for details.
#if DEBUG_SERIAL
            sBadColCounter++;
            sLastBadCol = inColMask;
#endif
        }
        break;
    }

    // update the current column
    updateCol(inCol, inRowMask);

#if DEBUG_SERIAL
    if ((inCol != (sLastGoodCol+1)) && (inCol!=(sLastGoodCol-7)))
    {
        sBadColOrderCounter++;
    }
    sLastGoodCol = inCol;
#endif

    // remember the last column and row samples
    sLastColMask = inColMask;
    sLastRowMask = inRowMask;

    // how long did it take?
    sLastIntTime = (TCNT1 - startCnt);
    if ((sLastIntTime > sMaxIntTime) && (sLastIntTime < (1000 * 16)))
    {
        sMaxIntTime = sLastIntTime;
    }
}

//------------------------------------------------------------------------------
void loop()
{
    // The main loop is used for low priority serial communication only.
    // All the lamp matrix fun happens in the timer interrupt.

    // count the loops (used for debug output below)
    static uint32_t loopCounter = 0;
    loopCounter++;

#if DEBUG_SERIAL
    if ((loopCounter % 10) == 0)
    {
        Serial.println("REPLAY!");
        if (0)
        {
            Serial.println("PASS THROUGH!");
        }
        Serial.print("TTAG_INT ");
        Serial.println(TTAG_INT_A);
        Serial.print("INT dt max ");
        Serial.print(sMaxIntTime / 16);
        Serial.print("us last ");
        Serial.print(sLastIntTime / 16);
        Serial.println("us");
        Serial.print("Bad col: ");
        Serial.print(sBadColCounter);
        Serial.print(" col ");
        Serial.print(sLastBadCol);
        Serial.print(" ord ");
        Serial.print(sBadColOrderCounter);
        Serial.print(" last good: ");
        Serial.println(sLastGoodCol);
        Serial.print("CM ");
        Serial.print(sLastCurr);
        Serial.print(" max ");
        Serial.println(sMaxCurr);
        // data debugging
        debugInputs(sLastColMask, sLastRowMask);
        debugOutput(sLastOutColMask, sLastOutRowMask);
        // dump the full matrix
        for (uint32_t c=0; c<NUM_COL; c++)
        {
            Serial.print("C");
            Serial.print(c);
            Serial.print(" + ");
            for (uint32_t r=0; r<NUM_ROW; r++)
            {
                Serial.print(sMatrixState[c][r]);
                Serial.print(" ");
            }
            Serial.println("");
        }
    }
#endif

    // wait 500ms
    delay(500);
}

//------------------------------------------------------------------------------
inline void updateMx(uint16_t *pMx, bool on, uint16_t step)
{
    if (on)
    {
        // increase the stored brightness value
        if (*pMx < (65535 - step))
        {
            *pMx += step;
        }
        else
        {
            *pMx = 0xffff;
        }
    }
    else
    {
        // decrease the stored brightness value
        if (*pMx > step)
        {
            *pMx -= step;
        }
        else
        {
            *pMx = 0;
        }
    }
}

//------------------------------------------------------------------------------
void updateCol(uint32_t col, byte rowMask)
{
    // paranoia check
    if (col >= NUM_COL)
    {
        return;
    }
    
    // get a pointer to the matrix column
    uint16_t *pMx = &sMatrixState[col][0];
    const uint16_t *pkStep = &sGlowSteps[col][0];

    // update all row values
    for (uint32_t r=0; r<NUM_ROW; r++)
    {
        // update the matrix value
        updateMx(pMx, (rowMask & 0x01), *pkStep);

        // next row
        pMx++;
        pkStep++;
        rowMask >>= 1;
    }
}

//------------------------------------------------------------------------------
void driveLampMatrix()
{   
    // turn off everything briefly to avoid ghosting
    // the scope says this takes ~20us at 16MHz
    dataOutput(0x00, 0x00);

    // check which column we're currently updating
    uint32_t outCol = (sTtag % NUM_COL);

    // The original cycle is divided into ORIG_CYCLES column sub cycles.
    // These cycles are used to do PWM in order to adjust the lamp brightness.
    //
    // Illustration with ORIG_CYCLES==4 and four brightness steps B1-B4 and off (B0):
    //
    // * Lamp on
    //                      2ms 2ms ...
    // Orig col            1   2   3   4   5   6   7   8   1   2   3   4   5   6
    // afterglow col       12345678123456781234567812345678123456781234567812345
    // col cycle           1       2       3       4       1       2       3
    //
    // Brightness 1        *                               *
    // Brightness 2        *       *                       *       *
    // Brightness 3        *       *       *               *       *       *
    // Brightness 4        *       *       *       *       *       *       *

    uint32_t colCycle = ((sTtag / NUM_COL) % ORIG_CYCLES_A);

    // prepare the data
    // LSB is row/col 0, MSB is row/col 7
    byte colData = (1 << outCol);
    byte rowData = 0;
    uint16_t *pMx = &sMatrixState[outCol][0];
    byte *pMaxSubCycle = &sMaxSubcycle[outCol][0];
    for (uint32_t r=0; r<NUM_ROW; r++)
    {
        // make room for the next bit
        rowData >>= 1;
        
        // nothing to do if the matrix value is zero (off)
        if (*pMx)
        {
            uint16_t subCycle = (*pMx / (65536 / ORIG_CYCLES_A));

            // limit to the configured maximum brightness
            if (subCycle > *pMaxSubCycle)
            {
                subCycle = *pMaxSubCycle;
            }

            // Lamps are turned on when the value in the matrix is not zero
            // and when the value is high enough for the current sub cycle.
            if (subCycle >= colCycle)
            {
                rowData |= 0x80;
            }
        }
        pMx++;
        pMaxSubCycle++;
    }

    // output the data
    dataOutput(colData, rowData);
#if DEBUG_SERIAL
    sLastOutColMask = colData;
    sLastOutRowMask = rowData;
#endif
}

//------------------------------------------------------------------------------
void dataOutput(byte colData, byte rowData)
{
    // disable all columns (PD2-PD7, PB0-PB1) by pulling the ATMEGA pins high
    PORTD |= B11111100;
    PORTB |= B00000011;

    // pull LE and CLK low to start sending data
    PORTC &= B11111100;

    // clock out the shift register
    for (uint8_t bitMask=0x80; bitMask>0; bitMask>>=1)
    {
        PORTC &= B11111110; // CLK low
        if (rowData & bitMask)
        {
            PORTC |= B00000100; // set data bit
        }
        else
        {
            PORTC &= B11111011; // clear data bit
        }
        PORTC |= B00000001; // CLK high
    }

    PORTC &= B11111110; // CLK low

    // pull LE high to latch the data
    PORTC |= B00000010;

    // Enable by pulling OE low.
    // This is only done here to ensure that the LEDs are not turned on before
    // the columns are duty cycled.
    PORTC &= B11110111;

    // enable the column
    if (colData < 0B01000000)
    {
        PORTD &= ~(colData << 2);
    }
    else
    {
        PORTB &= ~(colData >> 6);
    }

    // pull LE low again
    PORTC &= B11111101;
}

//------------------------------------------------------------------------------
uint16_t testModeInput(void)
{
    // simulate the original column cycle
    byte col = ((sTtag / ORIG_CYCLES_A) % NUM_COL);
    byte colMask = (1 << col);

    // populate the row
    byte rowMask = 0;

#ifdef REPLAY_ENABLED
    // replay from table
    rowMask = replay(col);
#endif

    // invert the row mask as in the original input HIGH means off
    rowMask = ~rowMask;

    return ((colMask << 8) | rowMask);
}

//------------------------------------------------------------------------------
void applyCfg()
{
    // calculate the glow steps and maximum subcycles
    uint16_t *pGS = &sGlowSteps[0][0];
    uint8_t *pGlowDur = &sCfg.lampGlowDur[0][0];
    uint8_t *pBrightness = &sCfg.lampBrightness[0][0];
    byte *pMaxSubCycle = &sMaxSubcycle[0][0];
    for (byte c=0; c<NUM_COL; c++)
    {
        for (byte r=0; r<NUM_COL; r++)
        {
            // brightness step per lamp matrix update (assumes one update per original matrix step)
            uint32_t glowDur = (*pGlowDur * GLOWDUR_CFG_SCALE);
            *pGS++ = (glowDur > 0) ?
                ((uint16_t)(65535 / ((glowDur * 1000) / ORIG_INT)) * NUM_COL) : 0xffff;

            // translate maximum brightness into maximum lamp driving subcycle
            *pMaxSubCycle++ = (*pBrightness >> (8/ORIG_CYCLES_A-1));

            // next
            pGlowDur++;
            pBrightness++;
        }
    }
}

//------------------------------------------------------------------------------
void setDefaultCfg()
{
    // initialize configuration to default values
    memset(&sCfg, 0, sizeof(sCfg));
    sCfg.version = 0;
    uint8_t *pGlowDur = &sCfg.lampGlowDur[0][0];
    uint8_t *pBrightness = &sCfg.lampBrightness[0][0];
    for (byte c=0; c<NUM_COL; c++)
    {
        for (byte r=0; r<NUM_ROW; r++)
        {
            *pGlowDur++ = (DEFAULT_GLOWDUR / GLOWDUR_CFG_SCALE);
            *pBrightness++ = DEFAULT_BRIGHTNESS;
        }
    }
}

#if DEBUG_SERIAL
//------------------------------------------------------------------------------
void debugInputs(byte inColMask, byte inRowMask)
{
    // output the data
    char msg[64];
    sprintf(msg, "IN C 0x%02X R 0x%02X\n", inColMask, inRowMask);
    Serial.print(msg);
}

//------------------------------------------------------------------------------
void debugOutput(byte outColMask, byte outRowMask)
{
    // output the data
    char msg[64];
    sprintf(msg, "OUT C 0x%02X R 0x%02X\n", outColMask, outRowMask);
    Serial.print(msg);
}
#endif


#ifdef REPLAY_ENABLED
// Recording of attract mode from a Creature of the TOTAN pinball.
// Recorded from a modified pinmame version.
const AG_LAMP_SWITCH_t kLampReplay[] PROGMEM =
{
{5, 0, 0},   // +0.000s 0
{5, 7, 0}, {7, 1, 0}, {7, 2, 0}, {5, 1, 2}, {7, 0, 0}, {7, 3, 0}, {7, 4, 0}, {1, 7, 4}, {5, 2, 0}, {5, 3, 0}, 
{5, 4, 0}, {4, 6, 2}, {5, 5, 0}, {4, 4, 4}, {5, 6, 0}, {0, 5, 3}, {0, 6, 0}, {2, 6, 0}, {3, 4, 0}, {4, 5, 0}, 
{0, 4, 2}, {2, 5, 0}, {3, 3, 0}, {0, 2, 2}, {0, 3, 0}, {2, 4, 0}, {2, 7, 0}, {3, 0, 0}, {3, 5, 0}, {0, 1, 2}, 
{3, 2, 0}, {0, 0, 2}, {3, 1, 0}, {3, 6, 0}, {6, 5, 0}, {6, 6, 0}, {6, 7, 0}, {3, 7, 2}, {4, 7, 0}, {6, 4, 0}, 
{2, 0, 2}, {4, 0, 0}, {6, 3, 0}, {7, 6, 0}, {4, 1, 2}, {4, 2, 0}, {6, 1, 0}, {6, 2, 0}, {2, 1, 2}, {2, 2, 0},   // +0.500s 496
{2, 3, 0}, {4, 3, 0}, {6, 0, 0}, {1, 1, 2}, {1, 2, 0}, {1, 3, 0}, {1, 6, 0}, {1, 0, 2}, {1, 4, 0}, {1, 5, 0}, 
{5, 7, 3}, {7, 5, 0}, {5, 0, 2}, {5, 1, 0}, {7, 1, 0}, {7, 2, 0}, {1, 7, 2}, {5, 2, 0}, {7, 0, 0}, {7, 3, 0}, 
{7, 4, 0}, {4, 6, 2}, {5, 3, 0}, {5, 4, 0}, {5, 5, 0}, {4, 4, 2}, {5, 6, 0}, {0, 6, 2}, {3, 4, 0}, {4, 5, 0}, 
{0, 4, 2}, {0, 5, 0}, {2, 5, 0}, {2, 6, 0}, {3, 3, 0}, {0, 2, 2}, {0, 3, 0}, {2, 4, 0}, {2, 7, 0}, {3, 0, 2}, 
{3, 2, 0}, {3, 5, 0}, {0, 0, 2}, {0, 1, 0}, {6, 5, 0}, {6, 6, 0}, {6, 7, 0}, {3, 1, 2}, {3, 6, 0}, {4, 7, 0},   // +0.933s 928
{6, 4, 0}, {3, 7, 2}, {4, 0, 0}, {6, 3, 0}, {7, 6, 0}, {2, 0, 2}, {4, 1, 0}, {6, 1, 0}, {6, 2, 0}, {2, 1, 3}, 
{4, 2, 0}, {4, 3, 0}, {6, 0, 0}, {1, 1, 2}, {1, 2, 0}, {1, 3, 0}, {1, 6, 0}, {2, 2, 0}, {2, 3, 0}, {1, 0, 2}, 
{1, 4, 0}, {1, 5, 0}, {0, 2, 2}, {0, 3, 0}, {0, 4, 0}, {1, 7, 0}, {7, 5, 0}, {0, 0, 2}, {0, 1, 0}, {0, 5, 0}, 
{0, 6, 0}, {2, 4, 0}, {2, 5, 0}, {2, 6, 2}, {2, 7, 0}, {4, 4, 2}, {6, 5, 0}, {6, 6, 0}, {6, 7, 0}, {3, 0, 2}, 
{4, 5, 0}, {4, 6, 0}, {4, 7, 0}, {5, 6, 0}, {3, 1, 2}, {3, 2, 0}, {3, 3, 0}, {5, 5, 0}, {6, 4, 0}, {3, 4, 2},   // +1.333s 1328
{5, 4, 0}, {7, 4, 0}, {3, 5, 2}, {5, 3, 0}, {6, 3, 0}, {7, 3, 0}, {3, 6, 2}, {5, 2, 0}, {6, 2, 0}, {3, 7, 3}, 
{5, 1, 0}, {7, 0, 0}, {7, 6, 0}, {2, 0, 2}, {4, 0, 0}, {5, 0, 0}, {6, 1, 0}, {7, 1, 0}, {2, 1, 2}, {4, 1, 0}, 
{4, 2, 0}, {6, 0, 0}, {7, 2, 0}, {2, 2, 2}, {2, 3, 0}, {4, 3, 0}, {5, 7, 0}, {1, 1, 2}, {1, 2, 0}, {1, 6, 0}, 
{7, 5, 0}, {1, 0, 2}, {1, 3, 0}, {1, 4, 0}, {1, 5, 0}, {0, 2, 4}, {0, 3, 0}, {0, 4, 0}, {1, 7, 0}, {0, 0, 2}, 
{0, 1, 0}, {0, 5, 0}, {0, 6, 0}, {2, 4, 0}, {2, 5, 0}, {2, 6, 2}, {2, 7, 0}, {6, 5, 0}, {6, 6, 0}, {4, 4, 2},   // +1.767s 1760
{4, 5, 0}, {4, 7, 0}, {6, 7, 0}, {3, 0, 2}, {4, 6, 0}, {5, 5, 0}, {5, 6, 0}, {3, 1, 3}, {3, 2, 0}, {3, 3, 0}, 
{6, 4, 0}, {3, 4, 2}, {5, 4, 0}, {6, 3, 0}, {7, 4, 0}, {3, 5, 2}, {5, 3, 0}, {6, 2, 0}, {7, 3, 0}, {3, 6, 2}, 
{5, 2, 0}, {7, 0, 0}, {7, 6, 0}, {3, 7, 2}, {5, 0, 0}, {5, 1, 0}, {6, 1, 0}, {2, 0, 2}, {4, 0, 0}, {6, 0, 0}, 
{7, 1, 0}, {2, 1, 2}, {4, 1, 0}, {4, 2, 0}, {7, 2, 0}, {2, 2, 2}, {2, 3, 0}, {4, 3, 0}, {5, 7, 0}, {1, 1, 2}, 
{1, 2, 0}, {1, 3, 0}, {1, 4, 0}, {1, 6, 0}, {7, 5, 0}, {1, 0, 2}, {1, 5, 0}, {5, 0, 0}, {7, 2, 0}, {5, 1, 2},   // +2.167s 2160
{5, 7, 0}, {7, 0, 0}, {7, 1, 0}, {4, 0, 3}, {4, 1, 0}, {4, 2, 0}, {5, 2, 0}, {1, 6, 2}, {2, 1, 0}, {4, 3, 0}, 
{7, 3, 0}, {3, 4, 2}, {3, 5, 0}, {3, 7, 0}, {5, 3, 0}, {5, 4, 0}, {7, 5, 0}, {2, 0, 2}, {2, 2, 0}, {3, 6, 0}, 
{2, 3, 2}, {6, 0, 0}, {7, 4, 0}, {1, 1, 2}, {1, 2, 0}, {1, 3, 0}, {1, 4, 0}, {5, 5, 0}, {7, 6, 0}, {1, 5, 2}, 
{3, 3, 0}, {5, 6, 0}, {3, 0, 2}, {3, 1, 0}, {4, 5, 0}, {4, 6, 0}, {1, 0, 2}, {3, 2, 0}, {6, 1, 15}, {6, 3, 0}, 
{6, 2, 4}, {4, 4, 2}, {6, 4, 0}, {2, 6, 4}, {2, 7, 0}, {4, 7, 0}, {6, 7, 0}, {6, 5, 4}, {6, 6, 0}, {2, 4, 2},   // +2.967s 2960
{2, 5, 0}, {0, 6, 2}, {0, 4, 3}, {0, 5, 0}, {0, 0, 2}, {0, 1, 0}, {0, 2, 0}, {0, 3, 0}, {1, 7, 2}, {5, 0, 2}, 
{7, 0, 0}, {7, 1, 0}, {7, 2, 0}, {4, 0, 2}, {5, 1, 0}, {5, 2, 0}, {5, 7, 0}, {1, 6, 2}, {2, 1, 0}, {4, 1, 0}, 
{4, 2, 0}, {4, 3, 0}, {3, 7, 2}, {5, 3, 0}, {7, 3, 0}, {2, 0, 2}, {5, 4, 0}, {7, 5, 0}, {2, 2, 2}, {2, 3, 0}, 
{3, 4, 0}, {3, 5, 0}, {3, 6, 0}, {6, 0, 0}, {5, 5, 2}, {7, 4, 0}, {7, 6, 0}, {1, 1, 2}, {1, 2, 0}, {1, 3, 0}, 
{1, 4, 0}, {1, 5, 0}, {3, 3, 3}, {4, 5, 0}, {4, 6, 0}, {5, 6, 0}, {3, 0, 2}, {3, 1, 0}, {3, 2, 0}, {1, 0, 2},   // +3.467s 3472
{6, 1, 0}, {6, 3, 0}, {2, 6, 2}, {2, 7, 0}, {4, 4, 0}, {6, 2, 0}, {6, 4, 0}, {4, 7, 2}, {6, 7, 0}, {0, 6, 2}, 
{2, 4, 0}, {2, 5, 0}, {6, 5, 0}, {6, 6, 0}, {0, 2, 2}, {0, 3, 0}, {0, 4, 0}, {0, 5, 0}, {0, 0, 2}, {0, 1, 0}, 
{1, 0, 0}, {1, 4, 0}, {1, 5, 0}, {7, 5, 0}, {1, 1, 2}, {1, 2, 0}, {1, 3, 0}, {1, 6, 0}, {1, 7, 0}, {2, 1, 2}, 
{2, 2, 0}, {2, 3, 0}, {4, 3, 0}, {4, 1, 2}, {4, 2, 0}, {6, 0, 0}, {6, 1, 0}, {2, 0, 2}, {4, 0, 0}, {6, 2, 0}, 
{7, 6, 0}, {3, 7, 3}, {4, 7, 0}, {6, 3, 0}, {6, 4, 0}, {3, 1, 2}, {3, 6, 0}, {6, 5, 0}, {6, 6, 0}, {0, 0, 2},   // +3.867s 3872
{0, 1, 0}, {3, 2, 0}, {6, 7, 0}, {2, 4, 2}, {2, 7, 0}, {3, 0, 0}, {3, 5, 0}, {0, 2, 2}, {0, 3, 0}, {2, 5, 0}, 
{3, 3, 0}, {0, 4, 2}, {0, 5, 0}, {2, 6, 0}, {3, 4, 0}, {0, 6, 2}, {1, 7, 0}, {4, 4, 0}, {4, 5, 0}, {4, 6, 2}, 
{5, 5, 0}, {5, 6, 0}, {5, 2, 2}, {5, 3, 0}, {5, 4, 0}, {5, 1, 2}, {7, 0, 0}, {7, 3, 0}, {7, 4, 0}, {5, 0, 2}, 
{5, 7, 0}, {7, 1, 0}, {7, 2, 0}, {1, 0, 2}, {1, 4, 0}, {1, 5, 0}, {7, 5, 0}, {1, 1, 3}, {1, 2, 0}, {1, 3, 0}, 
{1, 6, 0}, {2, 1, 2}, {2, 2, 0}, {2, 3, 0}, {4, 2, 2}, {4, 3, 0}, {6, 0, 0}, {2, 0, 2}, {4, 1, 0}, {6, 1, 0},   // +4.300s 4304
{6, 2, 0}, {3, 7, 2}, {4, 0, 0}, {6, 3, 0}, {7, 6, 0}, {3, 1, 2}, {3, 6, 0}, {4, 7, 0}, {6, 4, 0}, {0, 0, 2}, 
{0, 1, 0}, {3, 2, 0}, {6, 5, 0}, {6, 6, 0}, {6, 7, 0}, {2, 4, 2}, {2, 7, 0}, {3, 0, 0}, {3, 5, 0}, {0, 2, 2}, 
{0, 3, 0}, {2, 5, 0}, {3, 3, 0}, {0, 4, 2}, {0, 5, 0}, {2, 6, 0}, {3, 4, 0}, {0, 6, 2}, {1, 7, 0}, {4, 4, 0}, 
{4, 5, 0}, {4, 6, 2}, {5, 5, 0}, {5, 6, 0}, {5, 2, 2}, {5, 3, 0}, {5, 4, 0}, {5, 1, 3}, {7, 3, 0}, {7, 4, 0}, 
{1, 0, 2}, {1, 5, 0}, {5, 0, 0}, {5, 7, 0}, {7, 0, 0}, {7, 1, 0}, {1, 1, 2}, {1, 2, 0}, {1, 3, 0}, {1, 4, 0},   // +4.700s 4704
{7, 2, 0}, {1, 6, 2}, {2, 2, 0}, {2, 3, 0}, {7, 5, 0}, {2, 1, 2}, {4, 2, 0}, {4, 3, 0}, {5, 7, 0}, {4, 0, 2}, 
{4, 1, 0}, {6, 0, 0}, {7, 2, 0}, {2, 0, 2}, {5, 0, 0}, {6, 1, 0}, {7, 1, 0}, {3, 7, 2}, {5, 1, 0}, {7, 0, 0}, 
{7, 6, 0}, {3, 5, 2}, {3, 6, 0}, {5, 2, 0}, {6, 2, 0}, {3, 4, 2}, {5, 3, 0}, {6, 3, 0}, {7, 3, 0}, {3, 2, 2}, 
{3, 3, 0}, {5, 4, 0}, {7, 4, 0}, {3, 0, 3}, {3, 1, 0}, {5, 5, 0}, {6, 4, 0}, {4, 5, 2}, {4, 6, 0}, {4, 7, 0}, 
{5, 6, 0}, {4, 4, 2}, {6, 5, 0}, {6, 6, 0}, {6, 7, 0}, {0, 6, 2}, {2, 4, 0}, {2, 5, 0}, {2, 6, 0}, {2, 7, 0},   // +5.100s 5104
{0, 0, 2}, {0, 1, 0}, {0, 5, 0}, {1, 7, 0}, {0, 2, 2}, {0, 3, 0}, {0, 4, 0}, {1, 0, 2}, {1, 3, 0}, {1, 4, 0}, 
{1, 5, 0}, {1, 1, 2}, {1, 2, 0}, {1, 6, 0}, {2, 2, 2}, {2, 3, 0}, {7, 5, 0}, {2, 1, 2}, {4, 2, 0}, {4, 3, 0}, 
{5, 7, 0}, {2, 0, 2}, {4, 0, 0}, {4, 1, 0}, {6, 0, 0}, {7, 2, 0}, {3, 7, 2}, {5, 0, 0}, {6, 1, 0}, {7, 1, 0}, 
{3, 6, 3}, {5, 1, 0}, {7, 0, 0}, {7, 6, 0}, {3, 5, 2}, {5, 2, 0}, {6, 2, 0}, {3, 4, 2}, {5, 3, 0}, {6, 3, 0}, 
{7, 3, 0}, {3, 1, 2}, {3, 2, 0}, {3, 3, 0}, {5, 4, 0}, {7, 4, 0}, {3, 0, 2}, {4, 6, 0}, {5, 5, 0}, {5, 6, 0},   // +5.533s 5536
{6, 4, 0}, {4, 4, 2}, {4, 5, 0}, {4, 7, 0}, {6, 5, 2}, {6, 6, 0}, {6, 7, 0}, {2, 4, 6}, {2, 5, 0}, {2, 6, 0}, 
{2, 7, 0}, {0, 0, 2}, {0, 1, 0}, {0, 5, 0}, {0, 6, 0}, {0, 2, 2}, {0, 3, 0}, {0, 4, 0}, {0, 0, 3}, {0, 1, 0}, 
{0, 2, 0}, {0, 3, 0}, {0, 4, 0}, {0, 5, 2}, {0, 6, 0}, {2, 4, 0}, {2, 5, 2}, {4, 7, 0}, {6, 5, 0}, {6, 6, 0}, 
{2, 6, 2}, {2, 7, 0}, {6, 4, 0}, {6, 7, 0}, {4, 4, 2}, {6, 0, 0}, {6, 1, 0}, {6, 2, 0}, {1, 0, 2}, {3, 1, 0}, 
{3, 2, 0}, {6, 3, 0}, {3, 0, 2}, {3, 4, 0}, {4, 5, 0}, {4, 6, 0}, {1, 2, 2}, {1, 3, 0}, {1, 4, 0}, {1, 5, 0},   // +6.033s 6032
{3, 3, 0}, {5, 6, 0}, {1, 1, 2}, {7, 6, 0}, {2, 2, 2}, {2, 3, 0}, {5, 5, 0}, {7, 4, 0}, {2, 0, 2}, {3, 5, 0}, 
{3, 6, 0}, {3, 7, 2}, {5, 4, 0}, {7, 5, 0}, {1, 6, 2}, {2, 1, 0}, {5, 3, 0}, {7, 3, 0}, {4, 0, 3}, {4, 1, 0}, 
{4, 2, 0}, {4, 3, 0}, {5, 1, 2}, {5, 2, 0}, {5, 7, 0}, {7, 0, 0}, {1, 7, 2}, {5, 0, 0}, {7, 1, 0}, {7, 2, 0}, 
{0, 0, 2}, {0, 1, 0}, {0, 2, 0}, {0, 3, 0}, {0, 4, 0}, {0, 5, 2}, {0, 6, 0}, {2, 4, 0}, {2, 5, 2}, {4, 7, 0}, 
{6, 5, 0}, {2, 6, 2}, {2, 7, 0}, {6, 6, 0}, {6, 7, 0}, {4, 4, 2}, {6, 0, 0}, {6, 4, 0}, {1, 0, 2}, {3, 1, 0},   // +6.500s 6496
{3, 2, 0}, {6, 1, 0}, {6, 2, 0}, {6, 3, 0}, {3, 0, 2}, {3, 4, 0}, {4, 5, 0}, {4, 6, 0}, {1, 4, 2}, {1, 5, 0}, 
{3, 3, 0}, {5, 6, 0}, {1, 1, 3}, {1, 2, 0}, {1, 3, 0}, {2, 2, 2}, {2, 3, 0}, {5, 5, 0}, {7, 4, 0}, {7, 6, 0}, 
{2, 0, 2}, {3, 5, 0}, {3, 6, 0}, {3, 7, 2}, {5, 4, 0}, {1, 6, 2}, {2, 1, 0}, {5, 3, 0}, {7, 3, 0}, {7, 5, 0}, 
{4, 0, 2}, {4, 1, 0}, {4, 2, 0}, {4, 3, 0}, {5, 1, 2}, {5, 2, 0}, {5, 7, 0}, {7, 0, 0}, {5, 0, 2}, {6, 5, 0}, 
{7, 1, 0}, {7, 2, 0}, {0, 6, 2}, {1, 1, 0}, {1, 6, 0}, {2, 3, 0}, {2, 4, 0}, {4, 7, 0}, {5, 6, 0}, {6, 3, 0},   // +6.867s 6864
{6, 4, 0}, {7, 3, 0}, {7, 4, 0}, {7, 6, 0}, {0, 5, 4}, {1, 2, 0}, {2, 0, 0}, {5, 5, 0}, {6, 2, 0}, {6, 6, 0}, 
{7, 0, 0}, {2, 5, 2}, {1, 3, 3}, {6, 7, 0}, {0, 4, 2}, {2, 1, 0}, {2, 6, 0}, {5, 4, 0}, {6, 1, 0}, {7, 1, 0}, 
{0, 3, 4}, {1, 4, 0}, {6, 0, 0}, {6, 5, 0}, {7, 2, 0}, {7, 5, 0}, {4, 4, 2}, {5, 3, 0}, {7, 3, 0}, {1, 5, 2}, 
{0, 2, 2}, {4, 5, 0}, {5, 2, 0}, {6, 3, 0}, {6, 4, 0}, {2, 2, 2}, {6, 6, 0}, {7, 0, 0}, {0, 1, 2}, {1, 0, 0}, 
{2, 3, 0}, {3, 0, 0}, {4, 6, 0}, {5, 1, 0}, {2, 0, 2}, {7, 4, 0}, {7, 6, 0}, {6, 2, 2}, {6, 7, 0}, {0, 0, 2},   // +7.367s 7360
{1, 1, 0}, {3, 1, 0}, {5, 0, 0}, {7, 1, 0}, {2, 1, 2}, {6, 1, 3}, {6, 7, 0}, {0, 6, 2}, {1, 2, 0}, {3, 2, 0}, 
{5, 6, 0}, {7, 3, 0}, {3, 3, 4}, {6, 6, 0}, {7, 0, 0}, {7, 5, 0}, {0, 5, 2}, {1, 3, 0}, {1, 7, 0}, {5, 5, 0}, 
{6, 0, 0}, {6, 5, 2}, {3, 4, 2}, {6, 3, 0}, {6, 4, 0}, {7, 1, 0}, {2, 2, 2}, {2, 3, 0}, {5, 4, 0}, {0, 4, 2}, 
{1, 4, 0}, {2, 0, 0}, {3, 5, 0}, {4, 6, 0}, {4, 7, 0}, {6, 7, 0}, {7, 4, 0}, {7, 6, 0}, {6, 2, 2}, {7, 3, 0}, 
{0, 3, 5}, {2, 1, 0}, {3, 6, 0}, {5, 3, 0}, {6, 1, 0}, {6, 6, 0}, {1, 5, 2}, {2, 7, 0}, {7, 0, 0}, {3, 7, 2},   // +7.867s 7872
{7, 5, 0}, {0, 2, 2}, {5, 2, 0}, {6, 0, 0}, {1, 0, 2}, {4, 0, 0}, {6, 5, 0}, {7, 1, 0}, {6, 4, 2}, {6, 3, 2}, 
{0, 1, 2}, {1, 1, 0}, {4, 1, 0}, {5, 1, 0}, {6, 5, 0}, {7, 3, 0}, {2, 2, 2}, {5, 7, 0}, {1, 2, 2}, {2, 0, 0}, 
{2, 3, 0}, {4, 6, 0}, {6, 2, 0}, {6, 6, 0}, {7, 4, 0}, {7, 6, 0}, {1, 3, 7}, {6, 7, 0}, {0, 0, 2}, {2, 1, 0}, 
{4, 2, 0}, {6, 1, 0}, {7, 0, 0}, {5, 0, 2}, {1, 4, 2}, {6, 5, 0}, {7, 1, 0}, {0, 0, 2}, {1, 6, 0}, {4, 3, 0}, 
{5, 6, 2}, {6, 0, 0}, {7, 5, 0}, {1, 5, 2}, {7, 3, 0}, {0, 1, 2}, {2, 4, 0}, {5, 5, 0}, {6, 6, 0}, {1, 0, 2},   // +8.467s 8464
{2, 3, 0}, {6, 3, 0}, {6, 4, 0}, {0, 2, 2}, {2, 2, 0}, {4, 7, 0}, {5, 4, 0}, {7, 4, 0}, {7, 6, 0}, {2, 0, 2}, 
{2, 5, 0}, {6, 2, 0}, {6, 7, 0}, {7, 0, 0}, {1, 1, 2}, {0, 3, 3}, {2, 1, 0}, {5, 3, 0}, {7, 2, 0}, {6, 1, 2}, 
{6, 7, 0}, {7, 1, 0}, {0, 4, 2}, {1, 2, 0}, {2, 6, 0}, {5, 2, 0}, {6, 0, 2}, {6, 6, 0}, {7, 5, 0}, {7, 3, 2}, 
{0, 5, 2}, {4, 4, 0}, {5, 1, 0}, {1, 3, 2}, {6, 3, 0}, {6, 4, 0}, {6, 5, 0}, {7, 0, 0}, {0, 6, 2}, {5, 0, 0}, 
{4, 5, 2}, {1, 4, 2}, {2, 0, 0}, {2, 2, 0}, {6, 2, 0}, {6, 7, 0}, {7, 1, 0}, {0, 0, 2}, {2, 3, 0}, {4, 6, 0},   // +8.933s 8928
{4, 7, 0}, {5, 6, 0}, {7, 4, 0}, {7, 6, 0}, {3, 0, 4}, {6, 6, 0}, {7, 3, 0}, {0, 1, 3}, {1, 5, 0}, {2, 1, 0}, 
{6, 1, 0}, {1, 7, 2}, {5, 5, 0}, {3, 1, 2}, {7, 0, 0}, {1, 0, 2}, {6, 0, 0}, {6, 5, 0}, {7, 5, 0}, {0, 2, 2}, 
{5, 4, 0}, {1, 1, 4}, {3, 2, 0}, {6, 3, 0}, {6, 4, 0}, {6, 5, 0}, {7, 1, 0}, {0, 3, 2}, {1, 2, 2}, {2, 0, 0}, 
{2, 2, 0}, {2, 3, 0}, {2, 7, 0}, {5, 3, 0}, {6, 6, 0}, {3, 3, 2}, {4, 7, 0}, {6, 2, 0}, {7, 3, 0}, {7, 4, 0}, 
{7, 6, 0}, {0, 4, 5}, {1, 3, 0}, {2, 1, 0}, {5, 2, 0}, {6, 1, 0}, {6, 7, 0}, {7, 0, 0}, {1, 4, 4}, {3, 4, 0},   // +9.467s 9472
{0, 5, 2}, {6, 0, 0}, {6, 5, 0}, {7, 1, 0}, {7, 5, 0}, {5, 1, 2}, {1, 5, 2}, {5, 7, 0}, {3, 5, 2}, {6, 3, 0}, 
{6, 4, 0}, {6, 6, 0}, {7, 3, 0}, {0, 6, 2}, {5, 0, 0}, {1, 0, 2}, {2, 0, 2}, {2, 2, 0}, {6, 2, 0}, {6, 7, 0}, 
{7, 0, 0}, {0, 6, 2}, {2, 3, 0}, {3, 6, 0}, {4, 6, 0}, {5, 6, 0}, {1, 1, 2}, {7, 4, 0}, {7, 6, 0}, {0, 5, 2}, 
{1, 6, 0}, {5, 5, 0}, {6, 1, 0}, {6, 7, 0}, {7, 1, 0}, {2, 1, 3}, {3, 7, 0}, {0, 4, 2}, {1, 2, 0}, {5, 4, 2}, 
{6, 6, 0}, {7, 3, 0}, {4, 0, 2}, {6, 0, 0}, {7, 5, 0}, {0, 3, 2}, {1, 3, 0}, {5, 3, 0}, {6, 5, 0}, {7, 0, 2},   // +10.000s 10000
{0, 2, 2}, {6, 3, 0}, {6, 4, 0}, {7, 2, 0}, {4, 1, 2}, {5, 2, 0}, {6, 7, 0}, {7, 1, 0}, {1, 4, 2}, {2, 0, 0}, 
{6, 2, 0}, {0, 1, 2}, {2, 2, 0}, {2, 3, 0}, {5, 1, 0}, {7, 4, 0}, {7, 6, 0}, {4, 6, 2}, {4, 7, 0}, {7, 3, 0}, 
{0, 0, 3}, {1, 5, 0}, {2, 1, 0}, {4, 2, 0}, {6, 1, 0}, {6, 6, 0}, {5, 0, 2}, {6, 0, 2}, {7, 5, 0}, {0, 6, 2}, 
{4, 3, 0}, {6, 5, 0}, {7, 0, 0}, {1, 0, 2}, {5, 6, 0}, {6, 3, 2}, {6, 4, 0}, {2, 4, 2}, {6, 5, 0}, {7, 1, 0}, 
{0, 5, 2}, {1, 1, 0}, {5, 5, 0}, {2, 0, 2}, {1, 2, 2}, {2, 5, 0}, {6, 2, 0}, {6, 6, 0}, {7, 3, 0}, {0, 4, 2},   // +10.533s 10528
{1, 7, 0}, {2, 2, 0}, {2, 3, 0}, {4, 6, 0}, {5, 4, 0}, {1, 3, 2}, {6, 7, 0}, {7, 4, 0}, {7, 6, 0}, {2, 1, 2}, 
{2, 6, 0}, {6, 1, 0}, {7, 0, 0}, {0, 3, 5}, {5, 3, 0}, {6, 5, 0}, {1, 4, 6}, {2, 7, 0}, {4, 4, 0}, {5, 2, 0}, 
{6, 0, 0}, {7, 1, 0}, {7, 5, 0}, {0, 2, 2}, {6, 6, 0}, {1, 5, 2}, {4, 5, 0}, {6, 3, 2}, {6, 4, 0}, {7, 3, 0}, 
{0, 1, 2}, {5, 1, 0}, {6, 7, 0}, {1, 0, 2}, {2, 0, 0}, {2, 3, 0}, {3, 0, 0}, {2, 2, 2}, {4, 7, 0}, {6, 2, 0}, 
{7, 0, 0}, {7, 4, 0}, {7, 6, 0}, {0, 0, 3}, {6, 7, 0}, {1, 1, 2}, {2, 1, 0}, {3, 1, 0}, {5, 0, 0}, {5, 7, 0},   // +11.033s 11040
{6, 1, 0}, {0, 0, 4}, {1, 2, 0}, {3, 2, 0}, {6, 6, 0}, {7, 1, 0}, {7, 5, 0}, {5, 6, 2}, {6, 0, 0}, {6, 5, 2}, 
{0, 1, 2}, {3, 3, 0}, {5, 5, 0}, {6, 4, 0}, {7, 3, 0}, {1, 3, 2}, {6, 3, 0}, {0, 2, 2}, {1, 6, 0}, {3, 4, 0}, 
{6, 7, 0}, {7, 0, 0}, {2, 0, 2}, {5, 4, 0}, {1, 4, 2}, {2, 2, 0}, {3, 5, 0}, {6, 2, 0}, {0, 3, 2}, {2, 3, 0}, 
{5, 3, 0}, {7, 1, 0}, {4, 6, 3}, {4, 7, 0}, {6, 6, 0}, {7, 4, 0}, {7, 6, 0}, {0, 4, 2}, {2, 1, 0}, {3, 6, 0}, 
{1, 5, 2}, {5, 2, 0}, {6, 1, 0}, {7, 3, 0}, {3, 7, 2}, {6, 5, 0}, {0, 5, 2}, {5, 1, 0}, {7, 2, 0}, {4, 0, 2},   // +11.567s 11568
{6, 0, 0}, {7, 0, 0}, {7, 5, 0}, {0, 6, 2}, {1, 0, 0}, {5, 0, 0}, {6, 5, 0}, {6, 4, 2}, {4, 1, 2}, {6, 3, 0}, 
{6, 6, 0}, {0, 0, 2}, {1, 1, 0}, {5, 6, 0}, {7, 1, 0}, {2, 0, 2}, {6, 2, 0}, {1, 2, 2}, {2, 3, 0}, {4, 2, 0}, 
{4, 7, 0}, {6, 7, 0}, {7, 3, 0}, {7, 4, 0}, {7, 6, 0}, {2, 1, 2}, {2, 2, 0}, {0, 1, 3}, {4, 3, 0}, {5, 5, 0}, 
{6, 1, 0}, {1, 3, 2}, {6, 5, 0}, {7, 0, 0}, {6, 0, 2}, {7, 5, 0}, {0, 2, 2}, {1, 4, 0}, {2, 4, 0}, {5, 4, 0}, 
{7, 1, 0}, {6, 6, 2}, {1, 5, 2}, {6, 3, 0}, {6, 4, 0}, {0, 3, 2}, {1, 7, 0}, {7, 3, 0}, {2, 5, 2}, {5, 3, 0},   // +12.067s 12064
{1, 0, 2}, {6, 7, 0}, {2, 0, 2}, {6, 2, 0}, {0, 4, 2}, {2, 2, 0}, {2, 6, 0}, {5, 2, 0}, {7, 0, 0}, {1, 1, 3}, 
{2, 1, 0}, {2, 3, 0}, {4, 6, 0}, {6, 7, 0}, {7, 4, 0}, {7, 6, 0}, {6, 1, 2}, {0, 5, 2}, {2, 7, 0}, {6, 6, 0}, 
{7, 1, 0}, {1, 2, 2}, {4, 4, 0}, {5, 1, 0}, {7, 5, 2}, {6, 0, 2}, {6, 5, 0}, {7, 3, 0}, {0, 6, 2}, {1, 3, 0}, 
{4, 5, 0}, {5, 0, 0}, {6, 4, 2}, {6, 3, 2}, {6, 7, 0}, {7, 0, 0}, {0, 6, 2}, {3, 0, 0}, {5, 6, 0}, {1, 4, 2}, 
{2, 0, 0}, {5, 7, 0}, {6, 2, 0}, {7, 1, 0}, {0, 5, 2}, {2, 3, 0}, {5, 5, 0}, {6, 6, 0}, {2, 1, 2}, {2, 2, 0},   // +12.600s 12592
{4, 6, 0}, {4, 7, 0}, {7, 4, 0}, {7, 6, 0}, {3, 1, 3}, {6, 1, 0}, {7, 3, 0}, {0, 4, 2}, {1, 5, 0}, {5, 4, 0}, 
{6, 5, 0}, {6, 0, 2}, {7, 5, 0}, {0, 3, 2}, {1, 6, 0}, {3, 2, 0}, {5, 3, 0}, {1, 0, 2}, {6, 4, 0}, {6, 5, 0}, 
{7, 0, 0}, {6, 3, 2}, {0, 2, 2}, {5, 2, 0}, {1, 1, 2}, {3, 3, 0}, {6, 6, 0}, {7, 1, 0}, {0, 1, 2}, {2, 0, 0}, 
{5, 1, 0}, {6, 2, 0}, {6, 7, 2}, {0, 0, 2}, {1, 2, 0}, {3, 4, 0}, {7, 3, 0}, {2, 1, 3}, {2, 2, 0}, {2, 3, 0}, 
{4, 6, 0}, {5, 0, 0}, {7, 2, 0}, {1, 3, 2}, {6, 1, 0}, {6, 5, 0}, {7, 0, 0}, {7, 4, 0}, {7, 6, 0}, {0, 6, 2},   // +13.067s 13072
{1, 4, 2}, {3, 5, 0}, {5, 6, 0}, {6, 0, 2}, {7, 1, 0}, {7, 5, 0}, {6, 6, 2}, {0, 5, 2}, {1, 5, 0}, {3, 6, 0}, 
{5, 5, 0}, {1, 0, 6}, {0, 4, 2}, {6, 3, 0}, {6, 4, 0}, {6, 7, 0}, {7, 3, 0}, {3, 7, 2}, {5, 4, 0}, {1, 1, 3}, 
{2, 0, 0}, {2, 2, 0}, {2, 3, 0}, {4, 7, 0}, {6, 2, 0}, {6, 7, 2}, {7, 4, 0}, {7, 6, 0}, {0, 3, 2}, {2, 1, 0}, 
{4, 0, 0}, {5, 3, 0}, {7, 0, 0}, {6, 1, 2}, {6, 6, 0}, {1, 2, 2}, {1, 7, 0}, {0, 2, 2}, {5, 2, 0}, {6, 0, 0}, 
{7, 1, 0}, {7, 5, 0}, {4, 1, 2}, {6, 5, 0}, {1, 3, 2}, {6, 3, 2}, {6, 4, 0}, {6, 7, 0}, {7, 3, 0}, {0, 1, 2},   // +13.700s 13696
{4, 2, 0}, {5, 1, 0}, {1, 4, 2}, {2, 0, 2}, {6, 2, 0}, {7, 0, 0}, {0, 0, 3}, {2, 2, 0}, {2, 7, 0}, {5, 0, 0}, 
{6, 6, 0}, {2, 3, 2}, {4, 3, 0}, {4, 6, 0}, {4, 7, 0}, {7, 1, 0}, {7, 4, 0}, {7, 6, 0}, {1, 5, 2}, {0, 0, 2}, 
{2, 1, 0}, {5, 6, 0}, {6, 1, 0}, {6, 5, 0}, {2, 4, 2}, {7, 3, 0}, {0, 1, 2}, {1, 0, 2}, {5, 5, 0}, {6, 0, 0}, 
{6, 5, 0}, {7, 5, 0}, {2, 5, 2}, {5, 7, 0}, {7, 0, 0}, {0, 2, 2}, {1, 1, 0}, {5, 4, 0}, {2, 6, 2}, {6, 3, 0}, 
{6, 4, 0}, {6, 6, 0}, {0, 3, 2}, {1, 2, 0}, {2, 0, 2}, {4, 4, 0}, {5, 3, 0}, {6, 7, 0}, {7, 1, 0}, {2, 3, 2},   // +14.200s 14192
{6, 2, 0}, {0, 4, 3}, {1, 3, 0}, {2, 2, 0}, {4, 7, 0}, {5, 2, 0}, {7, 4, 0}, {7, 6, 0}, {2, 1, 2}, {4, 5, 0}, 
{6, 1, 0}, {6, 5, 0}, {7, 3, 0}, {0, 5, 2}, {1, 6, 0}, {1, 4, 2}, {3, 0, 0}, {5, 1, 0}, {7, 0, 0}, {6, 0, 2}, 
{7, 5, 0}, {0, 6, 2}, {1, 5, 0}, {3, 1, 0}, {5, 0, 0}, {6, 6, 0}, {7, 1, 2}, {6, 3, 2}, {6, 4, 0}, {0, 0, 2}, 
{1, 0, 0}, {3, 2, 0}, {6, 7, 0}, {5, 6, 2}, {7, 3, 0}, {2, 0, 2}, {3, 3, 0}, {6, 2, 0}, {7, 2, 0}, {0, 1, 3}, 
{1, 1, 0}, {5, 5, 0}, {6, 7, 0}, {2, 2, 2}, {2, 3, 0}, {4, 6, 0}, {7, 0, 0}, {2, 1, 2}, {3, 4, 0}, {6, 1, 0},   // +14.667s 14672
{6, 6, 0}, {7, 4, 0}, {7, 6, 0}, {1, 2, 2}, {0, 2, 2}, {3, 5, 0}, {5, 4, 0}, {6, 5, 2}, {7, 1, 0}, {1, 3, 2}, 
{6, 0, 0}, {7, 5, 0}, {0, 3, 2}, {3, 6, 0}, {5, 3, 0}, {7, 3, 0}, {6, 7, 2}, {6, 3, 2}, {6, 4, 0}, {0, 4, 2}, 
{1, 4, 0}, {3, 7, 0}, {7, 0, 0}, {2, 0, 2}, {5, 2, 0}, {6, 2, 0}, {6, 6, 0}, {4, 0, 3}, {7, 1, 0}, {1, 5, 2}, 
{1, 7, 0}, {2, 3, 0}, {0, 5, 2}, {2, 1, 0}, {2, 2, 0}, {4, 6, 0}, {4, 7, 0}, {6, 1, 0}, {7, 4, 0}, {7, 6, 0}, 
{4, 1, 2}, {5, 1, 0}, {6, 5, 0}, {7, 3, 0}, {6, 0, 2}, {7, 5, 0}, {0, 6, 2}, {1, 0, 0}, {4, 2, 0}, {5, 0, 2},   // +15.200s 15200
{6, 5, 0}, {4, 3, 2}, {6, 3, 0}, {6, 4, 0}, {7, 0, 0}, {0, 6, 2}, {1, 1, 0}, {2, 7, 0}, {6, 6, 0}, {5, 6, 2}, 
{1, 2, 2}, {2, 0, 0}, {2, 4, 0}, {7, 1, 0}, {0, 5, 2}, {5, 5, 0}, {6, 2, 0}, {6, 7, 0}, {1, 3, 2}, {0, 4, 3}, 
{2, 2, 0}, {7, 3, 0}, {2, 1, 2}, {2, 3, 0}, {2, 5, 0}, {4, 6, 0}, {5, 4, 0}, {6, 1, 0}, {6, 5, 0}, {1, 4, 2}, 
{7, 4, 0}, {7, 6, 0}, {0, 3, 2}, {5, 3, 0}, {5, 7, 0}, {7, 0, 0}, {2, 6, 2}, {6, 6, 0}, {0, 2, 2}, {1, 5, 0}, 
{5, 2, 0}, {6, 0, 0}, {7, 1, 0}, {7, 5, 0}, {1, 0, 4}, {6, 3, 0}, {6, 4, 0}, {6, 7, 0}, {0, 1, 2}, {4, 4, 0},   // +15.700s 15696
{5, 1, 0}, {7, 3, 0}, {1, 1, 4}, {1, 6, 0}, {2, 0, 0}, {6, 2, 0}, {6, 7, 0}, {0, 0, 3}, {4, 5, 0}, {5, 0, 0}, 
{7, 0, 0}, {2, 1, 2}, {6, 1, 0}, {2, 2, 2}, {2, 3, 0}, {4, 7, 0}, {6, 6, 0}, {7, 4, 0}, {7, 6, 0}, {0, 6, 2}, 
{1, 2, 0}, {5, 6, 0}, {3, 0, 2}, {6, 0, 0}, {6, 5, 0}, {7, 1, 0}, {7, 5, 0}, {0, 5, 4}, {1, 3, 0}, {5, 5, 0}, 
{7, 2, 0}, {6, 3, 2}, {6, 4, 0}, {6, 7, 0}, {7, 3, 0}, {3, 1, 2}, {0, 4, 2}, {1, 4, 0}, {7, 0, 0}, {2, 0, 2}, 
{5, 4, 0}, {6, 2, 0}, {3, 2, 2}, {6, 6, 0}, {7, 1, 3}, {0, 3, 2}, {1, 5, 0}, {2, 1, 0}, {5, 3, 0}, {2, 2, 2},   // +16.267s 16272
{2, 3, 0}, {6, 1, 0}, {6, 5, 0}, {3, 3, 2}, {4, 6, 0}, {4, 7, 0}, {7, 3, 0}, {7, 4, 0}, {7, 6, 0}, {0, 2, 2}, 
{1, 0, 0}, {5, 2, 2}, {6, 0, 0}, {6, 5, 0}, {7, 5, 0}, {3, 4, 2}, {7, 0, 0}, {1, 1, 2}, {0, 1, 2}, {5, 1, 0}, 
{6, 3, 0}, {6, 4, 0}, {6, 6, 0}, {1, 2, 2}, {1, 7, 0}, {3, 5, 0}, {2, 0, 2}, {6, 7, 0}, {7, 1, 0}, {0, 0, 2}, 
{5, 0, 0}, {6, 2, 0}, {1, 3, 3}, {2, 1, 2}, {3, 6, 0}, {6, 1, 0}, {6, 5, 0}, {7, 3, 0}, {0, 0, 2}, {1, 4, 0}, 
{2, 3, 0}, {5, 6, 0}, {2, 2, 2}, {4, 7, 0}, {6, 0, 0}, {7, 0, 0}, {7, 4, 0}, {7, 6, 0}, {3, 7, 2}, {6, 6, 0},   // +16.733s 16736
{7, 5, 0}, {0, 1, 2}, {1, 5, 0}, {2, 7, 0}, {5, 5, 0}, {7, 1, 0}, {6, 3, 2}, {6, 4, 0}, {4, 0, 2}, {0, 2, 2}, 
{1, 0, 0}, {5, 4, 0}, {6, 7, 0}, {7, 3, 0}, {0, 3, 4}, {2, 0, 0}, {5, 3, 0}, {6, 2, 0}, {1, 1, 2}, {4, 1, 0}, 
{6, 7, 0}, {0, 4, 2}, {5, 7, 0}, {7, 0, 0}, {2, 1, 3}, {5, 2, 0}, {6, 1, 0}, {6, 6, 0}, {1, 2, 2}, {4, 2, 0}, 
{0, 5, 2}, {2, 2, 0}, {2, 3, 0}, {4, 6, 0}, {5, 1, 0}, {7, 1, 0}, {6, 5, 2}, {7, 4, 0}, {7, 6, 0}, {0, 6, 2}, 
{1, 3, 0}, {6, 0, 0}, {7, 5, 0}, {4, 3, 2}, {5, 0, 0}, {7, 3, 0}, {6, 7, 2}, {0, 0, 2}, {1, 6, 0}, {6, 3, 0},   // +17.267s 17264
{6, 4, 0}, {1, 4, 2}, {2, 4, 0}, {5, 6, 0}, {7, 0, 0}, {2, 0, 2}, {6, 2, 0}, {6, 6, 0}, {2, 5, 2}, {7, 1, 0}, 
{0, 1, 3}, {1, 5, 0}, {2, 1, 0}, {5, 5, 0}, {6, 1, 2}, {2, 6, 2}, {6, 5, 0}, {7, 3, 0}, {0, 2, 2}, {2, 3, 0}, 
{6, 0, 0}, {7, 5, 0}, {1, 0, 2}, {2, 2, 0}, {4, 4, 0}, {4, 6, 0}, {4, 7, 0}, {5, 4, 0}, {7, 2, 0}, {7, 4, 0}, 
{7, 6, 0}, {6, 5, 2}, {6, 3, 2}, {6, 4, 0}, {7, 0, 0}, {0, 3, 2}, {1, 1, 0}, {4, 5, 0}, {5, 3, 0}, {6, 6, 0}, 
{1, 2, 4}, {2, 0, 0}, {3, 0, 0}, {6, 2, 0}, {6, 7, 0}, {7, 1, 0}, {0, 4, 2}, {5, 2, 0}, {1, 3, 2}, {3, 1, 0},   // +17.767s 17760
{6, 5, 3}, {7, 3, 0}, {2, 1, 2}, {6, 1, 0}, {0, 5, 2}, {1, 4, 0}, {3, 2, 0}, {5, 1, 0}, {7, 0, 0}, {2, 2, 2}, 
{2, 3, 2}, {4, 6, 0}, {6, 0, 0}, {6, 6, 0}, {7, 4, 0}, {7, 5, 0}, {7, 6, 0}, {0, 6, 2}, {1, 5, 0}, {3, 3, 0}, 
{5, 0, 0}, {7, 1, 0}, {1, 7, 2}, {1, 0, 2}, {3, 4, 0}, {6, 3, 0}, {6, 4, 0}, {6, 7, 0}, {0, 6, 2}, {5, 6, 0}, 
{7, 3, 0}, {0, 5, 4}, {1, 1, 0}, {2, 0, 0}, {3, 5, 0}, {6, 2, 0}, {6, 7, 0}, {5, 5, 2}, {7, 0, 0}, {2, 1, 3}, 
{3, 6, 0}, {6, 1, 0}, {0, 4, 2}, {1, 2, 0}, {2, 7, 0}, {5, 4, 0}, {6, 6, 0}, {3, 7, 2}, {0, 3, 2}, {2, 3, 0},   // +18.300s 18304
{6, 0, 0}, {6, 5, 0}, {7, 1, 0}, {7, 5, 0}, {1, 3, 2}, {2, 2, 0}, {4, 7, 0}, {5, 3, 0}, {7, 4, 0}, {7, 6, 0}, 
{4, 0, 2}, {0, 2, 2}, {5, 2, 0}, {6, 3, 0}, {6, 4, 0}, {6, 7, 0}, {7, 3, 0}, {4, 1, 2}, {0, 1, 2}, {1, 4, 0}, 
{7, 0, 0}, {2, 0, 2}, {4, 2, 0}, {5, 1, 0}, {5, 7, 0}, {6, 2, 0}, {6, 6, 2}, {0, 0, 2}, {5, 0, 0}, {7, 1, 0}, 
{1, 5, 3}, {2, 1, 0}, {4, 3, 0}, {6, 1, 2}, {6, 5, 0}, {0, 6, 2}, {5, 6, 0}, {7, 3, 0}, {1, 0, 2}, {2, 4, 0}, 
{1, 6, 2}, {2, 2, 0}, {2, 3, 0}, {6, 0, 0}, {6, 5, 0}, {7, 5, 0}, {0, 5, 2}, {4, 6, 0}, {4, 7, 0}, {5, 5, 0},   // +18.767s 18768
{7, 0, 0}, {7, 4, 0}, {7, 6, 0}, {1, 1, 2}, {6, 6, 0}, {2, 5, 2}, {6, 3, 0}, {6, 4, 0}, {1, 2, 2}, {7, 1, 0}, 
{0, 4, 2}, {2, 0, 0}, {5, 4, 0}, {6, 2, 0}, {6, 7, 0}, {2, 6, 2}, {1, 3, 2}, {7, 2, 0}, {7, 3, 0}, {0, 3, 3}, 
{2, 1, 0}, {5, 3, 0}, {6, 1, 0}, {6, 5, 0}, {1, 4, 2}, {4, 4, 0}, {6, 0, 2}, {7, 0, 0}, {7, 5, 0}, {0, 2, 2}, 
{6, 6, 0}, {1, 5, 2}, {2, 3, 0}, {5, 2, 0}, {7, 1, 0}, {2, 2, 2}, {4, 5, 0}, {4, 7, 0}, {6, 3, 0}, {6, 4, 0}, 
{7, 4, 0}, {7, 6, 0}, {1, 0, 2}, {0, 1, 2}, {6, 7, 0}, {1, 1, 13}, {2, 0, 0}, {6, 7, 0}, {7, 3, 0}, {6, 2, 2},   // +19.467s 19472
{0, 0, 4}, {3, 0, 0}, {5, 1, 0}, {6, 6, 0}, {1, 2, 2}, {2, 1, 0}, {7, 0, 0}, {6, 5, 4}, {1, 3, 2}, {2, 2, 0}, 
{2, 3, 0}, {4, 6, 0}, {6, 1, 0}, {7, 1, 0}, {7, 4, 0}, {7, 5, 0}, {7, 6, 0}, {0, 0, 6}, {1, 7, 0}, {5, 0, 0}, 
{1, 4, 3}, {3, 1, 0}, {6, 7, 0}, {6, 4, 2}, {7, 3, 0}, {0, 1, 2}, {5, 6, 0}, {6, 0, 0}, {3, 2, 2}, {6, 6, 0}, 
{0, 2, 2}, {1, 5, 0}, {2, 0, 0}, {7, 0, 0}, {5, 5, 2}, {6, 3, 0}, {0, 3, 2}, {2, 7, 0}, {6, 5, 0}, {7, 1, 0}, 
{1, 0, 2}, {2, 1, 0}, {3, 3, 0}, {5, 4, 0}, {6, 2, 0}, {2, 2, 2}, {2, 3, 0}, {7, 4, 0}, {7, 6, 0}, {0, 4, 2},   // +20.100s 20096
{4, 6, 0}, {4, 7, 0}, {6, 5, 0}, {7, 3, 0}, {7, 5, 0}, {1, 1, 2}, {3, 4, 0}, {5, 3, 0}, {6, 1, 0}, {0, 5, 2}, 
{1, 2, 2}, {5, 2, 0}, {6, 0, 0}, {6, 4, 0}, {6, 6, 0}, {3, 5, 3}, {7, 0, 0}, {0, 6, 2}, {1, 3, 0}, {5, 7, 0}, 
{6, 7, 0}, {2, 0, 2}, {5, 1, 0}, {6, 3, 0}, {7, 1, 2}, {0, 0, 2}, {1, 4, 0}, {3, 6, 0}, {5, 0, 0}, {6, 5, 0}, 
{2, 1, 2}, {6, 2, 2}, {7, 3, 0}, {0, 1, 2}, {1, 5, 0}, {2, 2, 0}, {3, 7, 0}, {4, 6, 0}, {1, 6, 2}, {2, 3, 0}, 
{5, 6, 0}, {6, 6, 0}, {7, 4, 0}, {7, 6, 0}, {1, 0, 2}, {6, 1, 0}, {7, 0, 0}, {7, 5, 0}, {5, 5, 2}, {0, 2, 3},   // +20.600s 20608
{4, 0, 0}, {6, 7, 0}, {7, 1, 0}, {1, 1, 2}, {6, 4, 0}, {6, 0, 2}, {0, 3, 2}, {2, 0, 0}, {5, 4, 0}, {6, 7, 0}, 
{7, 3, 0}, {1, 2, 2}, {4, 1, 0}, {6, 3, 0}, {6, 6, 2}, {7, 2, 0}, {2, 1, 2}, {5, 3, 0}, {7, 0, 0}, {0, 4, 2}, 
{4, 2, 0}, {6, 2, 0}, {1, 3, 2}, {2, 3, 0}, {6, 5, 0}, {7, 4, 0}, {7, 5, 0}, {7, 6, 0}, {2, 2, 2}, {4, 7, 0}, 
{6, 1, 0}, {0, 5, 2}, {4, 3, 0}, {5, 2, 0}, {7, 1, 0}, {1, 4, 2}, {6, 4, 0}, {6, 7, 0}, {6, 0, 2}, {0, 6, 3}, 
{7, 3, 0}, {2, 4, 2}, {5, 1, 0}, {6, 6, 0}, {1, 5, 2}, {2, 0, 0}, {6, 3, 0}, {7, 0, 0}, {0, 6, 2}, {2, 5, 0},   // +21.133s 21136
{5, 0, 2}, {1, 0, 2}, {2, 1, 0}, {2, 6, 0}, {6, 2, 0}, {6, 5, 0}, {7, 1, 0}, {0, 5, 2}, {1, 7, 0}, {2, 2, 2}, 
{5, 6, 0}, {0, 4, 2}, {1, 1, 0}, {2, 3, 0}, {4, 4, 0}, {6, 5, 0}, {7, 3, 0}, {7, 4, 0}, {7, 6, 0}, {4, 6, 2}, 
{4, 7, 0}, {5, 5, 0}, {6, 1, 0}, {7, 5, 0}, {1, 2, 2}, {4, 5, 0}, {6, 6, 0}, {0, 3, 2}, {7, 0, 0}, {3, 0, 3}, 
{5, 4, 0}, {6, 0, 0}, {6, 4, 0}, {0, 2, 2}, {1, 3, 0}, {6, 7, 0}, {2, 0, 2}, {2, 7, 0}, {5, 3, 0}, {1, 4, 2}, 
{3, 1, 0}, {6, 3, 0}, {7, 1, 0}, {0, 1, 2}, {2, 1, 0}, {5, 2, 0}, {6, 5, 0}, {0, 0, 4}, {1, 5, 0}, {3, 2, 0},   // +21.633s 21632
{6, 2, 0}, {7, 3, 0}, {5, 1, 2}, {6, 6, 0}, {7, 5, 0}, {2, 2, 2}, {2, 3, 0}, {3, 3, 0}, {4, 7, 0}, {6, 1, 0}, 
{7, 0, 0}, {7, 4, 0}, {7, 6, 0}, {0, 6, 2}, {1, 0, 0}, {5, 0, 0}, {5, 7, 0}, {6, 0, 2}, {6, 4, 0}, {6, 7, 0}, 
{7, 1, 0}, {3, 4, 3}, {1, 1, 2}, {5, 6, 0}, {0, 5, 2}, {2, 0, 0}, {3, 5, 0}, {6, 3, 0}, {6, 7, 0}, {7, 3, 0}, 
{1, 2, 4}, {0, 4, 2}, {1, 6, 0}, {3, 6, 0}, {5, 5, 0}, {6, 6, 0}, {2, 1, 2}, {6, 2, 0}, {7, 0, 0}, {1, 3, 2}, 
{6, 5, 0}, {0, 3, 2}, {3, 7, 0}, {5, 4, 0}, {2, 2, 2}, {6, 1, 0}, {7, 1, 0}, {7, 5, 0}, {2, 3, 2}, {4, 0, 0},   // +22.133s 22128
{4, 6, 0}, {6, 7, 0}, {7, 4, 0}, {7, 6, 0}, {1, 4, 2}, {0, 2, 3}, {5, 3, 0}, {6, 4, 0}, {7, 3, 0}, {4, 1, 2}, 
{6, 0, 0}, {7, 2, 0}, {1, 5, 2}, {2, 0, 0}, {6, 6, 0}, {0, 1, 2}, {4, 2, 0}, {5, 2, 0}, {7, 0, 0}, {6, 3, 2}, 
{2, 1, 2}, {4, 3, 0}, {6, 5, 0}, {7, 1, 0}, {1, 0, 2}, {6, 2, 0}, {0, 0, 2}, {5, 1, 0}, {6, 5, 2}, {7, 3, 0}, 
{7, 5, 0}, {1, 1, 2}, {2, 3, 0}, {2, 4, 0}, {6, 1, 0}, {0, 0, 2}, {2, 2, 0}, {4, 6, 0}, {4, 7, 0}, {5, 0, 0}, 
{7, 4, 0}, {7, 6, 0}, {1, 2, 2}, {6, 0, 0}, {6, 4, 0}, {6, 6, 0}, {0, 1, 3}, {2, 5, 0}, {7, 0, 0}, {1, 3, 2},   // +22.633s 22640
{5, 6, 0}, {6, 7, 0}, {2, 0, 2}, {6, 3, 0}, {0, 2, 2}, {2, 6, 0}, {5, 5, 0}, {7, 1, 0}, {1, 4, 2}, {1, 7, 0}, 
{6, 5, 0}, {0, 3, 2}, {2, 1, 0}, {6, 2, 0}, {5, 4, 2}, {7, 3, 0}, {1, 5, 2}, {4, 4, 0}, {6, 6, 0}, {0, 4, 2}, 
{5, 3, 0}, {7, 0, 0}, {1, 0, 2}, {6, 1, 0}, {7, 5, 0}, {0, 5, 2}, {2, 2, 0}, {2, 3, 0}, {4, 6, 0}, {7, 4, 0}, 
{7, 6, 0}, {2, 7, 2}, {4, 5, 0}, {5, 2, 0}, {6, 7, 0}, {7, 1, 0}, {0, 6, 3}, {1, 1, 0}, {6, 0, 0}, {6, 4, 0}, 
{5, 1, 2}, {2, 0, 2}, {3, 0, 0}, {6, 7, 0}, {7, 3, 0}, {0, 0, 2}, {1, 2, 0}, {6, 3, 0}, {2, 1, 2}, {5, 0, 0},   // +23.133s 23136
{6, 6, 0}, {7, 0, 2}, {3, 1, 2}, {5, 7, 0}, {6, 2, 0}, {0, 1, 2}, {1, 3, 0}, {5, 6, 0}, {6, 5, 0}, {7, 5, 0}, 
{6, 1, 2}, {3, 2, 2}, {7, 1, 0}, {0, 2, 2}, {1, 4, 0}, {2, 2, 0}, {2, 3, 0}, {6, 4, 0}, {6, 7, 0}, {7, 4, 0}, 
{7, 6, 0}, {4, 7, 2}, {5, 5, 0}, {6, 0, 0}, {3, 3, 2}, {7, 3, 0}, {6, 6, 3}, {0, 3, 2}, {1, 5, 0}, {1, 6, 0}, 
{2, 0, 0}, {5, 4, 0}, {6, 3, 0}, {7, 0, 0}, {3, 4, 2}, {0, 4, 4}, {1, 0, 0}, {2, 1, 0}, {5, 3, 0}, {6, 2, 0}, 
{6, 5, 0}, {7, 1, 0}, {3, 5, 4}, {6, 5, 0}, {1, 1, 2}, {6, 1, 0}, {7, 3, 0}, {0, 5, 2}, {5, 2, 0}, {7, 5, 0},   // +23.700s 23696
{1, 2, 2}, {2, 2, 0}, {6, 6, 0}, {7, 2, 0}, {2, 3, 2}, {3, 6, 0}, {4, 6, 0}, {4, 7, 0}, {6, 4, 0}, {7, 0, 0}, 
{7, 4, 0}, {7, 6, 0}, {0, 6, 3}, {5, 1, 0}, {6, 0, 0}, {6, 7, 0}, {1, 3, 2}, {2, 0, 2}, {3, 7, 0}, {7, 1, 0}, 
{0, 6, 2}, {1, 4, 0}, {6, 3, 0}, {6, 5, 0}, {2, 1, 2}, {5, 0, 0}, {0, 5, 2}, {4, 0, 0}, {6, 2, 0}, {7, 3, 0}, 
{1, 5, 2}, {5, 6, 2}, {6, 6, 0}, {7, 5, 0}, {0, 4, 2}, {1, 0, 0}, {6, 1, 0}, {7, 0, 0}, {4, 1, 2}, {5, 5, 0}, 
{0, 3, 2}, {6, 0, 0}, {6, 4, 0}, {6, 7, 0}, {7, 1, 0}, {1, 1, 2}, {2, 2, 0}, {2, 3, 0}, {4, 7, 0}, {7, 4, 0},   // +24.167s 24160
{7, 6, 0}, {1, 7, 3}, {5, 4, 0}, {0, 2, 2}, {2, 0, 0}, {4, 2, 0}, {6, 3, 0}, {6, 7, 0}, {7, 3, 0}, {1, 2, 2}, 
{5, 3, 0}, {0, 1, 2}, {2, 1, 2}, {4, 3, 0}, {6, 6, 0}, {5, 2, 2}, {6, 2, 0}, {7, 0, 0}, {0, 0, 2}, {1, 3, 0}, 
{6, 5, 0}, {2, 4, 2}, {2, 7, 0}, {5, 1, 0}, {6, 1, 2}, {7, 1, 0}, {7, 5, 0}, {0, 6, 2}, {1, 4, 0}, {2, 5, 0}, 
{5, 0, 0}, {6, 7, 0}, {2, 2, 4}, {2, 3, 0}, {4, 6, 0}, {6, 4, 0}, {7, 3, 0}, {0, 5, 3}, {2, 6, 0}, {5, 6, 0}, 
{6, 0, 0}, {7, 4, 0}, {7, 6, 0}, {1, 5, 2}, {2, 0, 0}, {6, 6, 0}, {4, 4, 2}, {6, 3, 0}, {7, 0, 0}, {5, 7, 2},   // +24.700s 24704
{0, 4, 2}, {2, 1, 0}, {4, 5, 0}, {5, 5, 0}, {6, 5, 0}, {7, 1, 0}, {1, 0, 2}, {6, 2, 0}, {7, 5, 2}, {0, 3, 2}, 
{3, 0, 0}, {5, 4, 0}, {6, 1, 0}, {6, 5, 0}, {7, 3, 0}, {1, 1, 2}, {3, 1, 2}, {6, 4, 0}, {6, 6, 0}, {0, 2, 2}, 
{1, 2, 0}, {1, 6, 0}, {6, 0, 0}, {7, 0, 0}, {2, 3, 2}, {5, 3, 0}, {7, 4, 0}, {7, 6, 0}, {1, 3, 3}, {2, 2, 0}, 
{3, 2, 0}, {4, 6, 0}, {4, 7, 0}, {6, 7, 0}, {2, 0, 2}, {6, 3, 0}, {0, 1, 2}, {5, 2, 0}, {7, 1, 0}, {1, 4, 2}, 
{3, 3, 0}, {6, 5, 0}, {2, 1, 2}, {6, 2, 0}, {0, 0, 2}, {1, 5, 0}, {3, 4, 0}, {7, 2, 0}, {7, 3, 0}, {5, 1, 2},   // +25.200s 25200
{6, 6, 0}, {7, 0, 2}, {0, 0, 2}, {1, 0, 0}, {3, 5, 0}, {6, 1, 0}, {7, 5, 0}, {5, 0, 2}, {3, 6, 2}, {6, 7, 0}, 
{7, 1, 0}, {0, 1, 2}, {1, 1, 0}, {2, 2, 0}, {6, 0, 0}, {6, 4, 0}, {2, 3, 3}, {3, 7, 0}, {4, 6, 0}, {5, 6, 0}, 
{7, 4, 0}, {7, 6, 0}, {0, 2, 2}, {2, 0, 0}, {6, 7, 0}, {7, 3, 0}, {1, 2, 2}, {6, 3, 0}, {2, 1, 2}, {4, 0, 0}, 
{5, 5, 0}, {6, 6, 0}, {0, 3, 2}, {7, 0, 0}, {5, 4, 2}, {6, 2, 0}, {0, 4, 2}, {1, 3, 0}, {4, 1, 0}, {6, 5, 0}, 
{7, 5, 0}, {5, 3, 2}, {6, 1, 0}, {1, 7, 2}, {4, 2, 0}, {6, 7, 0}, {7, 1, 0}, {0, 5, 2}, {1, 4, 0}, {6, 0, 0},   // +25.700s 25696
{6, 4, 0}, {5, 2, 2}, {0, 6, 2}, {4, 3, 0}, {7, 3, 0}, {2, 3, 2}, {4, 7, 0}, {5, 1, 0}, {6, 3, 0}, {6, 6, 0}, 
{7, 4, 0}, {7, 6, 0}, {1, 5, 3}, {2, 0, 0}, {2, 2, 0}, {7, 0, 0}, {0, 0, 2}, {2, 4, 0}, {2, 7, 2}, {5, 0, 0}, 
{6, 5, 0}, {7, 1, 0}, {1, 0, 2}, {2, 1, 0}, {6, 2, 0}, {0, 1, 2}, {2, 5, 0}, {5, 6, 2}, {6, 5, 0}, {7, 3, 0}, 
{1, 1, 2}, {6, 1, 0}, {7, 5, 0}, {2, 6, 2}, {0, 2, 2}, {1, 2, 0}, {5, 5, 0}, {6, 6, 0}, {6, 4, 2}, {7, 0, 0}, 
{5, 7, 2}, {6, 0, 0}, {6, 7, 0}, {0, 3, 3}, {1, 3, 0}, {2, 0, 0}, {2, 2, 0}, {2, 3, 0}, {4, 4, 0}, {4, 6, 2},   // +26.233s 26240
{4, 7, 0}, {5, 4, 0}, {7, 1, 0}, {7, 4, 0}, {7, 6, 0}, {1, 4, 2}, {6, 3, 0}, {6, 5, 0}, {2, 1, 2}, {4, 5, 0}, 
{0, 4, 2}, {5, 3, 0}, {6, 2, 0}, {7, 3, 0}, {1, 5, 2}, {1, 6, 2}, {3, 0, 0}, {6, 6, 0}, {7, 5, 0}, {0, 5, 2}, 
{1, 0, 0}, {6, 1, 0}, {7, 0, 0}, {5, 2, 2}, {6, 0, 2}, {6, 4, 0}, {6, 7, 0}, {7, 1, 0}, {1, 1, 2}, {3, 1, 0}, 
{0, 6, 2}, {5, 1, 0}, {2, 0, 2}, {2, 3, 0}, {6, 3, 0}, {6, 7, 0}, {7, 3, 0}, {1, 2, 3}, {2, 2, 0}, {4, 7, 0}, 
{7, 4, 0}, {7, 6, 0}, {0, 6, 2}, {3, 2, 0}, {5, 0, 0}, {6, 6, 0}, {7, 2, 0}, {2, 1, 2}, {6, 2, 0}, {0, 5, 2},   // +26.733s 26736
{7, 0, 0}, {1, 3, 2}, {3, 3, 0}, {5, 6, 0}, {6, 5, 0}, {0, 4, 4}, {6, 1, 0}, {7, 1, 0}, {7, 5, 0}, {1, 4, 2}, 
{5, 5, 0}, {6, 7, 0}, {0, 3, 2}, {3, 4, 0}, {5, 4, 2}, {6, 0, 0}, {6, 4, 0}, {7, 3, 0}, {0, 2, 2}, {6, 6, 0}, 
{1, 5, 2}, {2, 0, 0}, {3, 5, 0}, {7, 0, 0}, {2, 2, 3}, {2, 3, 0}, {4, 6, 0}, {5, 3, 0}, {6, 3, 0}, {7, 4, 0}, 
{7, 6, 0}, {0, 1, 2}, {1, 0, 2}, {2, 1, 0}, {5, 2, 0}, {6, 5, 0}, {7, 1, 0}, {0, 0, 2}, {1, 7, 0}, {3, 6, 0}, 
{6, 2, 0}, {7, 5, 2}, {1, 1, 2}, {5, 1, 0}, {6, 1, 0}, {6, 5, 0}, {7, 3, 0}, {0, 6, 2}, {3, 7, 0}, {1, 2, 2},   // +27.267s 27264
{5, 0, 0}, {6, 4, 0}, {6, 6, 0}, {6, 0, 2}, {7, 0, 0}, {4, 0, 2}, {0, 5, 2}, {1, 3, 0}, {2, 0, 0}, {5, 6, 0}, 
{6, 7, 0}, {2, 7, 3}, {6, 3, 0}, {1, 4, 2}, {2, 2, 0}, {2, 3, 0}, {7, 1, 0}, {7, 4, 0}, {7, 6, 0}, {0, 4, 2}, 
{4, 1, 0}, {4, 6, 0}, {4, 7, 0}, {6, 5, 0}, {2, 1, 2}, {5, 5, 0}, {6, 2, 0}, {1, 5, 2}, {7, 3, 0}, {4, 2, 2}, 
{6, 6, 0}, {0, 3, 2}, {5, 4, 0}, {7, 0, 0}, {1, 0, 2}, {5, 7, 0}, {6, 1, 0}, {7, 5, 0}, {6, 7, 2}, {0, 2, 2}, 
{4, 3, 0}, {5, 3, 0}, {6, 4, 0}, {7, 1, 0}, {1, 1, 2}, {6, 0, 0}, {6, 7, 2}, {2, 0, 3}, {2, 4, 0}, {7, 3, 0},   // +27.800s 27808
{0, 1, 2}, {1, 2, 0}, {2, 2, 0}, {5, 2, 0}, {6, 3, 0}, {1, 6, 2}, {2, 1, 0}, {2, 3, 0}, {2, 5, 0}, {4, 6, 0}, 
{6, 6, 0}, {7, 4, 0}, {7, 6, 0}, {6, 2, 2}, {7, 0, 0}, {0, 0, 2}, {1, 3, 0}, {2, 6, 0}, {5, 1, 0}, {6, 5, 0}, 
{7, 5, 2}, {6, 1, 2}, {7, 1, 0}, {0, 0, 2}, {4, 4, 0}, {6, 7, 0}, {1, 4, 2}, {5, 0, 0}, {6, 0, 0}, {6, 4, 0}, 
{0, 1, 2}, {4, 5, 0}, {7, 3, 0}, {7, 2, 2}, {1, 5, 2}, {2, 0, 0}, {5, 6, 0}, {6, 3, 0}, {6, 6, 0}, {0, 2, 3}, 
{3, 0, 0}, {7, 0, 0}, {2, 3, 2}, {0, 3, 2}, {2, 2, 0}, {3, 1, 0}, {5, 5, 0}, {6, 5, 0}, {7, 1, 0}, {7, 4, 0},   // +28.267s 28272
{7, 6, 0}, {1, 0, 2}, {2, 1, 0}, {4, 7, 0}, {6, 2, 0}, {5, 4, 2}, {0, 4, 2}, {3, 2, 0}, {6, 5, 0}, {7, 3, 0}, 
{1, 1, 2}, {6, 1, 0}, {7, 5, 0}, {0, 5, 2}, {5, 3, 0}, {1, 2, 2}, {3, 3, 0}, {6, 6, 0}, {5, 2, 2}, {6, 4, 0}, 
{7, 0, 0}, {0, 6, 2}, {1, 3, 0}, {3, 4, 0}, {6, 0, 0}, {6, 7, 0}, {2, 0, 2}, {5, 1, 0}, {7, 1, 2}, {0, 0, 3}, 
{1, 4, 0}, {1, 7, 0}, {3, 5, 0}, {6, 3, 0}, {6, 5, 0}, {2, 1, 2}, {2, 2, 0}, {2, 3, 0}, {5, 0, 0}, {3, 6, 2}, 
{4, 6, 0}, {4, 7, 0}, {6, 2, 0}, {7, 3, 0}, {7, 4, 0}, {7, 6, 0}, {0, 1, 2}, {1, 5, 0}, {6, 6, 0}, {5, 6, 2},   // +28.767s 28768
{6, 1, 0}, {7, 0, 0}, {7, 5, 0}, {1, 0, 2}, {3, 7, 0}, {6, 4, 2}, {0, 2, 2}, {2, 7, 0}, {4, 0, 0}, {5, 5, 0}, 
{6, 0, 0}, {6, 7, 0}, {7, 1, 0}, {1, 1, 2}, {0, 3, 4}, {2, 0, 0}, {4, 1, 0}, {6, 3, 0}, {6, 7, 0}, {7, 3, 0}, 
{1, 2, 3}, {5, 4, 0}, {6, 6, 2}, {0, 4, 2}, {2, 1, 0}, {2, 3, 0}, {4, 2, 0}, {6, 2, 0}, {7, 0, 0}, {7, 4, 0}, 
{7, 6, 0}, {2, 2, 2}, {4, 7, 0}, {5, 3, 0}, {5, 7, 0}, {1, 3, 2}, {4, 3, 0}, {6, 5, 0}, {0, 5, 4}, {6, 1, 0}, 
{7, 1, 0}, {7, 5, 0}, {1, 4, 2}, {2, 4, 0}, {5, 2, 0}, {6, 7, 0}, {0, 6, 4}, {6, 0, 0}, {6, 4, 0}, {7, 3, 0},   // +29.300s 29296
{5, 1, 2}, {6, 6, 0}, {1, 5, 2}, {1, 6, 0}, {2, 0, 0}, {2, 5, 0}, {7, 0, 0}, {0, 6, 3}, {6, 3, 0}, {1, 0, 4}, 
{2, 1, 0}, {2, 2, 0}, {2, 6, 0}, {5, 0, 0}, {6, 5, 0}, {7, 1, 0}, {0, 5, 2}, {2, 3, 0}, {4, 6, 0}, {6, 2, 0}, 
{7, 4, 0}, {7, 6, 0}, {5, 6, 2}, {7, 5, 0}, {0, 4, 2}, {1, 1, 0}, {6, 1, 0}, {6, 5, 0}, {7, 3, 0}, {4, 4, 2}, 
{1, 2, 2}, {5, 5, 0}, {6, 0, 0}, {6, 4, 0}, {6, 6, 0}, {7, 2, 0}, {0, 3, 2}, {7, 0, 0}, {4, 5, 2}, {5, 4, 0}, 
{6, 7, 0}, {0, 2, 2}, {1, 3, 0}, {2, 0, 0}, {6, 3, 0}, {1, 4, 5}, {5, 3, 0}, {6, 5, 0}, {7, 1, 0}, {0, 1, 2},   // +29.833s 29840
{3, 0, 0}, {2, 1, 2}, {5, 2, 0}, {6, 2, 0}, {7, 3, 0}, {0, 0, 2}, {1, 5, 0}, {2, 2, 0}, {2, 3, 0}, {7, 4, 0}, 
{7, 6, 0}, {3, 1, 2}, {4, 6, 0}, {4, 7, 0}, {6, 6, 0}, {5, 1, 2}, {6, 1, 0}, {7, 0, 0}, {7, 5, 0}, {0, 6, 2}, 
{1, 0, 0}, {3, 2, 2}, {5, 0, 0}, {6, 7, 0}, {7, 1, 0}, {1, 1, 2}, {6, 4, 0}, {0, 5, 2}, {1, 7, 0}, {6, 0, 0}, 
{2, 0, 2}, {5, 6, 0}, {6, 7, 0}, {7, 3, 0}, {1, 2, 2}, {3, 3, 0}, {6, 3, 3}, {0, 4, 2}, {2, 1, 0}, {6, 6, 0}, 
{3, 4, 2}, {5, 5, 0}, {6, 2, 0}, {7, 0, 0}, {1, 3, 2}, {2, 2, 0}, {2, 3, 0}, {6, 5, 0}, {0, 3, 2}, {2, 7, 0},   // +30.333s 30336
{4, 6, 0}, {7, 4, 0}, {7, 5, 0}, {7, 6, 0}, {3, 5, 2}, {5, 4, 0}, {6, 1, 0}, {7, 1, 0}, {1, 4, 2}, {6, 7, 0}, 
{6, 0, 2}, {6, 4, 0}, {0, 2, 2}, {7, 3, 0}, {3, 6, 2}, {5, 3, 0}, {1, 5, 2}, {2, 0, 0}, {6, 3, 0}, {6, 6, 0}, 
{0, 1, 2}, {7, 0, 0}, {3, 7, 3}, {5, 2, 0}, {5, 7, 0}, {6, 5, 2}, {7, 1, 0}, {1, 0, 2}, {2, 1, 0}, {6, 2, 0}, 
{0, 0, 2}, {2, 3, 0}, {5, 1, 0}, {2, 2, 2}, {4, 0, 0}, {4, 7, 0}, {6, 5, 0}, {7, 3, 0}, {7, 4, 0}, {7, 6, 0}, 
{1, 1, 2}, {6, 1, 0}, {7, 5, 0}, {0, 0, 2}, {6, 6, 0}, {1, 2, 2}, {1, 6, 0}, {5, 0, 0}, {7, 0, 0}, {0, 1, 2},   // +30.867s 30864
{4, 1, 0}, {6, 0, 0}, {6, 4, 0}, {1, 3, 2}, {6, 7, 0}, {0, 2, 2}, {2, 0, 0}, {5, 6, 0}, {4, 2, 2}, {6, 3, 0}, 
{7, 1, 0}, {1, 4, 3}, {5, 5, 0}, {6, 5, 0}, {0, 3, 2}, {2, 1, 0}, {6, 2, 2}, {7, 3, 0}, {0, 4, 2}, {1, 5, 0}, 
{4, 3, 0}, {5, 4, 0}, {6, 6, 0}, {7, 2, 0}, {7, 5, 0}, {2, 2, 2}, {6, 1, 0}, {7, 0, 0}, {7, 4, 0}, {7, 6, 0}, 
{1, 0, 2}, {2, 3, 0}, {4, 6, 0}, {4, 7, 0}, {2, 4, 2}, {5, 3, 0}, {6, 4, 0}, {0, 5, 2}, {6, 0, 0}, {6, 7, 0}, 
{7, 1, 0}, {1, 1, 2}, {2, 5, 2}, {5, 2, 0}, {0, 6, 2}, {2, 0, 0}, {6, 3, 0}, {6, 7, 0}, {7, 3, 0}, {1, 2, 5},   // +31.400s 31408
{2, 6, 0}, {5, 1, 0}, {2, 1, 2}, {6, 6, 0}, {7, 0, 0}, {0, 0, 2}, {5, 0, 0}, {6, 2, 0}, {1, 3, 2}, {4, 4, 0}, 
{6, 5, 0}, {2, 3, 2}, {7, 4, 0}, {7, 6, 0}, {2, 2, 2}, {4, 5, 0}, {4, 7, 0}, {5, 6, 0}, {7, 1, 0}, {7, 5, 0}, 
{0, 1, 2}, {1, 7, 0}, {6, 1, 0}, {6, 7, 0}, {1, 4, 2}, {6, 4, 2}, {7, 3, 0}, {3, 0, 2}, {5, 5, 0}, {6, 0, 0}, 
{6, 6, 0}, {0, 2, 2}, {1, 5, 0}, {2, 0, 0}, {6, 3, 2}, {7, 0, 0}, {3, 1, 2}, {0, 3, 3}, {2, 1, 0}, {2, 7, 0}, 
{5, 4, 0}, {6, 5, 0}, {7, 1, 0}, {1, 0, 2}, {6, 2, 0}, {3, 2, 2}, {7, 5, 0}, {5, 3, 2}, {6, 5, 0}, {7, 3, 0},   // +31.933s 31936
{0, 4, 2}, {1, 1, 0}, {2, 2, 0}, {2, 3, 0}, {3, 3, 0}, {4, 6, 0}, {6, 1, 0}, {6, 4, 2}, {6, 6, 0}, {7, 4, 0}, 
{7, 6, 0}, {1, 2, 2}, {6, 0, 0}, {3, 4, 2}, {5, 2, 0}, {5, 7, 0}, {7, 0, 0}, {0, 5, 2}, {1, 3, 0}, {6, 7, 0}, 
{2, 0, 2}, {6, 3, 0}, {3, 5, 2}, {5, 1, 0}, {0, 6, 3}, {1, 4, 0}, {6, 5, 0}, {7, 1, 0}, {2, 1, 2}, {1, 5, 2}, 
{3, 6, 0}, {6, 2, 0}, {5, 0, 2}, {7, 3, 0}, {0, 6, 2}, {1, 6, 0}, {3, 7, 0}, {6, 6, 0}, {7, 5, 0}, {1, 0, 2}, 
{2, 2, 0}, {2, 3, 0}, {4, 6, 2}, {4, 7, 0}, {5, 6, 0}, {6, 1, 0}, {7, 0, 0}, {7, 4, 0}, {7, 6, 0}, {0, 5, 2},   // +32.433s 32432
{6, 7, 0}, {1, 1, 2}, {4, 0, 0}, {5, 5, 0}, {6, 4, 0}, {7, 1, 0}, {0, 4, 2}, {6, 0, 0}, {2, 0, 2}, {5, 4, 0}, 
{6, 7, 0}, {4, 1, 2}, {0, 3, 2}, {1, 2, 0}, {6, 3, 0}, {7, 2, 0}, {7, 3, 0}, {2, 1, 3}, {5, 3, 0}, {6, 6, 0}, 
{0, 2, 2}, {4, 2, 0}, {6, 2, 0}, {1, 3, 2}, {6, 5, 0}, {7, 5, 0}, {4, 3, 2}, {5, 2, 0}, {7, 0, 0}, {0, 1, 2}, 
{2, 2, 0}, {6, 1, 0}, {2, 3, 2}, {4, 6, 0}, {5, 1, 0}, {6, 4, 0}, {6, 7, 0}, {1, 4, 2}, {7, 1, 0}, {7, 4, 0}, 
{7, 6, 0}, {0, 0, 2}, {2, 4, 0}, {6, 0, 0}, {5, 0, 2}, {6, 6, 0}, {2, 0, 2}, {0, 6, 2}, {1, 5, 0}, {6, 3, 0},   // +32.967s 32960
{7, 3, 0}, {2, 5, 3}, {5, 6, 0}, {2, 1, 2}, {6, 5, 0}, {7, 0, 0}, {0, 5, 2}, {1, 0, 0}, {1, 7, 0}, {6, 2, 0}, 
{5, 5, 2}, {2, 6, 2}, {6, 5, 0}, {7, 1, 0}, {7, 5, 0}, {1, 1, 2}, {2, 3, 0}, {0, 4, 2}, {2, 2, 0}, {6, 1, 0}, 
{6, 6, 0}, {1, 2, 2}, {4, 4, 0}, {4, 7, 0}, {5, 4, 0}, {7, 3, 0}, {7, 4, 0}, {7, 6, 0}, {6, 4, 2}, {0, 3, 2}, 
{6, 0, 0}, {6, 7, 0}, {1, 3, 2}, {2, 0, 0}, {2, 7, 0}, {4, 5, 2}, {5, 3, 0}, {7, 0, 0}, {1, 4, 2}, {6, 5, 0}, 
{0, 2, 3}, {2, 1, 0}, {6, 3, 0}, {7, 1, 2}, {1, 5, 2}, {3, 0, 0}, {5, 2, 0}, {6, 2, 0}, {6, 6, 0}, {7, 5, 0},   // +33.500s 33504
{0, 1, 4}, {1, 0, 0}, {5, 7, 0}, {7, 3, 0}, {3, 1, 2}, {5, 1, 0}, {6, 1, 0}, {2, 2, 2}, {2, 3, 0}, {4, 6, 0}, 
{4, 7, 0}, {6, 4, 0}, {6, 7, 0}, {0, 0, 2}, {1, 1, 0}, {6, 0, 0}, {7, 0, 0}, {7, 4, 0}, {7, 6, 0}, {6, 7, 2}, 
{3, 2, 2}, {5, 0, 0}, {2, 0, 2}, {6, 3, 0}, {7, 1, 0}, {0, 0, 2}, {1, 2, 0}, {1, 6, 0}, {6, 6, 0}, {5, 6, 3}, 
{7, 3, 0}, {0, 1, 2}, {2, 1, 0}, {3, 3, 0}, {6, 2, 0}, {6, 5, 0}, {1, 3, 2}, {5, 5, 0}, {0, 2, 2}, {6, 7, 2}, 
{7, 0, 0}, {7, 5, 0}, {3, 4, 2}, {5, 4, 0}, {6, 1, 0}, {0, 3, 2}, {1, 4, 0}, {2, 3, 0}, {4, 7, 0}, {7, 2, 0},   // +34.033s 34032
{7, 4, 0}, {7, 6, 0}, {2, 2, 2}, {5, 3, 0}, {7, 1, 0}, {0, 4, 2}, {6, 0, 0}, {6, 4, 0}, {6, 6, 0}, {1, 5, 2}, 
{3, 5, 0}, {5, 2, 0}, {2, 0, 2}, {7, 3, 0}, {0, 5, 3}, {6, 5, 0}, {2, 1, 2}, {5, 1, 0}, {6, 3, 0}, {1, 0, 2}, 
{3, 6, 0}, {7, 0, 0}, {0, 6, 2}, {6, 2, 0}, {6, 5, 0}, {5, 0, 2}, {7, 1, 0}, {7, 5, 0}, {1, 1, 2}, {6, 1, 0}, 
{0, 0, 2}, {3, 7, 0}, {6, 6, 0}, {1, 2, 2}, {2, 2, 0}, {4, 6, 0}, {5, 6, 0}, {6, 4, 0}, {2, 3, 2}, {6, 0, 0}, 
{7, 3, 0}, {7, 4, 0}, {7, 6, 0}, {6, 7, 2}, {0, 1, 2}, {1, 3, 0}, {4, 0, 0}, {1, 7, 2}, {2, 0, 0}, {5, 5, 0},   // +34.567s 34560
{6, 3, 0}, {7, 0, 0}, {6, 5, 3}, {0, 2, 2}, {1, 4, 0}, {2, 1, 2}, {4, 1, 0}, {6, 2, 0}, {1, 5, 2}, {5, 4, 0}, 
{6, 6, 0}, {7, 1, 0}, {0, 3, 4}, {4, 2, 0}, {1, 0, 2}, {7, 3, 0}, {7, 5, 0}, {2, 7, 2}, {5, 3, 0}, {6, 1, 0}, 
{6, 7, 0}, {0, 4, 2}, {2, 2, 0}, {2, 3, 0}, {4, 6, 0}, {4, 7, 0}, {6, 4, 0}, {7, 0, 0}, {7, 4, 0}, {7, 6, 0}, 
{1, 1, 2}, {4, 3, 0}, {6, 0, 2}, {6, 7, 0}, {2, 0, 2}, {5, 2, 0}, {7, 1, 0}, {0, 5, 3}, {1, 2, 0}, {6, 6, 0}, 
{2, 1, 2}, {2, 4, 0}, {5, 7, 2}, {6, 3, 0}, {7, 3, 0}, {1, 3, 2}, {5, 1, 0}, {6, 5, 0}, {0, 6, 2}, {2, 5, 0},   // +35.133s 35136
{6, 2, 0}, {7, 5, 0}, {7, 0, 2}, {2, 6, 2}, {5, 0, 0}, {6, 7, 0}, {0, 6, 2}, {1, 4, 0}, {6, 1, 0}, {6, 4, 0}, 
{2, 2, 2}, {2, 3, 0}, {4, 6, 0}, {0, 5, 2}, {4, 4, 0}, {6, 0, 0}, {6, 6, 0}, {7, 1, 0}, {7, 4, 0}, {7, 6, 0}, 
{1, 5, 2}, {1, 6, 0}, {5, 6, 0}, {2, 0, 2}, {4, 5, 0}, {0, 4, 2}, {7, 3, 0}, {5, 5, 3}, {6, 3, 0}, {6, 5, 0}, 
{1, 0, 2}, {2, 1, 0}, {3, 0, 0}, {7, 0, 0}, {0, 3, 2}, {5, 4, 0}, {6, 2, 2}, {6, 5, 0}, {1, 1, 2}, {3, 1, 0}, 
{7, 1, 0}, {7, 2, 0}, {0, 2, 2}, {5, 3, 0}, {6, 6, 2}, {7, 5, 0}, {1, 2, 2}, {2, 3, 0}, {3, 2, 0}, {4, 7, 0},   // +35.667s 35664
{6, 1, 0}, {7, 3, 0}, {0, 1, 2}, {2, 2, 0}, {5, 2, 0}, {6, 4, 0}, {6, 7, 0}, {7, 4, 0}, {7, 6, 0}, {3, 3, 2}, 
{0, 0, 2}, {1, 3, 0}, {6, 0, 0}, {7, 0, 0}, {2, 0, 3}, {5, 1, 0}, {6, 5, 0}, {1, 4, 2}, {3, 4, 0}, {0, 6, 2}, 
{5, 0, 0}, {7, 1, 0}, {2, 1, 2}, {3, 5, 0}, {6, 3, 0}, {6, 6, 0}, {1, 5, 2}, {0, 5, 2}, {5, 6, 0}, {6, 2, 0}, 
{7, 3, 0}, {7, 5, 0}, {1, 0, 2}, {3, 6, 0}, {6, 7, 2}, {1, 7, 2}, {6, 1, 0}, {6, 4, 0}, {7, 0, 0}, {0, 4, 2}, 
{1, 1, 0}, {2, 2, 0}, {2, 3, 0}, {7, 4, 0}, {7, 6, 0}, {3, 7, 2}, {4, 6, 0}, {4, 7, 0}, {5, 5, 0}, {6, 0, 0},   // +36.133s 36128
{6, 7, 0}, {7, 1, 0}, {2, 0, 2}, {1, 2, 2}, {4, 0, 0}, {6, 6, 0}, {0, 3, 3}, {6, 3, 0}, {5, 4, 2}, {7, 3, 0}, 
{2, 1, 2}, {2, 7, 0}, {4, 1, 0}, {6, 5, 0}, {1, 3, 2}, {0, 2, 2}, {5, 3, 0}, {6, 2, 0}, {7, 0, 0}, {4, 2, 2}, 
{6, 7, 0}, {7, 5, 0}, {1, 4, 2}, {0, 1, 2}, {6, 1, 0}, {2, 3, 2}, {4, 3, 0}, {5, 2, 0}, {6, 6, 0}, {7, 1, 0}, 
{2, 2, 2}, {4, 7, 0}, {5, 7, 0}, {6, 4, 0}, {7, 4, 0}, {7, 6, 0}, {1, 5, 2}, {0, 0, 2}, {2, 0, 0}, {5, 1, 0}, 
{6, 0, 0}, {7, 3, 0}, {2, 4, 3}, {2, 1, 2}, {6, 3, 0}, {6, 5, 0}, {7, 0, 0}, {0, 0, 2}, {1, 0, 0}, {5, 0, 2},   // +36.733s 36736
{6, 5, 0}, {0, 1, 2}, {1, 6, 0}, {2, 5, 0}, {6, 2, 0}, {7, 1, 0}, {7, 5, 0}, {1, 1, 2}, {6, 6, 0}, {5, 6, 2}, 
{6, 1, 0}, {6, 4, 0}, {7, 3, 0}, {0, 2, 2}, {1, 2, 0}, {2, 6, 0}, {2, 2, 2}, {5, 5, 0}, {6, 7, 0}, {0, 3, 2}, 
{1, 3, 0}, {2, 3, 0}, {4, 6, 0}, {6, 0, 0}, {7, 4, 0}, {7, 6, 0}, {2, 0, 2}, {4, 4, 0}, {7, 0, 0}, {0, 4, 3}, 
{5, 4, 0}, {6, 5, 0}, {1, 4, 2}, {6, 3, 0}, {7, 2, 0}, {2, 1, 2}, {5, 3, 0}, {7, 1, 0}, {0, 5, 2}, {1, 5, 0}, 
{4, 5, 0}, {6, 6, 0}, {6, 2, 2}, {0, 6, 2}, {5, 2, 0}, {7, 3, 0}, {1, 0, 2}, {7, 5, 0}, {3, 0, 2}, {5, 1, 0},   // +37.233s 37232
{6, 7, 0}, {0, 0, 2}, {6, 1, 0}, {7, 0, 0}, {1, 1, 2}, {2, 3, 0}, {3, 1, 0}, {6, 4, 0}, {2, 2, 2}, {5, 0, 0}, 
{6, 7, 0}, {7, 1, 0}, {7, 4, 0}, {7, 6, 0}, {2, 0, 2}, {4, 6, 0}, {4, 7, 0}, {6, 0, 0}, {0, 1, 2}, {1, 2, 0}, 
{6, 6, 0}, {2, 1, 3}, {3, 2, 0}, {5, 6, 0}, {7, 3, 0}, {6, 3, 2}, {0, 2, 2}, {1, 3, 0}, {1, 7, 0}, {6, 5, 0}, 
{5, 5, 2}, {7, 5, 0}, {3, 3, 2}, {6, 2, 0}, {7, 0, 0}, {6, 7, 2}, {0, 3, 2}, {1, 4, 0}, {5, 4, 0}, {6, 1, 0}, 
{6, 4, 0}, {3, 4, 2}, {7, 1, 0}, {6, 0, 2}, {6, 6, 0}, {0, 4, 2}, {2, 2, 0}, {4, 6, 0}, {1, 5, 2}, {2, 0, 0},   // +37.767s 37760
{2, 3, 0}, {2, 7, 0}, {5, 3, 0}, {7, 3, 0}, {7, 4, 0}, {7, 6, 0}, {3, 5, 2}, {6, 3, 0}, {6, 5, 0}, {7, 0, 3}, 
{0, 5, 2}, {1, 0, 0}, {2, 1, 0}, {5, 2, 0}, {3, 6, 2}, {6, 5, 0}, {6, 2, 2}, {7, 1, 0}, {0, 6, 2}, {1, 1, 0}, 
{7, 5, 0}, {5, 1, 2}, {5, 7, 0}, {6, 6, 0}, {1, 2, 2}, {3, 7, 0}, {6, 1, 0}, {7, 3, 0}, {0, 6, 2}, {6, 4, 0}, 
{6, 7, 0}, {1, 3, 2}, {5, 0, 0}, {0, 5, 2}, {2, 3, 0}, {4, 0, 0}, {7, 0, 0}, {2, 0, 2}, {2, 2, 0}, {4, 7, 0}, 
{6, 0, 0}, {6, 5, 0}, {7, 4, 0}, {7, 6, 0}, {1, 4, 2}, {5, 6, 0}, {0, 4, 3}, {2, 1, 0}, {1, 6, 2}, {4, 1, 0},   // +38.267s 38272
{6, 3, 0}, {7, 1, 0}, {0, 3, 2}, {1, 5, 0}, {5, 5, 0}, {6, 6, 0}, {6, 2, 2}, {7, 5, 0}, {1, 0, 2}, {4, 2, 0}, 
{5, 4, 0}, {7, 3, 0}, {0, 2, 2}, {6, 7, 0}, {6, 1, 2}, {6, 4, 0}, {7, 0, 0}, {0, 1, 2}, {1, 1, 0}, {4, 3, 0}, 
{5, 3, 0}, {6, 0, 2}, {6, 7, 0}, {2, 0, 2}, {2, 2, 0}, {5, 2, 0}, {7, 1, 0}, {7, 2, 0}, {0, 0, 2}, {1, 2, 0}, 
{2, 3, 0}, {4, 6, 0}, {4, 7, 0}, {2, 4, 2}, {5, 1, 0}, {6, 3, 0}, {6, 6, 0}, {7, 4, 0}, {7, 6, 0}, {2, 1, 3}, 
{7, 3, 0}, {0, 6, 2}, {2, 5, 0}, {6, 5, 0}, {1, 3, 2}, {5, 0, 0}, {6, 2, 0}, {7, 0, 2}, {0, 5, 2}, {2, 6, 0},   // +38.767s 38768
{6, 7, 0}, {7, 5, 0}, {1, 4, 2}, {5, 6, 0}, {4, 4, 2}, {6, 1, 0}, {7, 1, 0}, {6, 4, 2}, {6, 6, 0}, {0, 4, 2}, 
{5, 5, 0}, {1, 5, 2}, {2, 0, 0}, {4, 5, 0}, {6, 0, 0}, {7, 3, 0}, {2, 2, 2}, {2, 3, 0}, {4, 7, 0}, {7, 4, 0}, 
{7, 6, 0}, {0, 3, 3}, {1, 7, 0}, {3, 0, 0}, {6, 5, 0}, {1, 0, 2}, {2, 1, 0}, {5, 4, 0}, {6, 3, 0}, {7, 0, 0}, 
{1, 1, 10}, {6, 5, 0}, {7, 1, 0}, {7, 5, 0}, {0, 2, 2}, {6, 2, 0}, {3, 1, 4}, {1, 2, 2}, {2, 7, 0}, {5, 3, 0}, 
{6, 1, 0}, {6, 4, 0}, {6, 6, 0}, {7, 3, 0}, {0, 1, 7}, {1, 3, 0}, {2, 2, 0}, {2, 3, 0}, {3, 2, 0}, {4, 6, 0},   // +39.433s 39440
{6, 0, 0}, {6, 7, 0}, {7, 4, 0}, {7, 6, 0}, {2, 0, 2}, {5, 2, 0}, {7, 0, 0}, {1, 4, 4}, {3, 3, 0}, {2, 1, 2}, 
{5, 1, 0}, {6, 3, 0}, {6, 5, 0}, {0, 0, 2}, {5, 7, 0}, {7, 1, 0}, {1, 5, 2}, {3, 4, 0}, {6, 2, 2}, {6, 6, 0}, 
{0, 0, 2}, {1, 0, 0}, {3, 5, 0}, {5, 0, 0}, {7, 3, 0}, {7, 5, 0}, {0, 1, 4}, {6, 1, 0}, {6, 7, 0}, {7, 0, 0}, 
{1, 1, 3}, {1, 6, 0}, {3, 6, 0}, {5, 6, 0}, {6, 4, 0}, {2, 2, 2}, {2, 3, 0}, {4, 6, 0}, {4, 7, 0}, {7, 1, 0}, 
{7, 4, 0}, {7, 6, 0}, {0, 2, 2}, {2, 0, 0}, {3, 7, 0}, {6, 7, 0}, {1, 2, 2}, {5, 5, 0}, {6, 0, 0}, {0, 3, 2},   // +39.933s 39936
{4, 0, 0}, {6, 6, 0}, {7, 3, 0}, {2, 1, 2}, {5, 4, 0}, {0, 4, 2}, {6, 3, 0}, {1, 3, 2}, {4, 1, 0}, {6, 5, 0}, 
{7, 5, 0}, {5, 3, 2}, {6, 2, 0}, {7, 0, 0}, {7, 2, 0}, {0, 5, 2}, {1, 4, 2}, {4, 2, 0}, {5, 2, 0}, {6, 4, 0}, 
{6, 7, 0}, {0, 6, 2}, {6, 1, 0}, {7, 1, 0}, {4, 3, 2}, {2, 2, 3}, {5, 1, 0}, {6, 0, 0}, {6, 6, 0}, {0, 0, 2}, 
{1, 5, 0}, {2, 0, 0}, {2, 3, 0}, {4, 6, 0}, {7, 3, 0}, {7, 4, 0}, {7, 6, 0}, {2, 4, 2}, {5, 0, 0}, {6, 3, 2}, 
{1, 0, 2}, {2, 1, 0}, {6, 5, 0}, {7, 0, 0}, {0, 1, 2}, {2, 5, 0}, {5, 6, 0}, {7, 1, 2}, {1, 1, 2}, {6, 2, 0},   // +40.467s 40464
{6, 5, 0}, {0, 2, 2}, {2, 6, 0}, {7, 5, 0}, {1, 2, 2}, {1, 7, 0}, {5, 5, 0}, {6, 6, 0}, {7, 3, 0}, {6, 1, 2}, 
{6, 4, 2}, {0, 3, 3}, {1, 3, 0}, {2, 3, 0}, {4, 4, 0}, {4, 7, 0}, {5, 4, 0}, {6, 7, 0}, {2, 0, 2}, {2, 2, 0}, 
{6, 0, 0}, {7, 0, 0}, {7, 4, 0}, {7, 6, 0}, {1, 4, 2}, {6, 5, 0}, {0, 4, 2}, {2, 1, 0}, {4, 5, 0}, {5, 3, 0}, 
{6, 3, 2}, {7, 1, 0}, {1, 5, 2}, {2, 7, 0}, {6, 6, 2}, {7, 5, 0}, {0, 5, 2}, {3, 0, 0}, {5, 2, 0}, {6, 2, 0}, 
{7, 3, 0}, {1, 0, 2}, {6, 1, 2}, {6, 4, 0}, {6, 7, 0}, {7, 0, 0}, {0, 6, 2}, {5, 1, 0}, {1, 1, 3}, {3, 1, 0},   // +41.000s 41008
{2, 0, 2}, {5, 7, 0}, {6, 0, 0}, {6, 7, 0}, {7, 1, 0}, {0, 6, 2}, {1, 2, 0}, {2, 2, 0}, {2, 3, 0}, {4, 6, 0}, 
{4, 7, 0}, {7, 4, 0}, {7, 6, 0}, {3, 2, 2}, {5, 0, 0}, {0, 5, 2}, {6, 3, 0}, {6, 6, 0}, {7, 3, 0}, {2, 1, 2}, 
{1, 3, 2}, {5, 6, 0}, {6, 5, 0}, {0, 4, 2}, {3, 3, 0}, {6, 2, 0}, {7, 0, 0}, {5, 5, 2}, {7, 5, 0}, {0, 3, 2}, 
{1, 4, 0}, {1, 6, 0}, {6, 7, 0}, {3, 4, 2}, {5, 4, 2}, {6, 1, 0}, {6, 4, 0}, {7, 1, 0}, {0, 2, 2}, {1, 5, 3}, 
{2, 0, 0}, {3, 5, 0}, {5, 3, 0}, {6, 6, 0}, {0, 1, 2}, {2, 2, 0}, {2, 3, 0}, {4, 7, 0}, {6, 0, 0}, {7, 3, 0},   // +41.467s 41472
{7, 4, 0}, {7, 6, 0}, {5, 2, 2}, {2, 1, 2}, {6, 5, 0}, {7, 0, 0}, {0, 0, 2}, {1, 0, 0}, {3, 6, 0}, {6, 3, 0}, 
{7, 2, 0}, {5, 1, 2}, {6, 2, 2}, {6, 5, 0}, {7, 1, 0}, {7, 5, 0}, {0, 6, 2}, {1, 1, 0}, {3, 7, 0}, {5, 0, 0}, 
{6, 4, 2}, {6, 6, 0}, {1, 2, 2}, {6, 1, 0}, {7, 3, 0}, {0, 5, 2}, {5, 6, 0}, {1, 3, 2}, {4, 0, 0}, {6, 0, 0}, 
{6, 7, 0}, {2, 0, 3}, {7, 0, 0}, {2, 2, 2}, {4, 6, 0}, {0, 4, 2}, {1, 4, 0}, {2, 3, 0}, {5, 5, 0}, {6, 3, 0}, 
{6, 5, 0}, {7, 4, 0}, {7, 6, 0}, {2, 1, 2}, {4, 1, 0}, {7, 1, 0}, {0, 3, 4}, {1, 5, 0}, {5, 4, 0}, {6, 6, 0},   // +42.000s 42000
{1, 0, 6}, {1, 7, 0}, {7, 3, 0}, {7, 5, 0}, {0, 2, 2}, {4, 2, 0}, {5, 3, 0}, {6, 2, 0}, {1, 1, 5}, {6, 7, 0}, 
{7, 0, 0}, {6, 4, 4}, {7, 1, 0}, {0, 1, 2}, {4, 3, 0}, {5, 2, 0}, {6, 1, 0}, {1, 2, 4}, {2, 0, 0}, {2, 2, 0}, 
{2, 3, 0}, {2, 7, 0}, {6, 7, 0}, {7, 4, 0}, {7, 6, 0}, {2, 4, 2}, {4, 6, 0}, {4, 7, 0}, {7, 3, 0}, {0, 0, 2}, 
{2, 1, 0}, {5, 1, 0}, {6, 0, 0}, {6, 6, 0}, {1, 3, 2}, {2, 5, 2}, {7, 0, 0}, {0, 0, 2}, {5, 0, 0}, {6, 3, 0}, 
{6, 5, 0}, {7, 5, 0}, {0, 1, 4}, {1, 4, 0}, {2, 6, 0}, {5, 7, 0}, {6, 2, 0}, {5, 6, 3}, {6, 4, 0}, {6, 7, 0},   // +42.633s 42640
{7, 1, 0}, {4, 4, 2}, {6, 1, 0}, {0, 2, 2}, {1, 5, 2}, {2, 0, 0}, {4, 5, 0}, {5, 5, 0}, {6, 6, 0}, {7, 3, 0}, 
{0, 3, 2}, {2, 2, 0}, {2, 3, 0}, {4, 6, 0}, {6, 0, 0}, {5, 4, 2}, {7, 0, 0}, {7, 4, 0}, {7, 6, 0}, {1, 0, 2}, 
{1, 6, 0}, {2, 1, 0}, {3, 0, 0}, {6, 5, 0}, {0, 4, 2}, {6, 3, 0}, {7, 1, 0}, {3, 1, 2}, {5, 3, 0}, {0, 5, 2}, 
{1, 1, 0}, {6, 5, 0}, {5, 2, 2}, {6, 2, 0}, {7, 3, 0}, {7, 5, 0}, {0, 6, 2}, {1, 2, 0}, {3, 2, 0}, {6, 6, 3}, 
{5, 1, 2}, {6, 4, 0}, {0, 0, 2}, {1, 3, 0}, {3, 3, 0}, {6, 1, 0}, {6, 7, 0}, {7, 0, 0}, {7, 2, 0}, {2, 0, 2},   // +43.133s 43136
{5, 0, 0}, {1, 4, 2}, {2, 2, 0}, {2, 3, 0}, {3, 4, 0}, {7, 4, 0}, {7, 6, 0}, {2, 1, 2}, {4, 7, 0}, {6, 0, 0}, 
{6, 5, 0}, {7, 1, 0}, {0, 1, 2}, {3, 5, 0}, {5, 6, 0}, {1, 5, 2}, {6, 3, 2}, {7, 3, 0}, {7, 5, 0}, {0, 2, 2}, 
{1, 0, 0}, {3, 6, 0}, {6, 6, 0}, {5, 5, 2}, {0, 3, 7}, {1, 1, 0}, {3, 7, 0}, {6, 2, 0}, {6, 4, 0}, {6, 7, 0}, 
{7, 0, 0}, {5, 4, 2}, {6, 1, 2}, {7, 1, 0}, {1, 2, 2}, {2, 2, 0}, {4, 0, 0}, {6, 7, 0}, {0, 4, 2}, {1, 7, 0}, 
{2, 0, 0}, {2, 3, 0}, {4, 6, 0}, {4, 7, 0}, {5, 3, 0}, {7, 4, 0}, {7, 6, 0}, {4, 1, 2}, {6, 0, 0}, {6, 6, 0},   // +43.633s 43632
{7, 3, 0}, {1, 3, 4}, {2, 1, 0}, {6, 3, 0}, {6, 5, 0}, {0, 5, 2}, {4, 2, 0}, {5, 2, 0}, {7, 0, 2}, {1, 4, 2}, 
{6, 7, 0}, {0, 6, 3}, {2, 7, 0}, {4, 3, 0}, {5, 1, 0}, {6, 2, 0}, {7, 5, 0}, {7, 1, 2}, {0, 6, 4}, {1, 5, 0}, 
{2, 4, 0}, {6, 1, 0}, {6, 4, 0}, {6, 6, 0}, {7, 3, 2}, {2, 3, 2}, {5, 0, 0}, {7, 4, 0}, {7, 6, 0}, {0, 5, 2}, 
{1, 0, 0}, {2, 0, 0}, {2, 2, 0}, {4, 7, 0}, {6, 5, 0}, {2, 5, 2}, {5, 7, 0}, {6, 0, 0}, {7, 0, 0}, {0, 4, 4}, 
{1, 1, 0}, {2, 1, 0}, {5, 6, 0}, {2, 6, 2}, {6, 3, 0}, {7, 1, 0}, {6, 5, 5}, {1, 2, 2}, {7, 5, 0}, {7, 3, 2},   // +44.300s 44304
{0, 3, 4}, {5, 5, 0}, {6, 2, 0}, {4, 4, 2}, {6, 6, 0}, {1, 3, 4}, {1, 6, 0}, {2, 2, 0}, {2, 3, 0}, {4, 6, 0}, 
{6, 1, 0}, {6, 4, 0}, {7, 0, 0}, {7, 4, 0}, {7, 6, 0}, {0, 2, 2}, {4, 5, 0}, {5, 4, 0}, {1, 4, 4}, {6, 7, 0}, 
{2, 0, 2}, {0, 1, 3}, {5, 3, 0}, {6, 0, 0}, {7, 1, 0}, {1, 5, 2}, {3, 0, 0}, {6, 5, 0}, {2, 1, 2}, {0, 0, 2}, 
{5, 2, 0}, {6, 3, 0}, {7, 2, 0}, {7, 3, 0}, {1, 0, 2}, {3, 1, 0}, {5, 1, 2}, {6, 6, 0}, {7, 0, 0}, {0, 6, 2}, 
{6, 2, 0}, {7, 5, 0}, {1, 1, 2}, {2, 2, 0}, {2, 3, 0}, {5, 0, 0}, {7, 1, 0}, {7, 4, 0}, {7, 6, 0}, {3, 2, 2},   // +44.900s 44896
{6, 7, 0}, {0, 5, 2}, {4, 6, 0}, {4, 7, 0}, {6, 4, 0}, {1, 2, 2}, {5, 6, 0}, {6, 1, 0}, {7, 3, 0}, {3, 3, 2}, 
{0, 4, 3}, {2, 0, 0}, {6, 7, 0}, {1, 3, 2}, {6, 0, 0}, {2, 1, 2}, {5, 5, 0}, {6, 6, 0}, {7, 0, 0}, {3, 4, 2}, 
{0, 3, 2}, {6, 3, 0}, {6, 5, 0}, {1, 4, 2}, {1, 7, 0}, {5, 4, 0}, {7, 1, 0}, {7, 5, 0}, {3, 5, 2}, {0, 2, 2}, 
{2, 2, 0}, {6, 2, 0}, {6, 7, 0}, {1, 5, 2}, {2, 3, 0}, {4, 6, 0}, {6, 4, 0}, {7, 3, 0}, {7, 4, 0}, {7, 6, 0}, 
{5, 3, 2}, {6, 1, 0}, {3, 6, 2}, {0, 1, 2}, {2, 0, 0}, {6, 6, 0}, {7, 0, 0}, {1, 0, 3}, {2, 7, 0}, {5, 2, 0},   // +45.433s 45440
{6, 0, 0}, {3, 7, 2}, {7, 1, 0}, {0, 0, 2}, {2, 1, 0}, {6, 5, 0}, {1, 1, 2}, {5, 1, 0}, {6, 3, 0}, {0, 0, 4}, 
{1, 2, 0}, {4, 0, 0}, {6, 5, 0}, {7, 3, 0}, {6, 2, 2}, {7, 5, 0}, {5, 0, 2}, {6, 6, 0}, {0, 1, 2}, {1, 3, 0}, 
{2, 2, 0}, {2, 3, 0}, {4, 1, 0}, {4, 7, 0}, {5, 7, 0}, {7, 0, 0}, {7, 4, 0}, {7, 6, 0}, {6, 1, 2}, {6, 4, 0}, 
{0, 2, 2}, {1, 4, 0}, {5, 6, 0}, {6, 7, 0}, {2, 0, 2}, {7, 1, 0}, {4, 2, 3}, {0, 3, 2}, {1, 5, 0}, {5, 5, 0}, 
{6, 0, 0}, {6, 5, 0}, {2, 1, 2}, {7, 3, 0}, {0, 4, 2}, {1, 0, 0}, {1, 6, 0}, {5, 4, 0}, {4, 3, 2}, {6, 3, 0},   // +45.967s 45968
{7, 5, 0}, {0, 5, 2}, {6, 6, 0}, {7, 0, 0}, {1, 1, 2}, {5, 3, 0}, {2, 4, 2}, {6, 2, 0}, {6, 4, 0}, {0, 6, 2}, 
{2, 2, 0}, {5, 2, 0}, {6, 7, 0}, {7, 1, 0}, {7, 4, 0}, {7, 6, 0}, {1, 2, 2}, {2, 3, 0}, {2, 5, 0}, {4, 6, 0}, 
{4, 7, 0}, {6, 1, 0}, {2, 0, 2}, {7, 2, 0}, {0, 0, 3}, {5, 1, 0}, {7, 3, 0}, {1, 3, 2}, {2, 6, 0}, {6, 0, 0}, 
{6, 7, 0}, {5, 0, 2}, {0, 1, 2}, {2, 1, 0}, {6, 6, 0}, {7, 0, 0}, {4, 4, 2}, {6, 3, 0}, {1, 4, 2}, {5, 6, 0}, 
{4, 5, 2}, {6, 5, 0}, {0, 2, 2}, {6, 2, 0}, {7, 1, 0}, {7, 5, 0}, {3, 0, 2}, {1, 5, 2}, {2, 2, 0}, {2, 3, 0},   // +46.500s 46496
{5, 5, 0}, {6, 7, 0}, {7, 4, 0}, {7, 6, 0}, {0, 3, 2}, {4, 7, 0}, {6, 4, 0}, {7, 3, 0}, {3, 1, 2}, {6, 1, 0}, 
{1, 0, 2}, {2, 0, 0}, {5, 4, 0}, {6, 6, 0}, {7, 0, 0}, {0, 4, 5}, {1, 7, 0}, {3, 2, 0}, {6, 0, 0}, {1, 1, 2}, 
{2, 1, 0}, {5, 3, 0}, {6, 5, 0}, {7, 1, 0}, {0, 5, 4}, {1, 2, 0}, {3, 3, 0}, {6, 3, 0}, {7, 5, 0}, {6, 5, 2}, 
{7, 3, 0}, {3, 4, 2}, {5, 2, 0}, {6, 2, 0}, {1, 3, 2}, {6, 4, 0}, {0, 6, 2}, {6, 6, 0}, {7, 0, 0}, {1, 4, 2}, 
{2, 2, 0}, {2, 3, 0}, {2, 7, 0}, {3, 5, 0}, {4, 6, 0}, {5, 1, 0}, {6, 1, 0}, {7, 4, 0}, {7, 6, 0}, {0, 6, 2},   // +46.967s 46960
{2, 0, 0}, {6, 7, 0}, {6, 0, 3}, {1, 5, 2}, {3, 6, 0}, {7, 1, 0}, {0, 5, 2}, {5, 0, 0}, {6, 5, 0}, {2, 1, 2}, 
{3, 7, 0}, {6, 3, 0}, {0, 4, 2}, {1, 0, 0}, {7, 3, 0}, {4, 0, 2}, {5, 6, 0}, {5, 7, 0}, {6, 6, 2}, {7, 0, 0}, 
{7, 5, 0}, {0, 3, 2}, {1, 1, 0}, {5, 5, 0}, {6, 2, 0}, {4, 1, 2}, {0, 2, 2}, {2, 3, 0}, {6, 4, 0}, {6, 7, 0}, 
{7, 1, 0}, {1, 2, 2}, {2, 2, 0}, {5, 4, 0}, {6, 1, 0}, {7, 4, 0}, {7, 6, 0}, {2, 0, 2}, {4, 2, 0}, {4, 6, 0}, 
{4, 7, 0}, {0, 1, 3}, {5, 3, 0}, {6, 7, 0}, {7, 3, 0}, {1, 3, 2}, {1, 6, 0}, {4, 3, 0}, {0, 0, 2}, {2, 1, 0},   // +47.467s 47472
{6, 0, 0}, {5, 2, 2}, {6, 6, 0}, {2, 4, 2}, {7, 0, 0}, {0, 6, 2}, {1, 4, 0}, {5, 1, 0}, {6, 3, 0}, {6, 5, 0}, 
{7, 5, 0}, {6, 2, 4}, {7, 1, 0}, {2, 5, 2}, {5, 0, 0}, {6, 4, 0}, {6, 7, 0}, {7, 2, 0}, {0, 5, 2}, {1, 5, 0}, 
{6, 1, 0}, {2, 2, 2}, {2, 3, 0}, {7, 3, 0}, {2, 0, 2}, {2, 6, 0}, {4, 6, 0}, {5, 6, 0}, {6, 6, 0}, {7, 4, 0}, 
{7, 6, 0}, {0, 4, 3}, {1, 0, 0}, {6, 0, 0}, {7, 0, 0}, {2, 1, 4}, {4, 4, 0}, {5, 5, 0}, {0, 3, 2}, {1, 1, 0}, 
{6, 3, 0}, {6, 5, 0}, {7, 1, 0}, {1, 2, 4}, {4, 5, 2}, {5, 4, 0}, {6, 2, 0}, {6, 5, 0}, {7, 3, 0}, {7, 5, 0},   // +48.000s 48000
{0, 2, 2}, {1, 3, 2}, {6, 6, 0}, {5, 3, 2}, {6, 4, 0}, {7, 0, 0}, {0, 1, 2}, {1, 7, 0}, {2, 3, 0}, {3, 0, 0}, 
{6, 1, 0}, {7, 4, 0}, {7, 6, 0}, {1, 4, 2}, {2, 0, 0}, {2, 2, 0}, {6, 7, 0}, {4, 7, 3}, {1, 5, 2}, {2, 1, 0}, 
{5, 2, 0}, {6, 0, 0}, {7, 1, 0}, {0, 0, 2}, {3, 1, 0}, {6, 5, 0}, {1, 0, 4}, {6, 3, 0}, {7, 3, 0}, {7, 5, 0}, 
{0, 0, 2}, {3, 2, 0}, {5, 1, 0}, {6, 6, 0}, {2, 7, 2}, {0, 1, 2}, {1, 1, 0}, {6, 2, 0}, {6, 4, 0}, {7, 0, 0}, 
{3, 3, 2}, {5, 0, 0}, {6, 1, 2}, {6, 7, 0}, {7, 1, 0}, {0, 2, 2}, {1, 2, 0}, {2, 0, 0}, {2, 2, 0}, {5, 6, 0},   // +48.533s 48528
{2, 3, 2}, {3, 4, 0}, {7, 4, 0}, {7, 6, 0}, {0, 3, 2}, {4, 6, 0}, {4, 7, 0}, {6, 0, 0}, {6, 7, 0}, {7, 3, 0}, 
{5, 5, 3}, {5, 7, 0}, {1, 3, 2}, {2, 1, 0}, {6, 3, 0}, {6, 6, 0}, {0, 4, 2}, {3, 5, 0}, {5, 4, 0}, {6, 5, 2}, 
{7, 0, 0}, {0, 5, 2}, {1, 4, 0}, {5, 3, 2}, {6, 2, 0}, {7, 5, 0}, {3, 6, 2}, {6, 7, 0}, {7, 1, 0}, {0, 6, 2}, 
{1, 6, 13}, {6, 4, 0}, {7, 4, 0}, {7, 6, 0}, {1, 5, 2}, {2, 2, 0}, {2, 3, 0}, {6, 6, 0}, {7, 3, 0}, {0, 0, 6}, 
{4, 7, 0}, {5, 2, 0}, {6, 1, 0}, {3, 7, 2}, {6, 5, 0}, {1, 0, 4}, {2, 0, 0}, {7, 0, 0}, {0, 1, 2}, {5, 1, 0},   // +49.333s 49328
{6, 0, 0}, {1, 1, 7}, {2, 1, 0}, {4, 0, 0}, {6, 5, 0}, {7, 2, 0}, {2, 2, 4}, {2, 3, 0}, {5, 0, 0}, {6, 3, 0}, 
{7, 1, 0}, {0, 2, 2}, {1, 2, 0}, {4, 1, 0}, {6, 6, 0}, {7, 4, 0}, {7, 5, 0}, {7, 6, 0}, {4, 6, 2}, {1, 3, 2}, 
{5, 6, 0}, {6, 2, 0}, {6, 7, 0}, {7, 3, 0}, {0, 3, 2}, {6, 4, 0}, {4, 2, 2}, {6, 1, 0}, {1, 4, 2}, {5, 5, 0}, 
{6, 5, 0}, {2, 0, 2}, {6, 0, 0}, {7, 0, 0}, {0, 4, 2}, {1, 5, 0}, {4, 3, 2}, {6, 6, 0}, {2, 1, 3}, {5, 4, 0}, 
{6, 3, 0}, {7, 1, 0}, {0, 5, 2}, {1, 0, 0}, {1, 7, 2}, {2, 3, 0}, {2, 4, 0}, {7, 4, 0}, {7, 6, 0}, {2, 2, 2},   // +49.933s 49936
{6, 7, 0}, {7, 3, 0}, {0, 6, 2}, {1, 1, 0}, {2, 5, 0}, {5, 3, 0}, {6, 2, 0}, {7, 5, 0}, {4, 6, 2}, {4, 7, 0}, 
{7, 0, 0}, {1, 2, 2}, {6, 7, 0}, {0, 6, 2}, {2, 6, 0}, {5, 2, 0}, {6, 1, 0}, {6, 4, 0}, {6, 6, 2}, {7, 1, 0}, 
{2, 0, 2}, {2, 7, 0}, {4, 4, 0}, {0, 5, 2}, {1, 3, 0}, {5, 1, 0}, {2, 1, 2}, {4, 5, 0}, {6, 0, 0}, {6, 5, 0}, 
{7, 3, 0}, {0, 4, 3}, {1, 4, 4}, {2, 2, 0}, {3, 0, 0}, {5, 0, 0}, {6, 3, 0}, {6, 7, 0}, {7, 0, 0}, {7, 5, 0}, 
{0, 3, 2}, {2, 3, 0}, {7, 4, 0}, {7, 6, 0}, {3, 1, 2}, {6, 2, 0}, {0, 2, 2}, {1, 5, 0}, {4, 6, 0}, {5, 6, 0},   // +50.400s 50400
{5, 7, 0}, {6, 4, 0}, {6, 6, 0}, {3, 2, 2}, {7, 1, 0}, {6, 1, 2}, {0, 1, 2}, {5, 5, 0}, {1, 0, 2}, {2, 0, 0}, 
{3, 3, 0}, {6, 0, 0}, {6, 5, 0}, {7, 3, 0}, {0, 0, 2}, {5, 4, 0}, {7, 0, 3}, {1, 1, 2}, {2, 1, 0}, {3, 4, 0}, 
{5, 3, 0}, {6, 5, 0}, {0, 6, 2}, {1, 6, 0}, {6, 3, 0}, {1, 2, 2}, {3, 5, 0}, {6, 6, 0}, {7, 1, 0}, {2, 2, 2}, 
{2, 3, 0}, {5, 2, 0}, {7, 4, 0}, {7, 6, 0}, {6, 2, 2}, {6, 7, 0}, {7, 5, 0}, {0, 5, 2}, {1, 3, 0}, {3, 6, 0}, 
{4, 7, 0}, {5, 1, 0}, {7, 3, 0}, {6, 4, 2}, {1, 4, 2}, {6, 1, 0}, {6, 5, 0}, {0, 4, 2}, {3, 7, 0}, {5, 0, 0},   // +50.900s 50896
{7, 0, 0}, {7, 2, 0}, {2, 0, 2}, {1, 5, 2}, {4, 0, 0}, {0, 3, 3}, {2, 1, 0}, {5, 6, 0}, {6, 0, 0}, {6, 6, 0}, 
{7, 1, 0}, {1, 0, 2}, {4, 1, 0}, {6, 3, 4}, {6, 7, 0}, {7, 5, 0}, {0, 0, 2}, {0, 1, 0}, {0, 2, 0}, {1, 0, 0}, 
{1, 1, 0}, {1, 2, 0}, {1, 3, 0}, {1, 4, 0}, {1, 5, 0}, {1, 7, 0}, {2, 0, 0}, {2, 1, 0}, {2, 3, 0}, {2, 4, 0}, 
{2, 5, 0}, {2, 6, 0}, {2, 7, 0}, {3, 0, 0}, {3, 1, 0}, {3, 2, 0}, {3, 3, 0}, {3, 4, 0}, {3, 5, 0}, {3, 6, 0}, 
{3, 7, 0}, {4, 0, 0}, {4, 1, 0}, {4, 4, 0}, {4, 5, 0}, {4, 6, 0}, {4, 7, 0}, {5, 1, 0}, {5, 2, 0}, {5, 3, 0},   // +51.133s 51136
{5, 4, 0}, {5, 5, 0}, {6, 3, 0}, {6, 4, 0}, {7, 1, 0}, {7, 2, 0}, {7, 4, 0}, {7, 5, 0}, {7, 6, 0}, {1, 7, 2}, 
{5, 1, 0}, {7, 0, 0}, {7, 3, 0}, {7, 4, 0}, {5, 2, 2}, {5, 3, 0}, {5, 4, 0}, {4, 6, 2}, {5, 5, 0}, {5, 6, 0}, 
{0, 5, 2}, {0, 6, 0}, {3, 4, 0}, {4, 4, 0}, {4, 5, 0}, {0, 4, 2}, {2, 5, 0}, {2, 6, 0}, {3, 3, 0}, {0, 2, 2}, 
{0, 3, 0}, {2, 4, 0}, {2, 7, 0}, {0, 1, 2}, {3, 0, 0}, {3, 2, 0}, {3, 5, 0}, {0, 0, 2}, {6, 7, 0}, {3, 1, 3}, 
{3, 6, 0}, {4, 7, 0}, {6, 5, 0}, {6, 6, 0}, {3, 7, 2}, {4, 0, 0}, {6, 3, 0}, {6, 4, 0}, {2, 0, 2}, {4, 1, 0},   // +51.500s 51504
{6, 2, 0}, {7, 6, 0}, {2, 1, 2}, {4, 2, 0}, {4, 3, 0}, {6, 0, 0}, {6, 1, 0}, {1, 1, 2}, {1, 2, 0}, {2, 2, 0}, 
{2, 3, 0}, {1, 0, 2}, {1, 3, 0}, {1, 4, 0}, {1, 6, 0}, {1, 5, 2}, {7, 5, 0}, {5, 0, 2}, {5, 7, 0}, {7, 2, 0}, 
{5, 1, 2}, {7, 0, 0}, {7, 1, 0}, {1, 7, 2}, {5, 2, 0}, {5, 3, 0}, {5, 4, 0}, {7, 3, 0}, {7, 4, 0}, {4, 6, 2}, 
{5, 5, 0}, {5, 6, 0}, {0, 6, 2}, {4, 4, 0}, {4, 5, 0}, {0, 4, 3}, {0, 5, 0}, {2, 6, 0}, {3, 4, 0}, {0, 2, 2}, 
{0, 3, 0}, {2, 5, 0}, {3, 3, 0}, {0, 1, 2}, {2, 4, 0}, {2, 7, 0}, {3, 0, 0}, {3, 5, 0}, {0, 0, 2}, {3, 2, 0},   // +51.933s 51936
{6, 7, 0}, {3, 1, 2}, {3, 6, 0}, {6, 5, 0}, {6, 6, 0}, {3, 7, 2}, {4, 7, 0}, {6, 3, 0}, {6, 4, 0}, {2, 0, 2}, 
{4, 0, 0}, {6, 2, 0}, {7, 6, 0}, {4, 1, 2}, {4, 2, 0}, {6, 0, 0}, {6, 1, 0}, {1, 1, 2}, {1, 2, 0}, {2, 1, 0}, 
{2, 2, 0}, {2, 3, 0}, {4, 3, 0}, {1, 0, 2}, {1, 3, 0}, {1, 4, 0}, {1, 6, 0}, {0, 2, 2}, {0, 3, 0}, {0, 4, 0}, 
{1, 5, 0}, {1, 7, 0}, {7, 5, 0}, {0, 1, 2}, {0, 5, 0}, {0, 0, 3}, {0, 6, 0}, {2, 4, 0}, {2, 5, 0}, {2, 6, 0}, 
{2, 7, 0}, {6, 5, 2}, {6, 6, 0}, {4, 4, 2}, {4, 5, 0}, {4, 7, 0}, {6, 7, 0}, {3, 0, 2}, {3, 1, 0}, {4, 6, 0},   // +52.333s 52336
{5, 5, 0}, {5, 6, 0}, {3, 2, 2}, {3, 3, 0}, {6, 4, 0}, {3, 4, 2}, {5, 4, 0}, {6, 3, 0}, {7, 4, 0}, {3, 5, 2}, 
{3, 6, 0}, {5, 3, 0}, {6, 2, 0}, {7, 3, 0}, {3, 7, 2}, {5, 2, 0}, {7, 0, 0}, {7, 6, 0}, {2, 0, 2}, {5, 0, 0}, 
{5, 1, 0}, {6, 1, 0}, {4, 0, 2}, {4, 1, 0}, {6, 0, 0}, {7, 1, 0}, {2, 1, 2}, {4, 2, 0}, {4, 3, 0}, {5, 7, 0}, 
{7, 2, 0}, {1, 6, 2}, {2, 2, 0}, {2, 3, 0}, {1, 1, 3}, {1, 2, 0}, {1, 3, 0}, {1, 4, 0}, {7, 5, 0}, {1, 0, 2}, 
{1, 5, 0}, {0, 2, 2}, {0, 3, 0}, {0, 4, 0}, {1, 7, 0}, {0, 0, 2}, {0, 1, 0}, {0, 5, 0}, {0, 6, 0}, {2, 4, 2},   // +52.767s 52768
{2, 5, 0}, {2, 6, 0}, {2, 7, 0}, {4, 4, 2}, {6, 5, 0}, {6, 6, 0}, {6, 7, 0}, {4, 5, 2}, {4, 6, 0}, {4, 7, 0}, 
{5, 6, 0}, {3, 0, 2}, {3, 1, 0}, {5, 5, 0}, {6, 4, 0}, {3, 2, 2}, {3, 3, 0}, {5, 4, 0}, {7, 4, 0}, {3, 4, 2}, 
{5, 3, 0}, {6, 3, 0}, {7, 3, 0}, {3, 5, 2}, {3, 6, 0}, {5, 2, 0}, {6, 2, 0}, {3, 7, 2}, {5, 1, 0}, {7, 0, 0}, 
{7, 6, 0}, {2, 0, 3}, {5, 0, 0}, {6, 1, 0}, {7, 1, 0}, {4, 0, 2}, {4, 1, 0}, {6, 0, 0}, {7, 2, 0}, {2, 1, 2}, 
{4, 2, 0}, {4, 3, 0}, {5, 7, 0}, {1, 1, 2}, {1, 2, 0}, {1, 6, 0}, {2, 2, 0}, {2, 3, 0}, {7, 5, 0}, {1, 0, 2},   // +53.167s 53168
{1, 3, 0}, {1, 4, 0}, {1, 0, 2}, {1, 3, 0}, {1, 4, 0}, {1, 6, 0}, {7, 5, 0}, {1, 1, 2}, {1, 2, 0}, {2, 2, 0}, 
{2, 3, 0}, {2, 1, 2}, {4, 3, 0}, {2, 0, 2}, {4, 1, 0}, {4, 2, 0}, {6, 0, 0}, {6, 1, 0}, {3, 7, 2}, {4, 0, 0}, 
{6, 2, 0}, {7, 6, 0}, {3, 1, 2}, {3, 6, 0}, {4, 7, 0}, {6, 3, 0}, {6, 4, 0}, {0, 0, 2}, {6, 5, 0}, {6, 6, 0}, 
{0, 1, 3}, {3, 0, 0}, {3, 2, 0}, {3, 5, 0}, {6, 7, 0}, {0, 2, 2}, {0, 3, 0}, {2, 4, 0}, {2, 7, 0}, {0, 4, 2}, 
{2, 5, 0}, {2, 6, 0}, {3, 3, 0}, {0, 5, 2}, {0, 6, 0}, {3, 4, 0}, {4, 5, 0}, {1, 7, 2}, {4, 4, 0}, {5, 6, 0},   // +53.567s 53568
{4, 6, 2}, {5, 3, 0}, {5, 4, 0}, {5, 5, 0}, {5, 2, 2}, {5, 0, 2}, {5, 1, 0}, {7, 0, 0}, {7, 3, 0}, {7, 4, 0}, 
{5, 7, 2}, {7, 1, 0}, {7, 2, 0}, {1, 0, 2}, {1, 4, 0}, {1, 5, 0}, {7, 5, 0}, {1, 1, 2}, {1, 2, 0}, {1, 3, 0}, 
{1, 6, 0}, {2, 1, 2}, {2, 2, 0}, {2, 3, 0}, {4, 3, 0}, {4, 1, 3}, {4, 2, 0}, {6, 0, 0}, {6, 1, 0}, {2, 0, 2}, 
{4, 0, 0}, {6, 2, 0}, {7, 6, 0}, {3, 7, 2}, {4, 7, 0}, {6, 3, 0}, {6, 4, 0}, {0, 0, 2}, {3, 1, 0}, {3, 6, 0}, 
{6, 5, 0}, {6, 6, 0}, {0, 1, 2}, {3, 2, 0}, {6, 7, 0}, {0, 2, 2}, {0, 3, 0}, {2, 4, 0}, {2, 7, 0}, {3, 0, 0},   // +54.000s 54000
{3, 5, 0}, {0, 4, 2}, {2, 5, 0}, {3, 3, 0}, {0, 5, 2}, {0, 6, 0}, {2, 6, 0}, {3, 4, 0}, {1, 7, 2}, {4, 4, 0}, 
{4, 5, 0}, {4, 6, 2}, {5, 5, 0}, {5, 6, 0}, {5, 2, 2}, {5, 3, 0}, {5, 4, 0}, {5, 1, 2}, {7, 0, 0}, {7, 3, 0}, 
{7, 4, 0}, {1, 0, 3}, {1, 3, 0}, {1, 4, 0}, {1, 5, 0}, {5, 0, 0}, {5, 7, 0}, {7, 1, 0}, {7, 2, 0}, {1, 1, 2}, 
{1, 2, 0}, {1, 6, 0}, {2, 2, 2}, {2, 3, 0}, {7, 5, 0}, {2, 1, 2}, {4, 2, 0}, {4, 3, 0}, {5, 7, 0}, {2, 0, 2}, 
{4, 0, 0}, {4, 1, 0}, {6, 0, 0}, {7, 2, 0}, {3, 7, 2}, {5, 0, 0}, {6, 1, 0}, {7, 1, 0}, {3, 6, 2}, {5, 1, 0},   // +54.433s 54432
{7, 0, 0}, {7, 6, 0}, {3, 5, 2}, {5, 2, 0}, {6, 2, 0}, {3, 4, 2}, {5, 3, 0}, {5, 4, 0}, {6, 3, 0}, {7, 3, 0}, 
{3, 1, 2}, {3, 2, 0}, {3, 3, 0}, {7, 4, 0}, {3, 0, 2}, {4, 6, 0}, {5, 5, 0}, {5, 6, 0}, {6, 4, 0}, {4, 4, 2}, 
{4, 5, 0}, {4, 7, 0}, {2, 6, 3}, {2, 7, 0}, {6, 5, 0}, {6, 6, 0}, {6, 7, 0}, {0, 6, 2}, {2, 4, 0}, {2, 5, 0}, 
{0, 0, 2}, {0, 1, 0}, {0, 5, 0}, {1, 7, 0}, {0, 2, 2}, {0, 3, 0}, {0, 4, 0}, {1, 0, 2}, {1, 3, 0}, {1, 4, 0}, 
{1, 5, 0}, {1, 1, 2}, {1, 2, 0}, {1, 6, 0}, {7, 5, 0}, {2, 2, 2}, {2, 3, 0}, {4, 3, 0}, {5, 7, 0}, {2, 1, 2},   // +54.867s 54864
{4, 1, 0}, {4, 2, 0}, {7, 2, 0}, {2, 0, 2}, {4, 0, 0}, {6, 0, 0}, {7, 1, 0}, {3, 7, 2}, {5, 0, 0}, {5, 1, 0}, 
{6, 1, 0}, {3, 6, 2}, {5, 2, 0}, {7, 0, 0}, {7, 6, 0}, {3, 5, 2}, {5, 3, 0}, {6, 2, 0}, {7, 3, 0}, {3, 4, 3}, 
{5, 4, 0}, {6, 3, 0}, {7, 4, 0}, {3, 1, 2}, {3, 2, 0}, {3, 3, 0}, {6, 4, 0}, {3, 0, 2}, {4, 6, 0}, {5, 5, 0}, 
{5, 6, 0}, {4, 4, 2}, {4, 5, 0}, {4, 7, 0}, {6, 7, 0}, {2, 4, 2}, {2, 5, 0}, {2, 6, 0}, {2, 7, 0}, {6, 5, 0}, 
{6, 6, 0}, {0, 0, 2}, {0, 6, 0}, {0, 1, 2}, {0, 2, 0}, {0, 3, 0}, {0, 4, 0}, {0, 5, 0}, {1, 7, 0}, {1, 1, 2},   // +55.267s 55264
{1, 6, 0}, {2, 3, 0}, {6, 5, 0}, {0, 6, 2}, {2, 4, 0}, {4, 7, 0}, {5, 6, 0}, {6, 3, 0}, {6, 4, 0}, {7, 3, 0}, 
{7, 4, 0}, {7, 6, 0}, {0, 5, 4}, {1, 2, 0}, {2, 0, 0}, {2, 5, 0}, {5, 5, 0}, {6, 2, 0}, {6, 6, 0}, {7, 0, 0}, 
{1, 3, 5}, {2, 1, 0}, {6, 7, 0}, {0, 4, 2}, {2, 6, 0}, {5, 4, 0}, {6, 1, 0}, {7, 1, 0}, {0, 3, 4}, {1, 4, 0}, 
{4, 4, 0}, {5, 3, 0}, {6, 0, 0}, {6, 5, 0}, {7, 2, 0}, {7, 5, 0}, {7, 3, 2}, {1, 5, 2}, {0, 2, 2}, {4, 5, 0}, 
{5, 2, 0}, {6, 3, 0}, {6, 4, 0}, {6, 6, 2}, {7, 0, 0}, {0, 1, 2}, {1, 0, 0}, {2, 2, 0}, {2, 3, 0}, {4, 6, 0},   // +55.700s 55696
{5, 1, 0}, {2, 0, 2}, {3, 0, 0}, {6, 2, 0}, {7, 4, 0}, {7, 6, 0}, {6, 7, 2}, {0, 0, 2}, {1, 1, 0}, {3, 1, 0}, 
{5, 0, 0}, {7, 1, 0}, {2, 1, 5}, {6, 1, 0}, {6, 7, 0}, {7, 3, 0}, {0, 6, 2}, {1, 2, 0}, {3, 2, 0}, {5, 6, 0}, 
{6, 6, 2}, {3, 3, 2}, {6, 0, 0}, {7, 0, 0}, {7, 5, 0}, {0, 5, 2}, {1, 3, 0}, {1, 7, 0}, {5, 5, 0}, {6, 5, 2}, 
{7, 1, 0}, {3, 4, 2}, {6, 3, 0}, {6, 4, 0}, {0, 4, 2}, {7, 4, 0}, {7, 6, 0}, {1, 4, 2}, {2, 0, 0}, {2, 2, 0}, 
{2, 3, 0}, {3, 5, 0}, {4, 6, 0}, {4, 7, 0}, {5, 4, 0}, {6, 7, 0}, {7, 3, 0}, {6, 2, 2}, {0, 3, 5}, {1, 5, 0},   // +56.233s 56240
{2, 1, 0}, {2, 7, 0}, {3, 6, 0}, {5, 3, 0}, {6, 1, 0}, {6, 6, 0}, {7, 0, 2}, {3, 7, 2}, {0, 2, 2}, {6, 0, 0}, 
{7, 5, 0}, {1, 0, 2}, {4, 0, 0}, {5, 2, 0}, {6, 5, 0}, {7, 1, 0}, {6, 3, 4}, {6, 4, 0}, {0, 1, 2}, {1, 1, 0}, 
{4, 1, 0}, {6, 5, 0}, {7, 3, 0}, {2, 2, 2}, {4, 6, 0}, {5, 1, 0}, {5, 7, 0}, {1, 2, 2}, {2, 0, 0}, {2, 3, 0}, 
{6, 2, 0}, {6, 6, 0}, {7, 4, 0}, {7, 6, 0}, {0, 0, 2}, {4, 2, 0}, {5, 0, 0}, {7, 0, 0}, {1, 3, 5}, {2, 1, 0}, 
{4, 3, 0}, {6, 7, 0}, {7, 1, 0}, {0, 0, 2}, {5, 6, 0}, {6, 1, 0}, {1, 4, 2}, {6, 5, 0}, {7, 3, 2}, {0, 1, 2},   // +56.767s 56768
{1, 6, 0}, {2, 4, 0}, {5, 5, 0}, {6, 0, 0}, {7, 5, 0}, {1, 5, 2}, {0, 2, 2}, {5, 4, 0}, {6, 6, 0}, {7, 0, 0}, 
{1, 0, 2}, {2, 5, 0}, {6, 3, 0}, {6, 4, 0}, {0, 3, 2}, {2, 2, 2}, {2, 3, 0}, {4, 7, 0}, {5, 3, 0}, {6, 2, 0}, 
{6, 7, 0}, {7, 4, 0}, {7, 6, 0}, {1, 1, 2}, {2, 0, 0}, {2, 6, 0}, {7, 1, 0}, {0, 4, 2}, {5, 2, 0}, {7, 2, 0}, 
{2, 1, 3}, {6, 1, 0}, {6, 7, 0}, {0, 5, 2}, {1, 2, 0}, {7, 3, 0}, {4, 4, 2}, {5, 1, 0}, {6, 0, 0}, {7, 5, 0}, 
{6, 6, 2}, {7, 0, 0}, {0, 6, 2}, {1, 3, 0}, {5, 0, 0}, {6, 3, 2}, {6, 4, 0}, {6, 5, 0}, {4, 5, 2}, {7, 1, 0},   // +57.233s 57232
{0, 0, 2}, {5, 6, 0}, {1, 4, 2}, {2, 0, 0}, {6, 7, 0}, {2, 2, 2}, {2, 3, 0}, {3, 0, 0}, {6, 2, 0}, {7, 3, 0}, 
{0, 1, 2}, {4, 6, 0}, {4, 7, 0}, {7, 4, 0}, {7, 6, 0}, {5, 5, 3}, {1, 5, 2}, {2, 1, 0}, {3, 1, 0}, {6, 1, 0}, 
{6, 6, 0}, {7, 0, 0}, {0, 2, 4}, {1, 7, 0}, {5, 4, 0}, {1, 0, 2}, {6, 5, 0}, {3, 2, 2}, {6, 0, 0}, {7, 1, 0}, 
{7, 5, 0}, {0, 3, 2}, {5, 3, 0}, {1, 1, 2}, {6, 3, 0}, {6, 4, 0}, {6, 5, 0}, {3, 3, 2}, {7, 3, 0}, {6, 6, 2}, 
{0, 4, 2}, {1, 2, 0}, {2, 0, 0}, {2, 3, 0}, {2, 7, 0}, {4, 7, 0}, {5, 2, 0}, {6, 2, 0}, {7, 0, 0}, {2, 2, 2},   // +57.767s 57760
{7, 4, 0}, {7, 6, 0}, {1, 3, 2}, {2, 1, 0}, {3, 4, 0}, {6, 1, 0}, {6, 7, 0}, {7, 1, 0}, {0, 5, 3}, {5, 1, 0}, 
{1, 4, 4}, {6, 0, 0}, {6, 5, 0}, {7, 3, 0}, {7, 5, 0}, {0, 6, 2}, {3, 5, 0}, {1, 5, 2}, {5, 0, 0}, {5, 7, 2}, 
{6, 3, 0}, {6, 4, 0}, {6, 6, 0}, {0, 6, 2}, {1, 0, 0}, {3, 6, 0}, {7, 0, 0}, {5, 6, 2}, {2, 0, 2}, {6, 2, 0}, 
{0, 5, 2}, {1, 1, 0}, {2, 2, 0}, {5, 5, 0}, {6, 7, 0}, {7, 1, 0}, {2, 3, 2}, {3, 7, 0}, {4, 6, 0}, {7, 4, 0}, 
{7, 6, 0}, {0, 4, 3}, {2, 1, 0}, {1, 2, 2}, {1, 6, 0}, {5, 4, 0}, {6, 1, 0}, {6, 7, 0}, {7, 3, 0}, {4, 0, 2},   // +58.267s 58272
{0, 3, 2}, {5, 3, 0}, {6, 6, 0}, {6, 0, 2}, {7, 0, 0}, {7, 5, 0}, {0, 2, 2}, {1, 3, 0}, {4, 1, 2}, {5, 2, 0}, 
{6, 5, 0}, {7, 1, 0}, {6, 3, 2}, {6, 4, 0}, {0, 1, 2}, {5, 1, 0}, {7, 2, 0}, {1, 4, 2}, {2, 0, 0}, {4, 2, 0}, 
{6, 7, 0}, {7, 3, 0}, {0, 0, 2}, {5, 0, 0}, {6, 2, 0}, {2, 2, 2}, {2, 3, 0}, {4, 6, 0}, {4, 7, 0}, {7, 4, 0}, 
{7, 6, 0}, {1, 5, 2}, {2, 1, 0}, {6, 1, 0}, {6, 6, 0}, {0, 6, 3}, {4, 3, 0}, {5, 6, 0}, {7, 0, 0}, {6, 0, 2}, 
{6, 5, 2}, {7, 5, 0}, {1, 0, 2}, {2, 4, 0}, {7, 1, 0}, {0, 5, 2}, {5, 5, 0}, {6, 3, 0}, {6, 4, 0}, {2, 5, 2},   // +58.800s 58800
{6, 5, 0}, {1, 1, 2}, {7, 3, 0}, {0, 4, 2}, {5, 4, 0}, {1, 2, 2}, {2, 0, 0}, {2, 6, 0}, {6, 2, 0}, {6, 6, 0}, 
{7, 0, 0}, {0, 3, 4}, {1, 7, 0}, {4, 4, 0}, {4, 6, 0}, {6, 7, 0}, {1, 3, 2}, {2, 1, 0}, {2, 2, 0}, {2, 3, 0}, 
{5, 3, 0}, {6, 1, 0}, {7, 1, 0}, {7, 4, 0}, {7, 6, 0}, {4, 5, 3}, {1, 4, 2}, {6, 5, 0}, {0, 2, 2}, {5, 2, 0}, 
{7, 3, 0}, {1, 5, 2}, {3, 0, 0}, {6, 0, 0}, {7, 5, 0}, {0, 1, 4}, {2, 7, 0}, {6, 6, 0}, {7, 0, 0}, {1, 0, 2}, 
{3, 1, 0}, {5, 1, 0}, {6, 3, 0}, {6, 4, 0}, {3, 2, 2}, {2, 0, 2}, {6, 2, 0}, {6, 7, 0}, {0, 0, 2}, {1, 1, 0},   // +59.333s 59328
{5, 0, 0}, {7, 1, 0}, {2, 1, 2}, {2, 3, 0}, {3, 3, 0}, {4, 7, 0}, {2, 2, 2}, {6, 1, 0}, {6, 7, 0}, {7, 4, 0}, 
{7, 6, 0}, {0, 0, 3}, {1, 2, 0}, {3, 4, 0}, {5, 6, 0}, {7, 3, 0}, {5, 7, 2}, {6, 0, 0}, {7, 5, 0}, {5, 5, 2}, 
{6, 6, 0}, {7, 0, 0}, {0, 1, 2}, {1, 3, 0}, {3, 5, 0}, {6, 3, 2}, {6, 4, 0}, {6, 5, 0}, {0, 2, 2}, {3, 6, 0}, 
{5, 4, 0}, {7, 1, 0}, {1, 4, 4}, {2, 0, 0}, {5, 3, 0}, {6, 2, 0}, {6, 7, 0}, {0, 3, 2}, {1, 6, 0}, {3, 7, 0}, 
{7, 3, 0}, {0, 4, 4}, {5, 2, 0}, {6, 6, 0}, {1, 5, 2}, {2, 1, 0}, {2, 2, 0}, {2, 3, 0}, {4, 0, 0}, {4, 6, 0},   // +59.800s 59792
{4, 7, 0}, {6, 1, 0}, {7, 0, 0}, {7, 4, 0}, {7, 6, 0}, {5, 1, 3}, {0, 5, 2}, {4, 1, 0}, {1, 0, 2}, {6, 0, 0}, 
{6, 5, 0}, {7, 1, 0}, {7, 5, 0}, {0, 6, 2}, {4, 2, 0}, {5, 0, 0}, {7, 2, 2}, {1, 1, 2}, {6, 3, 0}, {6, 4, 0}, 
{6, 5, 0}, {7, 3, 0}, {0, 0, 2}, {4, 3, 0}, {5, 6, 0}, {1, 2, 2}, {6, 6, 0}, {2, 0, 2}, {6, 2, 0}, {7, 0, 0}, 
{0, 1, 4}, {1, 3, 0}, {2, 1, 0}, {2, 4, 0}, {5, 5, 0}, {6, 1, 0}, {6, 7, 0}, {7, 1, 0}, {2, 2, 2}, {2, 3, 0}, 
{4, 7, 0}, {7, 4, 0}, {7, 6, 0}, {1, 4, 3}, {0, 2, 2}, {2, 5, 0}, {6, 0, 0}, {6, 5, 0}, {7, 3, 0}, {7, 5, 0},   // +60.267s 60272
{5, 4, 2}, {1, 5, 2}, {2, 6, 2}, {6, 3, 0}, {6, 4, 0}, {6, 6, 0}, {0, 3, 2}, {1, 0, 0}, {5, 3, 0}, {7, 0, 0}, 
{1, 7, 2}, {2, 0, 2}, {4, 4, 0}, {6, 2, 0}, {0, 4, 2}, {1, 1, 0}, {6, 7, 0}, {7, 1, 0}, {5, 2, 2}, {2, 1, 2}, 
{1, 2, 2}, {2, 2, 0}, {4, 5, 0}, {4, 6, 0}, {6, 1, 0}, {6, 7, 0}, {7, 3, 0}, {0, 5, 3}, {2, 3, 0}, {5, 1, 0}, 
{7, 4, 0}, {7, 6, 0}, {6, 6, 2}, {2, 7, 4}, {6, 0, 0}, {7, 0, 0}, {0, 6, 2}, {1, 3, 0}, {5, 0, 0}, {7, 5, 0}, 
{3, 0, 2}, {6, 5, 0}, {7, 1, 0}, {1, 4, 4}, {6, 3, 0}, {6, 4, 0}, {0, 6, 2}, {5, 6, 0}, {3, 1, 4}, {6, 7, 0},   // +60.967s 60960
{0, 5, 3}, {1, 5, 0}, {2, 0, 0}, {5, 7, 0}, {6, 2, 0}, {7, 3, 0}, {5, 5, 2}, {2, 1, 2}, {2, 3, 0}, {6, 6, 0}, 
{7, 4, 0}, {7, 6, 0}, {1, 0, 2}, {2, 2, 0}, {3, 2, 0}, {4, 6, 0}, {4, 7, 0}, {7, 0, 0}, {6, 1, 2}, {0, 4, 2}, 
{5, 4, 0}, {7, 5, 0}, {1, 1, 2}, {1, 6, 0}, {3, 3, 0}, {6, 0, 0}, {6, 5, 0}, {0, 3, 2}, {5, 3, 0}, {7, 1, 0}, 
{6, 4, 2}, {1, 2, 2}, {3, 4, 0}, {6, 3, 0}, {6, 5, 0}, {0, 2, 2}, {5, 2, 0}, {7, 3, 0}, {1, 3, 2}, {6, 6, 0}, 
{0, 1, 2}, {2, 0, 0}, {5, 1, 0}, {3, 5, 3}, {6, 2, 0}, {7, 0, 0}, {1, 4, 2}, {2, 2, 0}, {2, 3, 0}, {4, 6, 0},   // +61.467s 61472
{6, 7, 0}, {7, 2, 0}, {0, 0, 2}, {2, 1, 0}, {5, 0, 0}, {7, 1, 0}, {7, 4, 0}, {7, 6, 0}, {1, 5, 2}, {6, 1, 0}, 
{3, 6, 2}, {5, 6, 0}, {6, 5, 0}, {0, 6, 2}, {7, 3, 0}, {1, 0, 2}, {7, 5, 0}, {3, 7, 2}, {6, 0, 0}, {6, 6, 0}, 
{0, 5, 2}, {5, 5, 0}, {6, 4, 0}, {1, 1, 2}, {6, 3, 0}, {7, 0, 0}, {6, 7, 2}, {2, 0, 2}, {4, 0, 0}, {5, 4, 0}, 
{0, 4, 3}, {1, 2, 0}, {6, 2, 0}, {7, 1, 0}, {6, 7, 2}, {7, 4, 0}, {7, 6, 0}, {2, 1, 2}, {2, 2, 0}, {2, 3, 0}, 
{4, 1, 0}, {4, 7, 0}, {6, 1, 0}, {0, 3, 2}, {1, 3, 0}, {5, 3, 0}, {7, 3, 0}, {1, 7, 2}, {6, 6, 0}, {7, 5, 0},   // +61.967s 61968
{6, 0, 2}, {7, 0, 0}, {0, 2, 2}, {4, 2, 0}, {5, 2, 0}, {6, 5, 0}, {1, 4, 2}, {6, 4, 0}, {6, 3, 2}, {7, 1, 0}, 
{6, 7, 2}, {0, 1, 2}, {1, 5, 0}, {2, 0, 0}, {4, 3, 0}, {5, 1, 0}, {2, 7, 2}, {6, 2, 0}, {7, 3, 0}, {2, 4, 3}, 
{0, 0, 2}, {2, 2, 0}, {5, 0, 0}, {6, 6, 0}, {1, 0, 2}, {2, 1, 0}, {2, 3, 0}, {2, 5, 0}, {4, 6, 0}, {4, 7, 0}, 
{7, 0, 0}, {7, 4, 0}, {7, 6, 0}, {6, 1, 2}, {0, 0, 2}, {5, 6, 0}, {6, 5, 0}, {1, 1, 2}, {2, 6, 0}, {7, 5, 0}, 
{6, 0, 2}, {7, 1, 0}, {0, 1, 2}, {1, 2, 0}, {4, 4, 0}, {5, 5, 0}, {5, 7, 0}, {6, 5, 0}, {6, 4, 2}, {0, 2, 2},   // +62.533s 62528
{5, 4, 0}, {6, 3, 0}, {7, 3, 0}, {1, 3, 2}, {2, 0, 0}, {4, 5, 0}, {6, 6, 0}, {5, 3, 2}, {7, 0, 0}, {0, 3, 3}, 
{1, 4, 0}, {3, 0, 0}, {6, 2, 0}, {6, 7, 0}, {2, 1, 2}, {0, 4, 2}, {1, 6, 0}, {2, 2, 0}, {2, 3, 0}, {4, 7, 0}, 
{5, 2, 0}, {6, 1, 0}, {7, 1, 0}, {7, 4, 0}, {7, 6, 0}, {1, 5, 2}, {3, 1, 0}, {6, 5, 0}, {5, 1, 2}, {6, 0, 0}, 
{7, 5, 0}, {0, 5, 2}, {1, 0, 0}, {3, 2, 0}, {7, 3, 0}, {6, 4, 2}, {6, 6, 0}, {0, 6, 2}, {5, 0, 0}, {6, 3, 0}, 
{1, 1, 2}, {3, 3, 0}, {7, 0, 0}, {0, 0, 4}, {2, 0, 0}, {3, 4, 0}, {5, 6, 0}, {6, 7, 0}, {7, 2, 0}, {1, 2, 3},   // +63.000s 63008
{6, 2, 0}, {7, 1, 0}, {0, 1, 4}, {3, 5, 0}, {6, 7, 0}, {1, 3, 2}, {2, 1, 0}, {2, 2, 0}, {5, 5, 0}, {6, 1, 0}, 
{7, 3, 0}, {2, 3, 2}, {4, 6, 0}, {6, 6, 0}, {7, 4, 0}, {7, 6, 0}, {3, 6, 2}, {0, 2, 2}, {5, 4, 0}, {7, 0, 0}, 
{7, 5, 0}, {1, 4, 2}, {3, 7, 0}, {6, 0, 0}, {6, 5, 0}, {7, 1, 2}, {0, 3, 2}, {6, 4, 0}, {4, 0, 2}, {5, 3, 0}, 
{6, 3, 0}, {6, 7, 0}, {1, 5, 2}, {2, 0, 0}, {4, 1, 0}, {7, 3, 0}, {6, 2, 2}, {0, 4, 3}, {1, 7, 0}, {5, 2, 0}, 
{6, 6, 0}, {1, 0, 2}, {2, 1, 0}, {4, 2, 0}, {2, 3, 2}, {6, 1, 0}, {7, 0, 0}, {0, 5, 2}, {2, 2, 0}, {4, 3, 0},   // +63.533s 63536
{4, 6, 0}, {4, 7, 0}, {5, 1, 0}, {7, 4, 0}, {7, 5, 0}, {7, 6, 0}, {1, 1, 2}, {6, 0, 0}, {6, 5, 0}, {7, 1, 2}, 
{6, 4, 2}, {0, 6, 2}, {2, 4, 0}, {6, 5, 15}, {1, 2, 2}, {2, 7, 0}, {2, 0, 2}, {2, 2, 0}, {2, 3, 2}, {7, 3, 0}, 
{7, 4, 0}, {7, 6, 0}, {0, 6, 2}, {4, 6, 0}, {6, 3, 0}, {2, 5, 2}, {5, 0, 0}, {1, 3, 2}, {6, 6, 0}, {0, 5, 4}, 
{2, 1, 0}, {5, 6, 0}, {7, 0, 0}, {1, 4, 3}, {2, 6, 0}, {5, 7, 0}, {6, 2, 0}, {6, 7, 0}, {7, 1, 2}, {0, 4, 2}, 
{5, 5, 0}, {7, 5, 0}, {1, 5, 2}, {0, 3, 2}, {6, 5, 0}, {2, 3, 2}, {4, 4, 0}, {5, 4, 0}, {6, 1, 0}, {6, 4, 0},   // +64.367s 64368
{7, 3, 0}, {1, 0, 2}, {1, 6, 0}, {2, 2, 0}, {7, 4, 0}, {7, 6, 0}, {0, 2, 2}, {2, 0, 0}, {4, 7, 0}, {5, 3, 0}, 
{6, 6, 0}, {4, 5, 2}, {6, 0, 0}, {7, 0, 0}, {0, 1, 2}, {1, 1, 0}, {5, 2, 0}, {2, 1, 2}, {5, 1, 2}, {6, 3, 0}, 
{6, 7, 0}, {0, 0, 2}, {1, 2, 0}, {3, 0, 0}, {7, 1, 0}, {6, 2, 3}, {6, 7, 0}, {7, 5, 0}, {5, 0, 2}, {7, 2, 0}, 
{0, 6, 2}, {6, 4, 0}, {7, 3, 0}, {1, 3, 2}, {3, 1, 0}, {6, 1, 0}, {6, 6, 0}, {5, 6, 2}, {7, 0, 0}, {0, 5, 2}, 
{2, 2, 0}, {2, 3, 0}, {6, 0, 0}, {6, 5, 0}, {1, 4, 2}, {2, 0, 0}, {3, 2, 0}, {7, 1, 0}, {7, 4, 0}, {7, 6, 0},   // +64.833s 64832
{4, 6, 2}, {4, 7, 0}, {5, 5, 0}, {0, 4, 2}, {6, 3, 0}, {6, 7, 0}, {2, 1, 2}, {7, 3, 0}, {1, 5, 2}, {3, 3, 0}, 
{5, 4, 2}, {0, 3, 3}, {6, 2, 0}, {6, 6, 0}, {1, 0, 2}, {3, 4, 0}, {7, 0, 0}, {7, 5, 0}, {5, 3, 2}, {0, 2, 2}, 
{1, 7, 0}, {6, 1, 0}, {6, 5, 0}, {1, 1, 2}, {6, 4, 0}, {7, 1, 0}, {2, 2, 2}, {2, 3, 0}, {3, 5, 0}, {1, 2, 2}, 
{2, 0, 0}, {5, 2, 0}, {6, 5, 0}, {7, 4, 0}, {7, 6, 0}, {0, 1, 2}, {4, 7, 0}, {6, 0, 0}, {7, 3, 0}, {2, 1, 2}, 
{3, 6, 0}, {1, 3, 2}, {5, 1, 0}, {6, 6, 0}, {0, 0, 2}, {6, 3, 0}, {7, 0, 0}, {1, 4, 3}, {2, 7, 0}, {3, 7, 0},   // +65.400s 65408
{6, 7, 0}, {7, 5, 0}, {6, 2, 2}, {7, 1, 0}, {0, 0, 2}, {5, 0, 0}, {1, 5, 2}, {6, 4, 0}, {6, 5, 0}, {4, 0, 2}, 
{5, 6, 0}, {6, 1, 0}, {7, 3, 0}, {0, 1, 2}, {1, 0, 2}, {2, 0, 0}, {2, 2, 0}, {6, 0, 0}, {0, 2, 2}, {2, 3, 0}, 
{4, 1, 0}, {5, 5, 0}, {5, 7, 0}, {6, 6, 0}, {4, 6, 2}, {7, 0, 0}, {7, 4, 0}, {7, 6, 0}, {1, 1, 2}, {5, 4, 0}, 
{6, 3, 0}, {0, 3, 2}, {2, 1, 0}, {4, 2, 0}, {6, 7, 0}, {7, 1, 2}, {0, 4, 2}, {1, 2, 0}, {5, 3, 0}, {6, 2, 0}, 
{6, 7, 3}, {7, 5, 0}, {0, 5, 2}, {4, 3, 0}, {5, 2, 0}, {7, 3, 0}, {1, 3, 2}, {1, 6, 0}, {6, 6, 0}, {6, 1, 2},   // +65.933s 65936
{6, 4, 0}, {7, 0, 0}, {0, 6, 2}, {2, 4, 0}, {5, 1, 0}, {6, 5, 2}, {1, 4, 2}, {2, 0, 0}, {2, 2, 0}, {2, 3, 0}, 
{5, 0, 0}, {6, 0, 0}, {7, 1, 0}, {7, 4, 0}, {7, 6, 0}, {0, 0, 2}, {2, 5, 0}, {2, 1, 2}, {4, 6, 0}, {4, 7, 0}, 
{6, 7, 0}, {1, 5, 2}, {2, 6, 0}, {5, 6, 0}, {6, 3, 0}, {7, 3, 0}, {0, 1, 2}, {7, 2, 0}, {6, 6, 2}, {7, 5, 0}, 
{4, 4, 3}, {6, 2, 0}, {7, 0, 0}, {1, 0, 2}, {5, 5, 0}, {0, 2, 2}, {4, 5, 0}, {6, 1, 0}, {6, 4, 0}, {6, 5, 2}, 
{1, 1, 2}, {5, 4, 0}, {7, 1, 0}, {0, 3, 2}, {2, 0, 0}, {3, 0, 0}, {6, 0, 0}, {1, 2, 2}, {2, 2, 0}, {2, 3, 0},   // +66.433s 66432
{6, 5, 0}, {3, 1, 2}, {7, 3, 0}, {7, 4, 0}, {7, 6, 0}, {1, 3, 2}, {2, 1, 0}, {4, 6, 0}, {5, 3, 0}, {6, 3, 0}, 
{6, 6, 0}, {0, 4, 2}, {3, 2, 0}, {7, 0, 0}, {1, 4, 4}, {5, 2, 0}, {6, 2, 0}, {6, 7, 0}, {0, 5, 3}, {1, 7, 0}, 
{3, 3, 0}, {7, 1, 0}, {7, 5, 0}, {6, 5, 2}, {1, 5, 2}, {3, 4, 0}, {5, 1, 0}, {6, 1, 0}, {6, 4, 2}, {7, 3, 0}, 
{0, 6, 2}, {1, 0, 0}, {2, 0, 2}, {3, 5, 0}, {6, 6, 0}, {5, 0, 2}, {6, 0, 0}, {7, 0, 0}, {0, 6, 2}, {1, 1, 0}, 
{2, 2, 0}, {2, 3, 0}, {3, 6, 0}, {7, 4, 0}, {7, 6, 0}, {2, 1, 2}, {2, 7, 0}, {4, 7, 0}, {6, 7, 0}, {0, 5, 2},   // +66.933s 66928
{5, 6, 0}, {6, 3, 0}, {7, 1, 0}, {1, 2, 2}, {3, 7, 0}, {7, 5, 0}, {0, 4, 2}, {5, 5, 0}, {6, 2, 0}, {6, 7, 0}, 
{7, 3, 3}, {1, 3, 2}, {4, 0, 0}, {6, 4, 0}, {0, 3, 2}, {5, 4, 0}, {6, 1, 0}, {6, 6, 0}, {4, 1, 2}, {5, 7, 0}, 
{7, 0, 0}, {0, 2, 2}, {2, 0, 0}, {6, 0, 0}, {6, 5, 0}, {1, 4, 2}, {5, 3, 0}, {7, 1, 0}, {2, 2, 2}, {4, 2, 0}, 
{0, 1, 2}, {2, 3, 0}, {5, 2, 0}, {6, 3, 0}, {6, 7, 0}, {2, 1, 2}, {4, 3, 0}, {7, 3, 0}, {7, 4, 0}, {7, 6, 0}, 
{0, 0, 2}, {1, 5, 0}, {4, 6, 0}, {4, 7, 0}, {1, 6, 2}, {5, 1, 0}, {2, 4, 2}, {6, 2, 0}, {6, 6, 0}, {0, 6, 3},   // +67.433s 67440
{1, 0, 0}, {5, 0, 0}, {7, 0, 0}, {7, 5, 0}, {2, 5, 4}, {6, 1, 0}, {6, 5, 0}, {1, 1, 2}, {5, 6, 0}, {6, 4, 0}, 
{7, 1, 0}, {0, 5, 2}, {1, 2, 2}, {2, 0, 0}, {6, 5, 0}, {2, 6, 2}, {5, 5, 0}, {6, 0, 0}, {7, 2, 0}, {7, 3, 0}, 
{0, 4, 2}, {2, 1, 0}, {2, 2, 0}, {2, 3, 0}, {6, 6, 0}, {1, 3, 2}, {6, 3, 0}, {7, 4, 0}, {7, 6, 0}, {4, 4, 2}, 
{4, 7, 0}, {7, 0, 0}, {1, 4, 2}, {5, 4, 0}, {6, 7, 0}, {7, 5, 0}, {0, 3, 2}, {6, 2, 0}, {7, 1, 0}, {4, 5, 3}, 
{1, 5, 2}, {5, 3, 0}, {6, 1, 0}, {6, 4, 0}, {6, 5, 0}, {0, 2, 2}, {7, 3, 0}, {1, 0, 4}, {2, 0, 0}, {3, 0, 0},   // +67.967s 67968
{6, 0, 0}, {6, 6, 0}, {0, 1, 2}, {5, 2, 0}, {7, 0, 0}, {1, 1, 2}, {3, 1, 2}, {6, 3, 0}, {1, 7, 2}, {2, 1, 0}, 
{2, 2, 0}, {2, 3, 0}, {5, 1, 0}, {6, 7, 0}, {7, 4, 0}, {7, 6, 0}, {0, 0, 2}, {1, 2, 0}, {4, 6, 0}, {7, 1, 0}, 
{6, 2, 2}, {3, 2, 2}, {6, 7, 0}, {7, 5, 0}, {0, 0, 3}, {5, 0, 0}, {7, 3, 0}, {1, 3, 2}, {6, 6, 0}, {0, 1, 2}, 
{3, 3, 0}, {6, 1, 0}, {6, 4, 0}, {7, 0, 0}, {2, 7, 2}, {5, 6, 0}, {1, 4, 2}, {2, 0, 0}, {6, 5, 0}, {0, 2, 2}, 
{5, 5, 0}, {6, 0, 0}, {7, 1, 0}, {3, 4, 2}, {0, 3, 2}, {2, 1, 0}, {2, 3, 0}, {6, 7, 0}, {1, 5, 2}, {2, 2, 0},   // +68.500s 68496
{5, 4, 0}, {6, 3, 0}, {7, 3, 0}, {7, 4, 0}, {7, 6, 0}, {3, 5, 2}, {0, 4, 2}, {4, 6, 0}, {4, 7, 0}, {5, 3, 0}, 
{6, 6, 0}, {7, 5, 0}, {5, 7, 2}, {6, 2, 0}, {7, 0, 0}, {0, 5, 3}, {1, 0, 0}, {5, 2, 0}, {3, 6, 2}, {6, 1, 0}, 
{6, 4, 0}, {6, 5, 0}, {0, 6, 4}, {1, 1, 0}, {5, 1, 0}, {6, 0, 0}, {7, 1, 0}, {2, 0, 2}, {3, 7, 0}, {6, 5, 0}, 
{1, 2, 2}, {5, 0, 0}, {7, 3, 0}, {0, 0, 2}, {6, 3, 0}, {1, 3, 2}, {1, 6, 0}, {2, 1, 0}, {6, 6, 0}, {2, 2, 2}, 
{2, 3, 0}, {4, 0, 0}, {5, 6, 0}, {7, 0, 0}, {0, 1, 2}, {6, 7, 0}, {7, 4, 0}, {7, 6, 0}, {1, 4, 2}, {4, 6, 0},   // +68.967s 68960
{6, 2, 0}, {7, 1, 0}, {4, 1, 2}, {7, 5, 0}, {0, 2, 3}, {1, 5, 0}, {5, 5, 0}, {6, 5, 0}, {6, 1, 2}, {7, 3, 0}, 
{6, 4, 2}, {7, 2, 0}, {1, 0, 2}, {4, 2, 0}, {5, 4, 0}, {0, 3, 2}, {2, 0, 0}, {6, 6, 0}, {6, 0, 2}, {7, 0, 0}, 
{1, 1, 2}, {2, 1, 0}, {4, 3, 0}, {0, 4, 2}, {5, 3, 0}, {6, 7, 0}, {2, 2, 2}, {2, 3, 0}, {6, 3, 0}, {7, 1, 0}, 
{1, 2, 2}, {2, 4, 0}, {7, 4, 0}, {7, 5, 0}, {7, 6, 0}, {4, 7, 2}, {5, 2, 0}, {6, 2, 0}, {6, 7, 0}, {0, 5, 2}, 
{2, 5, 0}, {7, 3, 0}, {1, 3, 3}, {6, 4, 0}, {6, 1, 2}, {6, 6, 0}, {0, 6, 2}, {2, 6, 0}, {5, 1, 0}, {7, 0, 0},   // +69.500s 69504
{2, 0, 2}, {6, 0, 0}, {6, 5, 0}, {1, 4, 2}, {4, 4, 0}, {7, 1, 0}, {0, 6, 2}, {1, 7, 0}, {5, 0, 0}, {6, 3, 2}, 
{6, 7, 0}, {2, 1, 2}, {4, 5, 0}, {7, 3, 0}, {0, 5, 2}, {1, 5, 0}, {2, 2, 0}, {5, 6, 0}, {2, 3, 2}, {3, 0, 0}, 
{6, 2, 0}, {6, 6, 0}, {0, 4, 2}, {5, 5, 0}, {7, 4, 0}, {7, 6, 0}, {1, 0, 3}, {3, 1, 0}, {4, 6, 0}, {4, 7, 0}, 
{7, 0, 0}, {7, 5, 0}, {2, 7, 2}, {0, 3, 2}, {5, 4, 0}, {6, 1, 0}, {6, 4, 0}, {6, 5, 0}, {1, 1, 2}, {3, 2, 0}, 
{7, 1, 0}, {0, 2, 2}, {5, 3, 0}, {1, 2, 2}, {2, 0, 0}, {6, 0, 0}, {6, 5, 0}, {0, 1, 2}, {3, 3, 0}, {7, 3, 0},   // +70.000s 70000
{1, 3, 2}, {2, 1, 0}, {5, 2, 0}, {6, 6, 0}, {3, 4, 2}, {6, 3, 0}, {7, 0, 0}, {0, 0, 2}, {5, 1, 0}, {5, 7, 0}, 
{1, 4, 2}, {2, 3, 0}, {6, 7, 0}, {7, 4, 0}, {7, 5, 0}, {7, 6, 0}, {2, 2, 2}, {3, 5, 0}, {6, 2, 0}, {7, 1, 0}, 
{0, 6, 2}, {4, 7, 0}, {5, 0, 0}, {1, 5, 3}, {3, 6, 0}, {6, 1, 0}, {6, 4, 0}, {6, 5, 0}, {7, 3, 2}, {0, 5, 2}, 
{1, 0, 0}, {1, 6, 0}, {3, 7, 0}, {5, 6, 0}, {2, 0, 2}, {6, 0, 0}, {6, 6, 0}, {7, 0, 2}, {1, 1, 2}, {4, 0, 0}, 
{5, 5, 0}, {0, 4, 2}, {2, 1, 0}, {6, 3, 0}, {4, 1, 2}, {6, 7, 0}, {1, 2, 2}, {7, 1, 0}, {0, 3, 2}, {2, 2, 0},   // +70.533s 70528
{2, 3, 0}, {5, 4, 0}, {6, 2, 0}, {4, 2, 2}, {6, 7, 0}, {7, 2, 0}, {7, 4, 0}, {7, 5, 0}, {7, 6, 0}, {4, 6, 2}, 
{7, 3, 0}, {1, 3, 3}, {4, 3, 0}, {5, 3, 0}, {6, 6, 0}, {0, 2, 2}, {6, 1, 0}, {6, 4, 0}, {7, 0, 0}, {6, 5, 2}, 
{1, 4, 2}, {2, 0, 0}, {5, 2, 0}, {0, 1, 2}, {2, 4, 0}, {6, 0, 0}, {7, 1, 0}, {6, 7, 2}, {2, 1, 2}, {7, 3, 0}, 
{1, 5, 2}, {5, 1, 0}, {6, 3, 0}, {0, 0, 2}, {2, 5, 0}, {7, 5, 0}, {2, 3, 2}, {6, 2, 0}, {6, 6, 0}, {0, 0, 2}, 
{2, 2, 0}, {5, 0, 0}, {7, 0, 0}, {7, 4, 0}, {7, 6, 0}, {1, 0, 3}, {2, 6, 0}, {6, 4, 0}, {1, 7, 2}, {4, 6, 0},   // +71.033s 71040
{4, 7, 0}, {6, 1, 0}, {6, 5, 0}, {0, 1, 2}, {1, 1, 0}, {5, 6, 0}, {7, 1, 0}, {4, 4, 2}, {6, 0, 0}, {0, 2, 2}, 
{1, 2, 0}, {2, 0, 0}, {6, 5, 0}, {5, 5, 2}, {7, 3, 0}, {6, 3, 2}, {0, 3, 2}, {1, 3, 0}, {2, 1, 0}, {4, 5, 0}, 
{5, 4, 0}, {6, 6, 0}, {7, 0, 2}, {0, 4, 2}, {1, 4, 0}, {2, 7, 0}, {6, 7, 0}, {2, 2, 2}, {3, 0, 0}, {5, 3, 0}, 
{6, 2, 0}, {7, 1, 0}, {2, 3, 2}, {7, 5, 0}, {0, 5, 3}, {1, 5, 0}, {4, 6, 0}, {5, 2, 0}, {6, 5, 0}, {7, 4, 0}, 
{7, 6, 0}, {3, 1, 2}, {6, 1, 0}, {7, 3, 0}, {0, 6, 2}, {6, 4, 0}, {1, 0, 2}, {5, 1, 0}, {2, 0, 2}, {5, 7, 0},   // +71.533s 71536
{6, 6, 0}, {0, 0, 2}, {3, 2, 0}, {5, 0, 0}, {6, 0, 0}, {7, 0, 0}, {1, 1, 2}, {2, 1, 0}, {6, 3, 2}, {6, 7, 0}, 
{3, 3, 2}, {5, 6, 0}, {7, 1, 0}, {0, 1, 2}, {1, 2, 0}, {7, 5, 0}, {6, 2, 2}, {6, 7, 0}, {2, 2, 2}, {2, 3, 0}, 
{5, 5, 0}, {7, 3, 0}, {7, 4, 0}, {7, 6, 0}, {0, 2, 2}, {1, 3, 0}, {1, 6, 0}, {3, 4, 0}, {6, 1, 0}, {6, 4, 0}, 
{6, 6, 0}, {4, 7, 3}, {7, 0, 0}, {0, 3, 4}, {2, 0, 0}, {5, 4, 0}, {6, 0, 0}, {6, 5, 0}, {1, 4, 2}, {3, 5, 0}, 
{7, 1, 0}, {5, 3, 4}, {6, 3, 0}, {6, 7, 0}, {0, 4, 2}, {1, 5, 0}, {2, 1, 0}, {7, 3, 0}, {3, 6, 2}, {7, 2, 0},   // +72.067s 72064
{6, 2, 2}, {6, 6, 0}, {0, 5, 2}, {5, 2, 0}, {7, 0, 0}, {7, 5, 0}, {1, 0, 2}, {2, 2, 0}, {2, 3, 0}, {3, 7, 0}, 
{7, 4, 2}, {7, 6, 0}, {6, 1, 3}, {6, 5, 0}, {6, 5, 10}, {1, 1, 2}, {6, 4, 2}, {0, 6, 2}, {4, 6, 0}, {4, 7, 0}, 
{6, 0, 0}, {7, 1, 0}, {4, 0, 2}, {5, 1, 0}, {1, 2, 2}, {6, 6, 0}, {2, 0, 2}, {2, 2, 0}, {2, 3, 0}, {6, 3, 0}, 
{7, 3, 0}, {0, 6, 3}, {5, 0, 2}, {4, 1, 2}, {6, 7, 0}, {1, 3, 2}, {1, 7, 0}, {2, 1, 0}, {6, 2, 0}, {7, 4, 0}, 
{7, 6, 0}, {0, 5, 2}, {7, 0, 0}, {5, 6, 2}, {1, 4, 2}, {4, 2, 0}, {6, 1, 0}, {6, 5, 0}, {7, 1, 0}, {7, 5, 0},   // +72.833s 72832
{0, 4, 2}, {4, 7, 0}, {5, 5, 0}, {0, 3, 4}, {1, 5, 0}, {6, 0, 0}, {6, 4, 0}, {7, 3, 0}, {2, 7, 2}, {4, 3, 0}, 
{5, 4, 0}, {6, 6, 0}, {0, 2, 3}, {1, 0, 0}, {2, 2, 0}, {2, 3, 0}, {2, 0, 2}, {5, 3, 0}, {6, 3, 0}, {7, 0, 0}, 
{2, 4, 2}, {6, 7, 0}, {0, 1, 2}, {1, 1, 0}, {2, 5, 2}, {5, 2, 0}, {6, 2, 0}, {7, 4, 0}, {7, 6, 0}, {0, 0, 2}, 
{2, 1, 0}, {6, 7, 0}, {7, 1, 0}, {1, 2, 2}, {2, 6, 0}, {5, 1, 0}, {5, 7, 2}, {0, 6, 2}, {6, 1, 0}, {6, 6, 0}, 
{7, 3, 0}, {7, 5, 0}, {1, 3, 2}, {4, 4, 0}, {4, 6, 0}, {5, 0, 0}, {6, 5, 2}, {7, 0, 0}, {4, 5, 2}, {6, 0, 0},   // +73.367s 73360
{6, 4, 0}, {0, 5, 2}, {2, 2, 0}, {2, 3, 0}, {5, 6, 0}, {1, 4, 3}, {1, 6, 0}, {2, 0, 0}, {6, 7, 0}, {7, 1, 0}, 
{3, 0, 2}, {6, 3, 0}, {0, 4, 2}, {5, 5, 0}, {2, 1, 2}, {3, 1, 0}, {7, 3, 0}, {7, 4, 0}, {7, 6, 0}, {1, 5, 2}, 
{6, 2, 0}, {6, 6, 0}, {5, 4, 2}, {0, 3, 2}, {3, 2, 0}, {6, 1, 0}, {7, 0, 0}, {7, 5, 0}, {1, 0, 2}, {6, 5, 0}, 
{4, 6, 2}, {4, 7, 0}, {6, 0, 0}, {6, 4, 0}, {7, 2, 0}, {0, 2, 2}, {3, 3, 0}, {5, 3, 0}, {1, 1, 2}, {6, 5, 0}, 
{7, 1, 0}, {2, 2, 2}, {3, 4, 0}, {6, 3, 0}, {2, 0, 3}, {2, 3, 0}, {5, 2, 0}, {6, 6, 0}, {7, 3, 0}, {0, 1, 2},   // +73.867s 73872
{1, 2, 0}, {3, 5, 0}, {1, 3, 4}, {6, 2, 0}, {6, 7, 0}, {7, 0, 0}, {0, 0, 2}, {2, 1, 0}, {3, 6, 0}, {5, 1, 0}, 
{7, 4, 0}, {7, 6, 0}, {1, 4, 2}, {7, 1, 0}, {6, 1, 2}, {6, 5, 0}, {0, 0, 2}, {3, 7, 0}, {5, 0, 0}, {7, 5, 0}, 
{1, 5, 2}, {4, 6, 0}, {7, 3, 0}, {0, 1, 2}, {4, 0, 0}, {6, 6, 0}, {1, 0, 2}, {1, 7, 0}, {5, 6, 0}, {6, 0, 0}, 
{6, 4, 0}, {2, 3, 2}, {4, 1, 0}, {0, 2, 3}, {2, 0, 0}, {2, 2, 0}, {7, 0, 0}, {1, 1, 2}, {5, 5, 0}, {6, 3, 0}, 
{6, 7, 0}, {0, 3, 2}, {2, 1, 0}, {4, 2, 0}, {5, 4, 2}, {6, 2, 0}, {7, 1, 0}, {1, 2, 2}, {4, 3, 0}, {6, 7, 0},   // +74.367s 74368
{7, 4, 0}, {7, 6, 0}, {0, 4, 2}, {7, 5, 0}, {2, 7, 2}, {5, 3, 0}, {6, 1, 0}, {6, 6, 0}, {7, 3, 0}, {0, 5, 2}, 
{2, 4, 0}, {1, 3, 2}, {4, 7, 0}, {5, 2, 0}, {6, 0, 0}, {6, 4, 0}, {6, 5, 2}, {7, 0, 0}, {0, 6, 2}, {2, 5, 0}, 
{5, 1, 0}, {6, 3, 2}, {7, 1, 0}, {1, 4, 3}, {2, 0, 0}, {2, 2, 0}, {2, 3, 0}, {6, 7, 0}, {0, 0, 2}, {5, 0, 0}, 
{5, 7, 0}, {2, 6, 2}, {6, 2, 0}, {7, 3, 0}, {1, 5, 2}, {2, 1, 0}, {6, 6, 0}, {0, 1, 2}, {5, 6, 0}, {4, 4, 2}, 
{7, 0, 0}, {7, 4, 0}, {7, 6, 0}, {6, 1, 2}, {6, 5, 0}, {7, 5, 0}, {1, 0, 2}, {5, 5, 0}, {0, 2, 2}, {1, 6, 2},   // +74.933s 74928
{4, 5, 0}, {4, 6, 0}, {4, 7, 0}, {6, 0, 0}, {6, 4, 0}, {6, 5, 0}, {7, 1, 0}, {1, 1, 2}, {0, 3, 2}, {5, 4, 0}, 
{1, 2, 3}, {2, 0, 0}, {2, 3, 0}, {6, 3, 0}, {6, 6, 0}, {7, 3, 0}, {2, 2, 2}, {3, 0, 0}, {0, 4, 2}, {1, 3, 0}, 
{5, 3, 0}, {6, 7, 0}, {7, 0, 0}, {2, 1, 2}, {6, 2, 0}, {3, 1, 2}, {7, 4, 0}, {7, 6, 0}, {1, 4, 2}, {6, 1, 0}, 
{6, 5, 0}, {7, 1, 0}, {7, 2, 0}, {7, 5, 0}, {0, 5, 2}, {5, 2, 0}, {1, 5, 2}, {3, 2, 0}, {6, 0, 2}, {6, 4, 0}, 
{7, 3, 0}, {1, 0, 6}, {2, 0, 0}, {4, 7, 0}, {6, 6, 0}, {7, 0, 0}, {0, 6, 3}, {2, 2, 0}, {3, 3, 0}, {5, 1, 0},   // +75.433s 75440
{6, 3, 0}, {2, 3, 2}, {1, 1, 2}, {2, 1, 0}, {0, 6, 2}, {3, 4, 0}, {5, 0, 0}, {6, 2, 0}, {6, 7, 0}, {7, 1, 0}, 
{0, 5, 4}, {1, 2, 0}, {7, 4, 0}, {7, 6, 0}, {5, 6, 2}, {6, 1, 0}, {6, 7, 0}, {7, 3, 0}, {7, 5, 0}, {1, 7, 2}, 
{3, 5, 0}, {0, 4, 2}, {5, 5, 0}, {6, 6, 0}, {7, 0, 0}, {1, 3, 2}, {6, 0, 0}, {6, 4, 0}, {0, 3, 2}, {3, 6, 0}, 
{6, 5, 0}, {7, 1, 0}, {2, 0, 2}, {4, 6, 0}, {5, 4, 0}, {1, 4, 3}, {6, 3, 0}, {0, 2, 2}, {2, 2, 0}, {2, 3, 0}, 
{5, 3, 0}, {6, 7, 0}, {7, 3, 0}, {2, 1, 2}, {3, 7, 0}, {0, 1, 2}, {2, 7, 0}, {6, 2, 0}, {1, 5, 2}, {5, 2, 0},   // +75.967s 75968
{7, 5, 0}, {0, 0, 2}, {4, 0, 0}, {6, 1, 0}, {6, 6, 0}, {7, 0, 0}, {7, 4, 0}, {7, 6, 0}, {5, 1, 2}, {1, 0, 2}, 
{6, 4, 0}, {0, 6, 2}, {6, 0, 0}, {6, 5, 0}, {7, 1, 0}, {4, 1, 2}, {5, 0, 0}, {1, 1, 2}, {2, 0, 0}, {5, 7, 0}, 
{6, 3, 2}, {6, 5, 0}, {7, 3, 0}, {0, 5, 3}, {1, 2, 0}, {4, 2, 0}, {4, 6, 0}, {4, 7, 0}, {5, 6, 0}, {2, 2, 2}, 
{2, 3, 0}, {1, 3, 2}, {2, 1, 0}, {6, 2, 0}, {6, 6, 0}, {7, 0, 0}, {0, 4, 2}, {4, 3, 2}, {5, 5, 0}, {6, 7, 0}, 
{7, 1, 0}, {1, 4, 2}, {1, 6, 0}, {6, 1, 2}, {7, 4, 0}, {7, 5, 0}, {7, 6, 0}, {0, 3, 2}, {2, 4, 0}, {5, 4, 0},   // +76.467s 76464
{6, 5, 0}, {7, 3, 0}, {1, 5, 2}, {6, 0, 2}, {6, 4, 0}, {0, 2, 2}, {1, 0, 0}, {2, 5, 0}, {5, 3, 0}, {2, 0, 2}, 
{4, 6, 0}, {6, 6, 0}, {7, 0, 0}, {2, 6, 3}, {6, 3, 0}, {1, 1, 2}, {2, 1, 0}, {7, 2, 0}, {0, 1, 2}, {2, 2, 0}, 
{2, 3, 0}, {4, 4, 0}, {5, 2, 0}, {6, 2, 0}, {6, 7, 0}, {7, 1, 0}, {1, 2, 4}, {7, 5, 0}, {0, 0, 2}, {4, 5, 0}, 
{6, 1, 0}, {6, 7, 0}, {7, 3, 0}, {7, 4, 0}, {7, 6, 0}, {5, 1, 2}, {6, 0, 2}, {6, 4, 0}, {6, 6, 0}, {7, 0, 0}, 
{0, 0, 2}, {1, 3, 0}, {3, 0, 0}, {5, 0, 2}, {0, 1, 2}, {2, 0, 0}, {3, 1, 0}, {6, 3, 0}, {6, 5, 0}, {7, 1, 0},   // +76.967s 76960
{1, 4, 3}, {4, 7, 2}, {5, 6, 0}, {0, 2, 2}, {3, 2, 0}, {6, 7, 0}, {7, 3, 0}, {2, 1, 2}, {2, 2, 0}, {2, 3, 0}, 
{5, 5, 0}, {6, 2, 0}, {0, 3, 2}, {1, 5, 0}, {1, 7, 0}, {3, 3, 2}, {6, 6, 0}, {7, 0, 0}, {5, 4, 2}, {6, 1, 0}, 
{7, 5, 0}, {0, 4, 2}, {1, 0, 0}, {3, 4, 0}, {7, 4, 0}, {7, 6, 0}, {5, 3, 2}, {0, 5, 2}, {3, 5, 0}, {6, 4, 0}, 
{6, 5, 0}, {7, 1, 0}, {1, 1, 2}, {6, 0, 0}, {2, 7, 2}, {5, 2, 0}, {0, 6, 2}, {1, 2, 0}, {2, 0, 0}, {3, 6, 0}, 
{6, 5, 0}, {7, 3, 0}, {4, 6, 3}, {4, 7, 0}, {5, 1, 0}, {6, 3, 0}, {2, 1, 2}, {3, 7, 0}, {6, 6, 0}, {7, 0, 0},   // +77.467s 77472
{0, 0, 2}, {1, 3, 0}, {2, 2, 0}, {2, 3, 0}, {5, 0, 0}, {6, 2, 0}, {1, 4, 4}, {4, 0, 0}, {6, 7, 0}, {7, 1, 0}, 
{7, 5, 0}, {0, 1, 2}, {5, 6, 0}, {6, 1, 0}, {4, 1, 2}, {5, 7, 0}, {7, 4, 0}, {7, 6, 0}, {1, 5, 2}, {6, 0, 0}, 
{6, 4, 0}, {6, 5, 0}, {7, 3, 0}, {0, 2, 4}, {4, 2, 0}, {5, 5, 0}, {1, 0, 2}, {2, 0, 0}, {6, 3, 0}, {6, 6, 0}, 
{7, 0, 0}, {0, 3, 5}, {4, 3, 0}, {0, 4, 6}, {1, 6, 0}, {2, 2, 0}, {4, 7, 0}, {6, 2, 0}, {6, 7, 0}, {7, 1, 0}, 
{1, 1, 2}, {2, 1, 0}, {2, 3, 0}, {2, 4, 0}, {5, 4, 0}, {1, 2, 4}, {6, 7, 0}, {7, 3, 0}, {0, 5, 2}, {5, 3, 0},   // +78.067s 78064
{6, 1, 0}, {7, 4, 0}, {7, 5, 0}, {7, 6, 0}, {2, 5, 2}, {6, 6, 0}, {7, 0, 2}, {1, 3, 2}, {6, 0, 0}, {6, 4, 0}, 
{7, 2, 0}, {0, 6, 2}, {2, 6, 0}, {5, 2, 0}, {6, 5, 0}, {7, 1, 0}, {2, 0, 3}, {6, 3, 2}, {0, 6, 2}, {1, 4, 0}, 
{2, 1, 0}, {6, 7, 0}, {7, 3, 0}, {2, 2, 2}, {2, 3, 0}, {4, 4, 0}, {4, 6, 0}, {5, 1, 0}, {0, 5, 2}, {6, 2, 0}, 
{1, 5, 2}, {6, 6, 0}, {7, 5, 0}, {4, 5, 2}, {5, 0, 0}, {6, 1, 0}, {7, 0, 0}, {0, 4, 2}, {7, 4, 0}, {7, 6, 0}, 
{5, 6, 2}, {6, 4, 0}, {0, 3, 2}, {1, 0, 0}, {3, 0, 0}, {6, 0, 0}, {6, 5, 0}, {7, 1, 0}, {2, 0, 5}, {5, 5, 0},   // +78.600s 78608
{0, 2, 2}, {1, 1, 0}, {6, 3, 0}, {6, 5, 0}, {7, 3, 0}, {1, 7, 2}, {3, 1, 0}, {5, 4, 0}, {0, 1, 2}, {1, 2, 0}, 
{6, 6, 0}, {2, 1, 2}, {2, 2, 0}, {6, 2, 0}, {7, 0, 0}, {2, 3, 2}, {4, 6, 0}, {4, 7, 0}, {5, 3, 0}, {6, 7, 0}, 
{0, 0, 2}, {1, 3, 0}, {3, 2, 0}, {7, 1, 0}, {5, 2, 2}, {6, 1, 0}, {7, 5, 0}, {1, 4, 2}, {6, 5, 0}, {7, 4, 0}, 
{7, 6, 0}, {0, 6, 2}, {2, 7, 0}, {3, 3, 0}, {7, 3, 0}, {5, 1, 2}, {6, 4, 0}, {1, 5, 2}, {6, 0, 0}, {0, 5, 3}, 
{5, 0, 0}, {6, 6, 0}, {7, 0, 0}, {1, 0, 2}, {2, 0, 0}, {3, 4, 0}, {6, 3, 2}, {0, 4, 2}, {5, 6, 0}, {6, 7, 0},   // +79.100s 79104
{1, 1, 2}, {2, 1, 0}, {2, 3, 0}, {3, 5, 0}, {6, 2, 0}, {7, 1, 0}, {2, 2, 2}, {4, 6, 0}, {5, 7, 0}, {6, 7, 2}, 
{7, 5, 0}, {0, 3, 2}, {1, 2, 0}, {5, 5, 0}, {6, 1, 0}, {7, 3, 0}, {3, 6, 2}, {7, 4, 0}, {7, 6, 0}, {6, 0, 2}, 
{6, 4, 0}, {6, 6, 0}, {7, 0, 0}, {0, 2, 2}, {1, 3, 0}, {5, 4, 0}, {1, 6, 2}, {3, 7, 0}, {6, 5, 0}, {2, 0, 2}, 
{6, 3, 0}, {7, 1, 0}, {0, 1, 5}, {1, 4, 0}, {5, 3, 0}, {6, 7, 0}, {2, 1, 2}, {4, 0, 0}, {7, 3, 0}, {6, 2, 2}, 
{0, 0, 2}, {2, 2, 0}, {4, 7, 0}, {5, 2, 0}, {1, 5, 2}, {2, 3, 0}, {4, 1, 0}, {6, 6, 0}, {7, 0, 0}, {6, 1, 2},   // +79.633s 79632
{7, 2, 0}, {7, 5, 0}, {0, 0, 2}, {5, 1, 0}, {1, 0, 2}, {6, 5, 0}, {7, 4, 0}, {7, 6, 0}, {4, 2, 2}, {6, 0, 0}, 
{6, 4, 0}, {7, 1, 0}, {0, 1, 2}, {1, 1, 3}, {2, 0, 0}, {5, 0, 0}, {6, 5, 0}, {7, 3, 0}, {0, 2, 2}, {4, 3, 0}, 
{6, 3, 0}, {6, 6, 2}, {0, 3, 2}, {1, 2, 0}, {2, 1, 0}, {5, 6, 0}, {7, 0, 0}, {2, 4, 2}, {6, 2, 0}, {1, 3, 2}, 
{2, 3, 0}, {5, 5, 0}, {6, 7, 0}, {7, 1, 0}, {7, 5, 0}, {0, 4, 2}, {2, 2, 0}, {2, 5, 0}, {4, 6, 0}, {4, 7, 0}, 
{6, 1, 0}, {1, 4, 2}, {0, 5, 2}, {5, 4, 0}, {6, 4, 0}, {6, 5, 0}, {7, 3, 0}, {1, 7, 2}, {2, 6, 0}, {6, 0, 0},   // +80.100s 80096
{7, 4, 0}, {7, 6, 0}, {1, 5, 2}, {5, 3, 0}, {0, 6, 2}, {2, 0, 0}, {6, 6, 0}, {1, 0, 3}, {4, 4, 0}, {6, 3, 0}, 
{7, 0, 0}, {5, 2, 2}, {0, 0, 2}, {4, 5, 0}, {1, 1, 2}, {2, 1, 0}, {5, 1, 0}, {6, 2, 0}, {6, 7, 0}, {7, 1, 0}, 
{3, 0, 2}, {0, 1, 2}, {1, 2, 6}, {2, 2, 0}, {2, 3, 0}, {2, 7, 0}, {3, 1, 0}, {4, 7, 0}, {6, 7, 0}, {7, 3, 0}, 
{0, 2, 2}, {5, 0, 0}, {6, 1, 0}, {7, 5, 0}, {6, 6, 2}, {7, 4, 0}, {7, 6, 0}, {3, 2, 2}, {5, 6, 0}, {7, 0, 0}, 
{0, 3, 2}, {1, 3, 0}, {6, 0, 0}, {6, 4, 0}, {3, 3, 3}, {6, 5, 0}, {7, 1, 0}, {2, 0, 2}, {5, 5, 2}, {5, 7, 0},   // +80.700s 80704
{6, 3, 0}, {6, 7, 0}, {0, 4, 2}, {1, 4, 0}, {2, 1, 0}, {3, 4, 0}, {7, 3, 0}, {6, 2, 2}, {5, 4, 2}, {0, 5, 2}, 
{1, 5, 0}, {3, 5, 0}, {6, 6, 0}, {7, 0, 0}, {7, 5, 0}, {2, 3, 2}, {4, 6, 0}, {6, 1, 0}, {2, 2, 2}, {3, 6, 0}, 
{7, 4, 0}, {7, 6, 0}, {0, 6, 2}, {5, 3, 0}, {6, 0, 0}, {6, 4, 0}, {6, 5, 0}, {1, 0, 2}, {1, 6, 0}, {3, 7, 0}, 
{7, 1, 0}, {0, 6, 5}, {1, 1, 0}, {2, 0, 0}, {5, 2, 0}, {6, 3, 0}, {6, 5, 0}, {4, 0, 2}, {7, 3, 0}, {0, 5, 4}, 
{1, 2, 0}, {6, 6, 0}, {7, 0, 0}, {2, 1, 2}, {4, 1, 0}, {5, 1, 0}, {6, 2, 0}, {0, 4, 2}, {1, 3, 0}, {6, 7, 0},   // +81.200s 81200
{7, 2, 0}, {4, 2, 2}, {7, 1, 0}, {5, 0, 2}, {6, 1, 0}, {7, 5, 0}, {0, 3, 2}, {1, 4, 0}, {2, 2, 0}, {2, 3, 0}, 
{4, 6, 0}, {4, 7, 0}, {6, 5, 0}, {4, 3, 2}, {7, 3, 0}, {7, 4, 0}, {7, 6, 0}, {0, 2, 2}, {1, 5, 0}, {6, 4, 0}, 
{5, 6, 2}, {6, 0, 0}, {2, 4, 3}, {6, 6, 0}, {7, 0, 0}, {0, 1, 2}, {1, 0, 0}, {2, 0, 0}, {5, 5, 0}, {6, 3, 2}, 
{0, 0, 2}, {2, 1, 0}, {2, 5, 0}, {5, 4, 0}, {6, 7, 0}, {1, 1, 2}, {6, 2, 0}, {7, 1, 0}, {0, 6, 4}, {5, 3, 0}, 
{6, 1, 0}, {6, 7, 0}, {7, 5, 0}, {1, 2, 2}, {2, 6, 0}, {7, 3, 0}, {1, 7, 2}, {2, 2, 0}, {2, 3, 0}, {4, 6, 0},   // +81.700s 81696
{5, 2, 0}, {6, 4, 0}, {6, 6, 0}, {6, 0, 2}, {7, 0, 0}, {7, 4, 0}, {7, 6, 0}, {0, 5, 2}, {1, 3, 0}, {4, 4, 0}, 
{5, 1, 2}, {6, 5, 0}, {7, 1, 0}, {2, 0, 3}, {6, 3, 0}, {0, 4, 2}, {4, 5, 0}, {5, 0, 0}, {1, 4, 2}, {6, 7, 0}, 
{7, 3, 0}, {2, 1, 2}, {2, 7, 0}, {6, 2, 0}, {0, 3, 2}, {5, 6, 0}, {3, 0, 2}, {6, 6, 0}, {1, 5, 2}, {7, 0, 0}, 
{6, 1, 2}, {7, 5, 0}, {0, 2, 2}, {3, 1, 0}, {4, 7, 0}, {5, 5, 0}, {1, 0, 2}, {2, 2, 0}, {2, 3, 0}, {6, 5, 0}, 
{7, 1, 0}, {6, 0, 2}, {6, 4, 0}, {7, 4, 0}, {7, 6, 0}, {0, 1, 2}, {5, 4, 0}, {5, 7, 0}, {1, 1, 3}, {2, 0, 0},   // +82.233s 82240
{3, 2, 0}, {6, 5, 0}, {7, 3, 0}, {6, 3, 2}, {1, 2, 2}, {2, 1, 0}, {6, 6, 0}, {0, 0, 2}, {3, 3, 0}, {5, 3, 0}, 
{7, 0, 0}, {6, 2, 2}, {1, 3, 2}, {1, 6, 0}, {6, 7, 0}, {7, 1, 0}, {7, 5, 0}, {0, 0, 2}, {5, 2, 0}, {6, 1, 0}, 
{1, 4, 2}, {3, 4, 0}, {0, 1, 2}, {2, 3, 0}, {6, 4, 0}, {6, 5, 0}, {7, 3, 0}, {2, 2, 2}, {4, 6, 0}, {4, 7, 0}, 
{5, 1, 0}, {6, 0, 0}, {1, 5, 2}, {3, 5, 0}, {7, 4, 0}, {7, 6, 0}, {0, 2, 3}, {2, 0, 0}, {6, 6, 0}, {1, 0, 2}, 
{6, 3, 0}, {7, 0, 0}, {0, 3, 2}, {5, 0, 0}, {7, 2, 0}, {3, 6, 2}, {6, 7, 0}, {1, 1, 2}, {2, 1, 0}, {6, 2, 0},   // +82.733s 82736
{7, 1, 0}, {0, 4, 2}, {5, 6, 0}, {3, 7, 2}, {6, 7, 0}, {0, 5, 2}, {1, 2, 0}, {5, 5, 0}, {6, 1, 0}, {7, 3, 0}, 
{7, 5, 0}, {4, 0, 4}, {6, 6, 0}, {6, 5, 6}, {0, 6, 3}, {1, 3, 0}, {2, 2, 0}, {2, 3, 0}, {4, 7, 0}, {5, 4, 0}, 
{6, 0, 0}, {6, 4, 0}, {7, 0, 0}, {7, 4, 0}, {7, 6, 0}, {4, 1, 2}, {2, 0, 2}, {6, 7, 0}, {7, 1, 0}, {0, 0, 2}, 
{1, 4, 0}, {5, 3, 0}, {6, 3, 0}, {2, 1, 2}, {4, 2, 0}, {5, 2, 2}, {7, 3, 0}, {0, 1, 2}, {1, 7, 0}, {6, 2, 0}, 
{6, 6, 0}, {1, 5, 2}, {7, 5, 0}, {4, 3, 2}, {5, 1, 0}, {6, 1, 0}, {6, 5, 2}, {7, 0, 0}, {0, 2, 2}, {1, 0, 0},   // +83.367s 83360
{5, 0, 0}, {6, 4, 0}, {2, 4, 3}, {6, 0, 0}, {2, 2, 2}, {2, 3, 0}, {4, 6, 0}, {6, 5, 0}, {7, 1, 0}, {7, 4, 0}, 
{7, 6, 0}, {0, 3, 2}, {1, 1, 0}, {2, 5, 0}, {5, 6, 0}, {2, 0, 2}, {2, 7, 0}, {6, 3, 0}, {1, 2, 2}, {6, 6, 0}, 
{7, 3, 0}, {2, 6, 2}, {5, 5, 0}, {0, 4, 2}, {2, 1, 0}, {6, 2, 0}, {6, 7, 0}, {1, 3, 2}, {4, 4, 0}, {7, 0, 0}, 
{0, 5, 4}, {1, 4, 0}, {4, 5, 0}, {5, 4, 0}, {6, 1, 0}, {6, 5, 0}, {7, 1, 0}, {5, 7, 2}, {7, 5, 0}, {1, 5, 5}, 
{3, 0, 0}, {6, 4, 0}, {6, 6, 0}, {7, 3, 0}, {0, 6, 2}, {2, 2, 0}, {5, 3, 0}, {6, 0, 0}, {2, 3, 2}, {4, 6, 0},   // +83.867s 83872
{4, 7, 0}, {7, 4, 0}, {7, 6, 0}, {0, 6, 2}, {1, 0, 0}, {2, 0, 0}, {3, 1, 0}, {7, 0, 0}, {5, 2, 2}, {6, 3, 0}, 
{6, 7, 0}, {2, 1, 2}, {3, 2, 0}, {0, 5, 2}, {1, 1, 0}, {1, 6, 0}, {6, 2, 0}, {5, 1, 2}, {6, 7, 0}, {7, 1, 0}, 
{0, 4, 2}, {1, 2, 0}, {3, 3, 0}, {7, 5, 0}, {6, 1, 2}, {6, 6, 0}, {7, 3, 2}, {0, 3, 2}, {3, 4, 0}, {5, 0, 0}, 
{6, 0, 0}, {6, 4, 0}, {1, 3, 3}, {6, 5, 0}, {7, 0, 0}, {0, 2, 2}, {2, 3, 0}, {3, 5, 0}, {2, 0, 2}, {2, 2, 0}, 
{4, 6, 0}, {5, 6, 0}, {6, 3, 0}, {7, 2, 0}, {7, 4, 0}, {7, 6, 0}, {1, 4, 2}, {3, 6, 0}, {6, 7, 0}, {7, 1, 0},   // +84.300s 84304
{0, 1, 2}, {2, 1, 2}, {5, 5, 0}, {0, 0, 2}, {3, 7, 0}, {6, 2, 0}, {6, 6, 0}, {7, 3, 0}, {1, 5, 2}, {5, 4, 0}, 
{4, 0, 2}, {0, 6, 2}, {5, 3, 0}, {6, 1, 0}, {7, 0, 0}, {7, 5, 0}, {4, 1, 2}, {6, 5, 0}, {1, 0, 2}, {5, 2, 2}, 
{6, 4, 0}, {0, 5, 3}, {4, 2, 0}, {6, 0, 0}, {6, 5, 0}, {7, 1, 0}, {1, 1, 2}, {2, 0, 0}, {2, 2, 0}, {2, 3, 0}, 
{4, 7, 0}, {5, 1, 0}, {6, 3, 2}, {6, 6, 0}, {7, 4, 0}, {7, 6, 0}, {0, 4, 2}, {1, 2, 0}, {4, 3, 0}, {7, 3, 0}, 
{1, 7, 2}, {2, 1, 0}, {5, 0, 0}, {6, 7, 0}, {1, 3, 2}, {6, 2, 0}, {7, 0, 0}, {2, 4, 2}, {7, 5, 0}, {0, 3, 2},   // +84.867s 84864
{5, 6, 0}, {6, 1, 0}, {6, 5, 0}, {7, 1, 0}, {1, 4, 2}, {6, 4, 2}, {0, 2, 2}, {2, 5, 0}, {5, 5, 0}, {6, 0, 0}, 
{7, 3, 0}, {1, 5, 3}, {2, 7, 0}, {6, 6, 0}, {0, 1, 4}, {1, 0, 0}, {2, 0, 0}, {2, 3, 0}, {2, 6, 0}, {6, 3, 0}, 
{7, 4, 0}, {7, 6, 0}, {2, 2, 2}, {4, 6, 0}, {4, 7, 0}, {5, 4, 0}, {6, 7, 0}, {7, 0, 0}, {1, 1, 4}, {2, 1, 0}, 
{4, 4, 0}, {6, 2, 0}, {0, 0, 2}, {5, 3, 0}, {6, 7, 0}, {7, 1, 0}, {5, 7, 2}, {1, 2, 2}, {0, 0, 2}, {4, 5, 0}, 
{6, 1, 0}, {6, 6, 0}, {7, 3, 0}, {7, 5, 0}, {5, 2, 2}, {0, 1, 2}, {6, 5, 0}, {1, 3, 2}, {3, 0, 0}, {6, 0, 0},   // +85.400s 85392
{6, 4, 0}, {7, 0, 0}, {5, 1, 3}, {0, 2, 2}, {1, 6, 0}, {2, 0, 0}, {2, 2, 0}, {4, 7, 0}, {6, 7, 0}, {7, 1, 0}, 
{1, 4, 2}, {2, 3, 0}, {6, 3, 0}, {7, 4, 0}, {7, 6, 0}, {0, 3, 2}, {2, 1, 0}, {3, 1, 0}, {5, 0, 0}, {7, 3, 2}, 
{6, 2, 2}, {6, 6, 0}, {0, 4, 2}, {1, 5, 0}, {3, 2, 0}, {5, 6, 0}, {7, 5, 0}, {6, 1, 2}, {0, 5, 2}, {6, 5, 0}, 
{7, 0, 0}, {1, 0, 2}, {5, 5, 0}, {6, 0, 0}, {6, 4, 0}, {7, 2, 0}, {3, 3, 2}, {0, 6, 2}, {5, 4, 0}, {6, 5, 0}, 
{7, 1, 0}, {1, 1, 3}, {6, 3, 0}, {2, 0, 2}, {3, 4, 0}, {6, 6, 0}, {0, 0, 2}, {1, 2, 0}, {2, 2, 0}, {2, 3, 0},   // +85.900s 85904
{4, 6, 0}, {5, 3, 0}, {7, 3, 0}, {7, 4, 0}, {7, 6, 0}, {2, 1, 4}, {5, 2, 0}, {6, 2, 0}, {6, 7, 0}, {7, 0, 0}, 
{0, 1, 2}, {1, 3, 0}, {3, 5, 0}, {1, 4, 4}, {5, 1, 0}, {6, 1, 0}, {6, 5, 0}, {7, 1, 0}, {7, 5, 0}, {0, 2, 2}, 
{3, 6, 0}, {5, 0, 2}, {1, 5, 2}, {6, 4, 0}, {6, 6, 0}, {7, 3, 0}, {1, 7, 2}, {3, 7, 0}, {6, 0, 0}, {0, 3, 3}, 
{1, 0, 0}, {2, 0, 0}, {5, 6, 0}, {7, 0, 2}, {2, 2, 2}, {2, 3, 0}, {6, 3, 0}, {6, 7, 0}, {0, 4, 2}, {1, 1, 0}, 
{2, 1, 0}, {4, 0, 0}, {4, 6, 0}, {4, 7, 0}, {7, 4, 0}, {7, 6, 0}, {5, 5, 2}, {6, 2, 0}, {6, 7, 2}, {7, 1, 0},   // +86.400s 86400
{1, 2, 2}, {4, 1, 0}, {7, 5, 0}, {0, 5, 2}, {2, 7, 13}, {6, 1, 0}, {6, 4, 0}, {6, 6, 0}, {7, 3, 0}, {0, 6, 2}, 
{1, 3, 0}, {4, 2, 0}, {5, 4, 0}, {2, 3, 6}, {6, 5, 0}, {7, 0, 0}, {7, 4, 0}, {7, 6, 0}, {1, 4, 2}, {2, 0, 0}, 
{2, 2, 0}, {0, 6, 4}, {4, 6, 0}, {6, 0, 0}, {4, 3, 2}, {5, 3, 0}, {0, 5, 7}, {1, 5, 0}, {5, 7, 0}, {6, 7, 0}, 
{7, 1, 0}, {2, 1, 2}, {5, 2, 0}, {6, 3, 0}, {0, 4, 4}, {2, 4, 0}, {7, 3, 0}, {1, 0, 2}, {6, 2, 0}, {6, 6, 0}, 
{7, 5, 0}, {0, 3, 2}, {2, 2, 0}, {2, 3, 0}, {2, 5, 0}, {5, 1, 0}, {7, 4, 2}, {7, 6, 0}, {1, 1, 2}, {1, 6, 0},   // +87.267s 87264
{2, 6, 0}, {6, 4, 0}, {6, 5, 0}, {7, 0, 0}, {0, 2, 2}, {5, 0, 0}, {6, 1, 0}, {4, 7, 2}, {0, 1, 2}, {1, 2, 0}, 
{2, 0, 0}, {4, 4, 0}, {6, 5, 0}, {7, 1, 0}, {5, 6, 2}, {6, 0, 0}, {1, 3, 3}, {2, 1, 0}, {0, 0, 2}, {4, 5, 0}, 
{6, 6, 0}, {7, 3, 0}, {5, 5, 2}, {6, 3, 0}, {1, 4, 2}, {3, 0, 0}, {6, 7, 0}, {7, 2, 0}, {7, 5, 0}, {0, 6, 2}, 
{5, 4, 0}, {7, 0, 0}, {1, 5, 2}, {2, 3, 0}, {6, 2, 0}, {2, 2, 2}, {3, 1, 0}, {5, 3, 0}, {6, 4, 0}, {6, 5, 0}, 
{7, 1, 0}, {7, 4, 0}, {7, 6, 0}, {0, 5, 2}, {6, 1, 0}, {1, 0, 2}, {3, 2, 0}, {2, 0, 2}, {4, 6, 0}, {4, 7, 0},   // +87.733s 87728
{5, 2, 0}, {6, 0, 0}, {6, 6, 0}, {7, 3, 0}, {3, 3, 2}, {0, 4, 2}, {1, 1, 0}, {5, 1, 0}, {2, 1, 3}, {6, 3, 0}, 
{7, 0, 0}, {3, 4, 2}, {6, 7, 0}, {0, 3, 2}, {1, 2, 0}, {5, 0, 0}, {3, 5, 4}, {6, 2, 0}, {6, 7, 0}, {7, 1, 0}, 
{7, 5, 0}, {0, 2, 2}, {1, 3, 0}, {5, 6, 0}, {1, 7, 2}, {2, 2, 0}, {2, 3, 0}, {3, 6, 0}, {6, 6, 0}, {7, 4, 0}, 
{7, 6, 0}, {6, 1, 2}, {6, 4, 0}, {7, 3, 0}, {1, 4, 2}, {5, 5, 0}, {0, 1, 2}, {2, 0, 0}, {3, 7, 0}, {4, 7, 0}, 
{6, 5, 0}, {7, 0, 0}, {4, 0, 5}, {6, 0, 0}, {0, 0, 2}, {1, 5, 0}, {2, 1, 0}, {5, 4, 0}, {6, 7, 0}, {7, 1, 0},   // +88.233s 88240
{2, 7, 2}, {4, 1, 0}, {6, 3, 2}, {7, 5, 0}, {0, 0, 2}, {5, 3, 0}, {6, 6, 0}, {7, 3, 0}, {1, 0, 2}, {4, 2, 0}, 
{6, 2, 0}, {6, 4, 2}, {0, 1, 2}, {2, 2, 0}, {2, 3, 0}, {4, 3, 0}, {7, 0, 0}, {7, 4, 0}, {7, 6, 0}, {1, 1, 2}, 
{5, 2, 0}, {6, 1, 0}, {6, 5, 0}, {0, 2, 2}, {2, 0, 0}, {1, 2, 2}, {4, 6, 0}, {5, 7, 0}, {6, 0, 0}, {2, 4, 2}, 
{5, 1, 0}, {6, 5, 0}, {7, 1, 0}, {0, 3, 2}, {1, 3, 0}, {2, 1, 3}, {6, 3, 0}, {6, 6, 0}, {0, 4, 2}, {5, 0, 0}, 
{7, 3, 0}, {1, 4, 2}, {2, 5, 0}, {6, 7, 0}, {1, 6, 2}, {6, 2, 0}, {7, 0, 0}, {0, 5, 2}, {5, 6, 0}, {7, 5, 0},   // +88.767s 88768
{1, 5, 2}, {2, 6, 0}, {6, 5, 0}, {7, 1, 0}, {0, 6, 2}, {2, 2, 0}, {6, 4, 0}, {1, 0, 2}, {2, 3, 0}, {5, 5, 0}, 
{6, 1, 0}, {7, 4, 0}, {7, 6, 0}, {4, 4, 2}, {7, 3, 0}, {0, 0, 2}, {2, 0, 0}, {5, 4, 0}, {6, 6, 0}, {1, 1, 2}, 
{4, 6, 0}, {4, 7, 0}, {6, 0, 0}, {2, 1, 2}, {7, 2, 0}, {0, 1, 3}, {4, 5, 0}, {5, 3, 0}, {6, 7, 0}, {7, 0, 0}, 
{1, 2, 2}, {6, 3, 0}, {5, 2, 2}, {7, 5, 0}, {3, 0, 2}, {6, 7, 0}, {7, 1, 0}, {0, 2, 2}, {1, 3, 0}, {6, 2, 0}, 
{5, 1, 2}, {6, 4, 0}, {2, 3, 2}, {6, 1, 0}, {6, 6, 0}, {7, 3, 0}, {0, 3, 2}, {2, 2, 0}, {3, 1, 0}, {5, 0, 0},   // +89.267s 89264
{7, 4, 0}, {7, 6, 0}, {1, 4, 2}, {2, 0, 0}, {6, 5, 0}, {6, 0, 2}, {7, 0, 0}, {3, 2, 2}, {4, 6, 0}, {5, 6, 0}, 
{0, 4, 3}, {6, 7, 0}, {7, 1, 0}, {1, 5, 2}, {2, 1, 0}, {6, 3, 0}, {1, 7, 2}, {0, 5, 2}, {3, 3, 0}, {5, 5, 0}, 
{7, 3, 0}, {1, 0, 2}, {6, 2, 0}, {6, 6, 0}, {7, 5, 0}, {3, 4, 4}, {5, 4, 0}, {0, 6, 2}, {1, 1, 0}, {6, 4, 0}, 
{6, 5, 0}, {7, 0, 0}, {2, 2, 2}, {2, 3, 0}, {6, 1, 0}, {1, 2, 2}, {2, 0, 0}, {5, 3, 0}, {7, 4, 0}, {7, 6, 0}, 
{0, 6, 2}, {2, 7, 0}, {3, 5, 0}, {6, 5, 0}, {7, 1, 0}, {4, 7, 2}, {6, 0, 0}, {0, 5, 2}, {1, 3, 0}, {2, 1, 0},   // +89.800s 89792
{6, 6, 0}, {3, 6, 3}, {5, 2, 0}, {7, 3, 0}, {0, 4, 2}, {1, 4, 0}, {6, 3, 0}, {7, 5, 0}, {6, 7, 2}, {7, 0, 0}, 
{5, 1, 2}, {6, 2, 0}, {0, 3, 2}, {1, 5, 0}, {3, 7, 0}, {5, 7, 0}, {6, 4, 0}, {6, 5, 2}, {7, 1, 0}, {0, 2, 2}, 
{1, 0, 0}, {6, 1, 0}, {2, 3, 2}, {4, 0, 0}, {5, 0, 0}, {7, 4, 0}, {7, 6, 0}, {2, 0, 2}, {2, 2, 0}, {6, 0, 0}, 
{6, 6, 0}, {7, 3, 0}, {0, 1, 2}, {1, 1, 0}, {5, 6, 2}, {0, 0, 2}, {2, 1, 0}, {4, 1, 0}, {4, 7, 0}, {6, 3, 0}, 
{7, 0, 0}, {1, 2, 3}, {1, 6, 0}, {4, 6, 0}, {5, 5, 0}, {6, 7, 0}, {0, 6, 4}, {4, 2, 2}, {5, 4, 0}, {6, 2, 0},   // +90.333s 90336
{6, 7, 0}, {7, 1, 0}, {7, 5, 0}, {1, 3, 2}, {5, 3, 2}, {6, 6, 0}, {0, 5, 2}, {4, 3, 0}, {6, 1, 0}, {6, 4, 0}, 
{7, 3, 0}, {2, 2, 2}, {1, 4, 2}, {2, 0, 0}, {2, 3, 0}, {5, 2, 0}, {6, 5, 0}, {7, 0, 0}, {7, 2, 0}, {7, 4, 0}, 
{7, 6, 0}, {0, 4, 2}, {2, 4, 0}, {5, 1, 2}, {6, 0, 0}, {1, 5, 2}, {2, 1, 0}, {2, 5, 0}, {4, 7, 0}, {6, 7, 0}, 
{7, 1, 0}, {5, 0, 3}, {6, 3, 0}, {0, 3, 2}, {7, 5, 0}, {2, 6, 2}, {6, 6, 0}, {7, 3, 0}, {1, 0, 2}, {5, 6, 0}, 
{6, 2, 0}, {0, 2, 2}, {4, 4, 0}, {6, 4, 0}, {6, 1, 2}, {6, 5, 0}, {7, 0, 0}, {1, 1, 2}, {2, 0, 2}, {4, 5, 0},   // +90.867s 90864
{5, 5, 0}, {0, 1, 2}, {1, 2, 0}, {2, 2, 0}, {2, 3, 0}, {6, 0, 0}, {6, 5, 0}, {7, 1, 0}, {7, 4, 0}, {7, 6, 0}, 
{3, 0, 2}, {1, 3, 2}, {1, 7, 0}, {5, 4, 0}, {0, 0, 3}, {2, 1, 0}, {4, 6, 0}, {6, 3, 0}, {6, 6, 0}, {7, 3, 0}, 
{1, 4, 2}, {3, 1, 0}, {6, 7, 2}, {0, 0, 2}, {3, 2, 0}, {5, 3, 0}, {6, 2, 0}, {7, 0, 0}, {7, 5, 0}, {1, 5, 2}, 
{0, 1, 2}, {6, 5, 0}, {7, 1, 0}, {2, 7, 2}, {3, 3, 0}, {5, 2, 0}, {6, 4, 0}, {1, 0, 2}, {6, 1, 0}, {0, 2, 2}, 
{2, 0, 0}, {3, 4, 0}, {7, 3, 0}, {2, 2, 2}, {6, 6, 0}, {0, 3, 2}, {1, 1, 0}, {2, 3, 0}, {3, 5, 0}, {5, 1, 0},   // +91.333s 91328
{6, 0, 0}, {7, 4, 0}, {7, 6, 0}, {2, 1, 2}, {6, 7, 2}, {7, 0, 0}, {0, 4, 3}, {1, 2, 0}, {3, 6, 0}, {4, 6, 0}, 
{4, 7, 0}, {5, 0, 0}, {6, 3, 0}, {5, 7, 2}, {7, 5, 0}, {0, 5, 2}, {6, 7, 0}, {7, 1, 0}, {1, 3, 2}, {3, 7, 0}, 
{5, 6, 0}, {6, 2, 0}, {6, 4, 2}, {0, 6, 2}, {4, 0, 0}, {5, 5, 0}, {6, 1, 0}, {6, 6, 0}, {7, 3, 0}, {1, 4, 4}, 
{2, 0, 0}, {6, 0, 0}, {6, 5, 0}, {0, 0, 2}, {1, 6, 0}, {2, 3, 0}, {4, 1, 0}, {5, 4, 0}, {7, 0, 0}, {2, 2, 2}, 
{7, 4, 0}, {7, 6, 0}, {4, 2, 2}, {6, 3, 0}, {6, 7, 0}, {7, 1, 0}, {0, 1, 2}, {1, 5, 0}, {2, 1, 0}, {5, 3, 0},   // +91.800s 91792
{4, 6, 3}, {4, 3, 2}, {5, 2, 0}, {6, 6, 0}, {7, 3, 0}, {0, 2, 2}, {1, 0, 0}, {6, 2, 0}, {7, 5, 0}, {5, 1, 2}, 
{7, 2, 0}, {2, 4, 2}, {7, 0, 0}, {1, 1, 2}, {6, 1, 0}, {6, 4, 0}, {6, 5, 0}, {0, 3, 2}, {5, 0, 0}, {1, 2, 2}, 
{2, 0, 0}, {2, 5, 0}, {6, 5, 2}, {7, 1, 0}, {0, 4, 2}, {1, 3, 0}, {2, 1, 0}, {2, 2, 0}, {2, 3, 0}, {5, 6, 0}, 
{6, 0, 0}, {7, 4, 0}, {7, 6, 0}, {2, 6, 2}, {6, 6, 0}, {7, 3, 2}, {1, 4, 3}, {4, 7, 0}, {5, 5, 0}, {6, 3, 0}, 
{7, 5, 0}, {0, 5, 2}, {6, 7, 0}, {7, 0, 0}, {4, 4, 2}, {6, 2, 0}, {1, 5, 2}, {6, 4, 0}, {0, 6, 2}, {5, 4, 0},   // +92.367s 92368
{6, 5, 0}, {7, 1, 0}, {1, 0, 2}, {4, 5, 0}, {6, 1, 0}, {1, 7, 2}, {2, 0, 0}, {0, 6, 2}, {5, 3, 0}, {6, 0, 0}, 
{6, 6, 0}, {7, 3, 0}, {1, 1, 2}, {2, 2, 2}, {2, 3, 0}, {3, 0, 0}, {7, 4, 0}, {7, 6, 0}, {0, 5, 2}, {2, 1, 0}, 
{5, 2, 0}, {6, 3, 0}, {7, 0, 0}, {1, 2, 2}, {6, 7, 0}, {0, 4, 3}, {3, 1, 0}, {2, 7, 2}, {4, 6, 0}, {4, 7, 0}, 
{6, 2, 0}, {6, 7, 0}, {5, 1, 2}, {7, 1, 0}, {7, 5, 0}, {0, 3, 2}, {1, 3, 0}, {3, 2, 2}, {6, 4, 0}, {6, 6, 0}, 
{7, 3, 0}, {0, 2, 2}, {5, 0, 0}, {6, 1, 0}, {6, 5, 2}, {0, 1, 2}, {1, 4, 0}, {2, 0, 0}, {7, 0, 0}, {3, 3, 2},   // +92.900s 92896
{5, 6, 0}, {6, 0, 0}, {5, 7, 2}, {6, 7, 0}, {7, 1, 0}, {0, 0, 2}, {1, 5, 0}, {2, 1, 0}, {2, 2, 0}, {2, 3, 0}, 
{7, 4, 0}, {7, 6, 0}, {3, 4, 2}, {5, 5, 0}, {6, 3, 0}, {7, 3, 3}, {7, 5, 0}, {0, 6, 2}, {1, 0, 0}, {4, 7, 0}, 
{5, 4, 0}, {6, 6, 0}, {6, 2, 2}, {3, 5, 2}, {6, 4, 0}, {0, 5, 2}, {1, 1, 0}, {1, 6, 0}, {5, 3, 0}, {6, 1, 0}, 
{6, 5, 0}, {7, 0, 0}, {1, 2, 4}, {2, 0, 0}, {3, 6, 0}, {5, 2, 0}, {6, 0, 2}, {6, 5, 0}, {7, 1, 0}, {0, 4, 2}, 
{1, 3, 2}, {2, 1, 0}, {2, 3, 0}, {3, 7, 0}, {5, 1, 0}, {2, 2, 2}, {6, 3, 0}, {6, 6, 0}, {7, 3, 0}, {7, 4, 0},   // +93.367s 93360
{7, 6, 0}, {0, 3, 2}, {5, 0, 0}, {1, 4, 3}, {4, 0, 0}, {6, 7, 0}, {7, 2, 0}, {4, 6, 2}, {6, 2, 0}, {7, 0, 0}, 
{7, 5, 0}, {1, 5, 2}, {5, 6, 0}, {0, 2, 2}, {6, 5, 0}, {7, 1, 0}, {4, 1, 2}, {6, 4, 0}, {1, 0, 2}, {5, 5, 0}, 
{6, 1, 0}, {0, 1, 2}, {2, 0, 0}, {7, 3, 0}, {6, 6, 2}, {1, 1, 2}, {4, 2, 0}, {6, 0, 0}, {2, 1, 2}, {5, 4, 0}, 
{0, 0, 2}, {2, 2, 0}, {2, 3, 0}, {6, 7, 0}, {7, 0, 0}, {1, 2, 2}, {4, 3, 0}, {6, 3, 0}, {7, 4, 0}, {7, 5, 0}, 
{7, 6, 0}, {5, 3, 3}, {0, 0, 2}, {6, 2, 0}, {6, 7, 0}, {7, 1, 0}, {1, 3, 2}, {2, 4, 0}, {4, 6, 0}, {4, 7, 0},   // +93.900s 93904
{6, 4, 0}, {0, 1, 2}, {1, 7, 0}, {6, 6, 0}, {5, 2, 2}, {6, 1, 0}, {7, 3, 0}, {0, 2, 2}, {2, 5, 0}, {1, 4, 2}, 
{2, 0, 0}, {6, 0, 0}, {6, 5, 0}, {7, 0, 0}, {2, 6, 2}, {5, 1, 0}, {0, 3, 2}, {1, 5, 2}, {6, 3, 0}, {6, 7, 0}, 
{7, 1, 0}, {0, 4, 2}, {2, 1, 0}, {2, 2, 0}, {2, 3, 0}, {2, 7, 0}, {4, 4, 0}, {7, 4, 0}, {7, 6, 0}, {5, 0, 2}, 
{6, 6, 3}, {7, 3, 0}, {0, 5, 2}, {1, 0, 0}, {4, 5, 0}, {6, 2, 0}, {7, 5, 0}, {3, 0, 2}, {4, 6, 0}, {5, 6, 0}, 
{0, 6, 2}, {7, 0, 0}, {1, 1, 2}, {5, 5, 0}, {6, 1, 0}, {6, 4, 0}, {6, 5, 0}, {3, 1, 2}, {0, 0, 2}, {1, 2, 0},   // +94.433s 94432
{2, 0, 0}, {5, 7, 0}, {3, 2, 2}, {5, 4, 0}, {6, 5, 0}, {7, 1, 0}, {1, 3, 2}, {6, 0, 0}, {2, 1, 2}, {5, 3, 0}, 
{6, 6, 0}, {0, 1, 2}, {2, 2, 0}, {3, 3, 0}, {7, 3, 0}, {1, 4, 2}, {2, 3, 0}, {5, 2, 0}, {6, 3, 0}, {7, 4, 0}, 
{7, 5, 0}, {7, 6, 0}, {1, 6, 3}, {6, 7, 0}, {7, 0, 0}, {0, 2, 2}, {3, 4, 0}, {6, 2, 0}, {1, 5, 2}, {4, 7, 0}, 
{5, 1, 0}, {6, 4, 0}, {6, 5, 0}, {3, 5, 2}, {6, 1, 0}, {7, 1, 0}, {1, 0, 2}, {5, 0, 0}, {0, 3, 2}, {2, 0, 0}, 
{7, 3, 0}, {3, 6, 2}, {6, 0, 0}, {6, 6, 0}, {1, 1, 2}, {5, 6, 0}, {0, 4, 2}, {3, 7, 0}, {7, 2, 0}, {2, 1, 2},   // +94.933s 94928
{6, 3, 0}, {6, 7, 0}, {7, 0, 0}, {1, 2, 2}, {0, 5, 3}, {2, 2, 0}, {2, 3, 0}, {4, 0, 0}, {5, 5, 0}, {7, 4, 0}, 
{7, 6, 0}, {6, 2, 2}, {6, 7, 0}, {7, 1, 0}, {7, 5, 0}, {1, 3, 2}, {4, 1, 0}, {5, 4, 2}, {0, 6, 2}, {4, 6, 0}, 
{4, 7, 0}, {6, 4, 0}, {6, 6, 0}, {7, 3, 0}, {4, 2, 2}, {6, 1, 0}, {1, 4, 2}, {2, 0, 0}, {6, 5, 0}, {0, 6, 2}, 
{4, 3, 0}, {5, 3, 0}, {7, 0, 0}, {6, 0, 2}, {0, 5, 2}, {2, 1, 0}, {6, 7, 0}, {7, 1, 0}, {1, 5, 2}, {2, 4, 0}, 
{5, 2, 0}, {1, 7, 2}, {6, 3, 0}, {0, 4, 2}, {2, 2, 0}, {2, 3, 0}, {7, 3, 0}, {7, 5, 0}, {1, 0, 3}, {6, 6, 0},   // +95.433s 95440
{7, 4, 0}, {7, 6, 0}, {0, 3, 2}, {2, 5, 0}, {5, 1, 0}, {6, 2, 0}, {4, 7, 2}, {6, 4, 0}, {1, 1, 2}, {6, 1, 0}, 
{6, 5, 0}, {7, 0, 0}, {0, 2, 2}, {2, 6, 0}, {5, 0, 0}, {1, 2, 2}, {2, 0, 0}, {0, 1, 2}, {2, 7, 0}, {6, 0, 0}, 
{6, 5, 0}, {7, 1, 0}, {4, 4, 2}, {5, 6, 0}, {1, 3, 2}, {2, 1, 0}, {6, 3, 0}, {6, 6, 0}, {0, 0, 2}, {5, 5, 0}, 
{7, 3, 0}, {1, 4, 2}, {2, 3, 3}, {4, 5, 0}, {6, 7, 0}, {7, 0, 0}, {7, 4, 0}, {7, 6, 0}, {0, 6, 2}, {2, 2, 0}, 
{5, 4, 0}, {6, 2, 0}, {7, 5, 0}, {1, 5, 2}, {5, 7, 0}, {3, 0, 2}, {5, 3, 0}, {6, 5, 0}, {7, 1, 0}, {0, 5, 2},   // +95.933s 95936
{4, 6, 0}, {6, 1, 0}, {6, 4, 0}, {1, 0, 2}, {2, 0, 2}, {5, 2, 0}, {6, 6, 0}, {7, 3, 0}, {0, 4, 2}, {1, 1, 0}, 
{3, 1, 0}, {5, 1, 2}, {6, 0, 0}, {2, 1, 2}, {7, 0, 0}, {1, 2, 2}, {1, 6, 0}, {3, 2, 0}, {6, 7, 0}, {0, 3, 2}, 
{5, 0, 0}, {6, 3, 0}, {7, 5, 0}, {2, 2, 3}, {2, 3, 2}, {6, 2, 0}, {6, 7, 0}, {7, 1, 0}, {7, 4, 0}, {7, 6, 0}, 
{0, 2, 2}, {1, 3, 0}, {3, 3, 0}, {5, 6, 0}, {6, 4, 0}, {6, 6, 2}, {6, 1, 2}, {7, 3, 0}, {3, 4, 2}, {4, 6, 0}, 
{4, 7, 0}, {5, 5, 0}, {0, 1, 2}, {1, 4, 0}, {2, 0, 0}, {6, 0, 0}, {6, 5, 0}, {7, 0, 0}, {7, 2, 0}, {0, 0, 6},   // +96.500s 96496
{1, 5, 0}, {2, 1, 0}, {3, 5, 0}, {5, 4, 0}, {6, 3, 0}, {6, 7, 0}, {7, 1, 0}, {3, 6, 6}, {5, 3, 0}, {6, 6, 0}, 
{7, 3, 0}, {0, 0, 3}, {1, 0, 0}, {2, 2, 0}, {2, 3, 0}, {6, 2, 0}, {7, 4, 0}, {7, 5, 0}, {7, 6, 0}, {0, 1, 4}, 
{5, 2, 0}, {6, 4, 0}, {6, 5, 0}, {7, 0, 0}, {1, 1, 2}, {3, 7, 0}, {4, 6, 0}, {6, 1, 0}, {0, 2, 4}, {1, 2, 0}, 
{2, 0, 0}, {6, 5, 0}, {7, 1, 0}, {5, 1, 2}, {6, 0, 0}, {0, 3, 2}, {1, 3, 0}, {1, 7, 0}, {2, 1, 0}, {4, 0, 0}, 
{6, 6, 2}, {7, 3, 0}, {0, 4, 2}, {5, 0, 0}, {6, 3, 0}, {1, 4, 2}, {4, 1, 0}, {6, 7, 0}, {7, 5, 0}, {7, 0, 2},   // +97.000s 96992
{0, 5, 3}, {1, 5, 0}, {6, 2, 0}, {2, 2, 2}, {2, 3, 0}, {4, 2, 0}, {5, 6, 0}, {6, 4, 0}, {7, 1, 0}, {7, 4, 0}, 
{7, 6, 0}, {0, 6, 2}, {6, 1, 0}, {6, 5, 0}, {1, 0, 2}, {2, 7, 0}, {5, 5, 0}, {2, 0, 2}, {4, 7, 0}, {7, 3, 0}, 
{0, 0, 2}, {4, 3, 0}, {6, 0, 0}, {6, 6, 0}, {1, 1, 2}, {5, 4, 0}, {2, 1, 2}, {2, 4, 2}, {5, 3, 0}, {6, 3, 0}, 
{7, 0, 0}, {0, 1, 2}, {1, 2, 0}, {6, 7, 0}, {2, 5, 2}, {5, 7, 0}, {5, 2, 2}, {6, 2, 0}, {7, 1, 0}, {7, 5, 0}, 
{0, 2, 3}, {1, 3, 0}, {2, 3, 0}, {6, 7, 0}, {2, 2, 2}, {2, 6, 0}, {5, 1, 0}, {7, 4, 0}, {7, 6, 0}, {6, 4, 2},   // +97.500s 97504
{6, 6, 0}, {7, 3, 0}, {4, 4, 2}, {5, 0, 0}, {6, 1, 0}, {0, 3, 2}, {1, 4, 0}, {2, 0, 0}, {4, 6, 0}, {4, 7, 0}, 
{1, 6, 2}, {4, 5, 0}, {6, 5, 0}, {7, 0, 0}, {5, 6, 2}, {6, 0, 0}, {0, 4, 2}, {2, 1, 0}, {6, 7, 0}, {7, 1, 0}, 
{1, 5, 2}, {3, 0, 0}, {6, 3, 2}, {5, 5, 2}, {7, 3, 0}, {7, 5, 0}, {0, 5, 2}, {1, 0, 0}, {3, 1, 0}, {6, 2, 0}, 
{6, 6, 0}, {6, 4, 3}, {7, 2, 0}, {2, 2, 2}, {2, 3, 0}, {3, 2, 0}, {5, 4, 0}, {7, 0, 0}, {7, 4, 0}, {7, 6, 0}, 
{0, 6, 2}, {1, 1, 0}, {6, 1, 0}, {6, 5, 0}, {3, 3, 2}, {1, 2, 2}, {2, 0, 0}, {4, 7, 0}, {6, 0, 0}, {0, 6, 2},   // +98.000s 98000
{5, 3, 0}, {6, 5, 0}, {7, 1, 0}, {1, 3, 2}, {3, 4, 0}, {0, 5, 2}, {2, 1, 0}, {6, 3, 0}, {3, 5, 2}, {5, 2, 0}, 
{6, 6, 0}, {7, 3, 0}, {1, 4, 2}, {0, 4, 2}, {6, 7, 0}, {7, 0, 0}, {3, 6, 2}, {6, 2, 0}, {7, 5, 0}, {0, 3, 3}, 
{1, 5, 0}, {5, 1, 0}, {2, 2, 2}, {2, 3, 0}, {3, 7, 0}, {6, 5, 0}, {7, 1, 0}, {7, 4, 0}, {7, 6, 0}, {1, 0, 2}, 
{6, 1, 0}, {6, 4, 0}, {0, 2, 2}, {1, 7, 0}, {4, 0, 2}, {5, 0, 0}, {7, 3, 0}, {0, 1, 2}, {1, 1, 0}, {2, 0, 0}, 
{4, 6, 0}, {6, 6, 0}, {6, 0, 2}, {4, 1, 2}, {5, 6, 0}, {7, 0, 0}, {0, 0, 2}, {2, 1, 0}, {1, 2, 2}, {5, 5, 0},   // +98.533s 98528
{6, 3, 0}, {6, 7, 0}, {4, 2, 2}, {7, 5, 0}, {2, 7, 2}, {7, 1, 0}, {0, 6, 3}, {1, 3, 0}, {5, 4, 0}, {6, 2, 0}, 
{6, 7, 0}, {4, 3, 2}, {6, 4, 0}, {2, 2, 2}, {2, 3, 0}, {6, 1, 0}, {6, 6, 0}, {7, 3, 0}, {7, 4, 0}, {7, 6, 0}, 
{0, 5, 2}, {5, 3, 0}, {1, 4, 2}, {2, 0, 2}, {2, 4, 0}, {6, 0, 0}, {6, 5, 0}, {7, 0, 0}, {4, 6, 2}, {4, 7, 0}, 
{5, 2, 0}, {5, 7, 0}, {0, 4, 2}, {1, 5, 0}, {2, 1, 2}, {6, 7, 0}, {7, 1, 0}, {5, 1, 2}, {6, 3, 0}, {0, 3, 2}, 
{2, 5, 0}, {1, 0, 3}, {6, 6, 0}, {7, 3, 0}, {5, 0, 2}, {6, 2, 0}, {7, 5, 0}, {0, 2, 2}, {1, 1, 0}, {2, 6, 0},   // +99.067s 99072
{1, 6, 2}, {2, 3, 0}, {7, 0, 0}, {7, 4, 0}, {7, 6, 0}, {2, 2, 2}, {5, 6, 0}, {6, 4, 0}, {6, 5, 0}, {1, 2, 2}, 
{6, 1, 0}, {0, 1, 2}, {2, 0, 0}, {4, 4, 0}, {1, 3, 2}, {4, 6, 0}, {5, 5, 0}, {6, 5, 0}, {7, 1, 0}, {6, 0, 2}, 
{0, 0, 2}, {2, 1, 0}, {4, 5, 0}, {1, 4, 2}, {6, 6, 0}, {7, 2, 0}, {7, 3, 0}, {5, 4, 2}, {7, 5, 0}, {1, 5, 2}, 
{3, 0, 0}, {6, 3, 0}, {6, 7, 0}, {0, 0, 3}, {7, 0, 0}, {6, 2, 2}, {1, 0, 2}, {2, 2, 0}, {5, 3, 0}, {6, 4, 0}, 
{6, 5, 0}, {0, 1, 2}, {2, 3, 0}, {3, 1, 0}, {7, 1, 0}, {7, 4, 0}, {7, 6, 0}, {0, 1, 2}, {1, 1, 0}, {1, 2, 0},   // +99.567s 99568
{1, 3, 0}, {1, 4, 0}, {1, 5, 0}, {1, 7, 0}, {2, 1, 0}, {2, 2, 0}, {2, 7, 0}, {3, 0, 0}, {3, 2, 0}, {3, 3, 0}, 
{3, 4, 0}, {3, 5, 0}, {3, 6, 0}, {3, 7, 0}, {4, 1, 0}, {4, 2, 0}, {4, 3, 0}, {4, 6, 0}, {5, 1, 0}, {5, 2, 0}, 
{5, 7, 0}, {6, 0, 0}, {6, 2, 0}, {7, 1, 0}, {6, 3, 2}, {6, 6, 0}, {6, 7, 0}, {7, 3, 0}, {7, 5, 0}, {0, 1, 6}, 
{1, 1, 0}, {2, 1, 0}, {3, 1, 0}, {4, 0, 0}, {4, 1, 0}, {5, 0, 0}, {5, 1, 0}, {6, 0, 0}, {6, 1, 0}, {7, 1, 0}, 
{0, 0, 2}, {1, 0, 0}, {2, 0, 0}, {3, 0, 0}, {7, 0, 0}, {0, 2, 7}, {1, 2, 0}, {2, 2, 0}, {3, 2, 0}, {4, 1, 0},   // +99.833s 99840
{4, 2, 0}, {5, 1, 0}, {5, 2, 0}, {6, 1, 0}, {6, 2, 0}, {7, 1, 0}, {7, 2, 0}, {0, 1, 2}, {1, 1, 0}, {2, 1, 0}, 
{3, 1, 0}, {0, 2, 6}, {0, 3, 0}, {1, 3, 0}, {2, 3, 0}, {3, 3, 0}, {4, 2, 0}, {4, 3, 0}, {5, 2, 0}, {5, 3, 0}, 
{6, 2, 0}, {6, 3, 0}, {7, 2, 0}, {7, 3, 0}, {1, 2, 2}, {2, 2, 0}, {3, 2, 0}, {0, 3, 6}, {0, 4, 0}, {1, 3, 0}, 
{1, 4, 0}, {2, 4, 0}, {3, 4, 0}, {4, 3, 0}, {4, 4, 0}, {5, 3, 0}, {5, 4, 0}, {6, 3, 0}, {6, 4, 0}, {7, 3, 0}, 
{7, 4, 0}, {2, 3, 2}, {3, 3, 0}, {0, 4, 7}, {0, 5, 0}, {1, 4, 0}, {1, 5, 0}, {2, 4, 0}, {2, 5, 0}, {3, 5, 0},   // +100.233s 100240
{4, 4, 0}, {4, 5, 0}, {5, 4, 0}, {5, 5, 0}, {6, 4, 0}, {6, 5, 0}, {7, 4, 0}, {7, 5, 0}, {3, 4, 2}, {0, 5, 6}, 
{0, 6, 0}, {1, 5, 0}, {1, 6, 0}, {2, 5, 0}, {2, 6, 0}, {3, 5, 0}, {3, 6, 0}, {4, 5, 0}, {4, 6, 0}, {5, 5, 0}, 
{5, 6, 0}, {6, 5, 0}, {6, 6, 0}, {7, 5, 0}, {7, 6, 0}, {4, 7, 6}, {0, 6, 2}, {1, 6, 0}, {1, 7, 0}, {2, 6, 0}, 
{2, 7, 0}, {3, 6, 0}, {3, 7, 0}, {4, 6, 0}, {5, 6, 0}, {5, 7, 0}, {6, 6, 0}, {6, 7, 0}, {7, 6, 0}, {5, 0, 6}, 
{0, 0, 3}, {1, 0, 0}, {1, 7, 0}, {2, 0, 0}, {2, 7, 0}, {3, 0, 0}, {3, 7, 0}, {4, 0, 0}, {4, 7, 0}, {5, 7, 0},   // +100.633s 100640
{6, 0, 0}, {6, 7, 0}, {7, 0, 0}, {4, 1, 6}, {5, 1, 0}, {6, 1, 0}, {0, 0, 2}, {0, 1, 0}, {1, 0, 0}, {1, 1, 0}, 
{2, 0, 0}, {2, 1, 0}, {3, 0, 0}, {3, 1, 0}, {4, 0, 0}, {5, 0, 0}, {6, 0, 0}, {7, 0, 0}, {7, 1, 0}, {0, 2, 6}, 
{4, 2, 0}, {5, 2, 0}, {6, 2, 0}, {7, 2, 0}, {0, 1, 2}, {1, 1, 0}, {1, 2, 0}, {2, 1, 0}, {2, 2, 0}, {3, 1, 0}, 
{3, 2, 0}, {4, 1, 0}, {5, 1, 0}, {6, 1, 0}, {7, 1, 0}, {0, 3, 7}, {1, 3, 0}, {4, 3, 0}, {5, 3, 0}, {6, 3, 0}, 
{7, 3, 0}, {0, 2, 2}, {1, 2, 0}, {2, 2, 0}, {2, 3, 0}, {3, 2, 0}, {3, 3, 0}, {4, 2, 0}, {5, 2, 0}, {6, 2, 0},   // +101.033s 101040
{7, 2, 0}, {0, 4, 6}, {1, 4, 0}, {2, 4, 0}, {4, 4, 0}, {5, 4, 0}, {6, 4, 0}, {7, 4, 0}, {0, 3, 2}, {1, 3, 0}, 
{2, 3, 0}, {3, 3, 0}, {3, 4, 0}, {4, 3, 0}, {5, 3, 0}, {6, 3, 0}, {7, 3, 0}, {0, 5, 6}, {1, 5, 0}, {2, 5, 0}, 
{3, 5, 0}, {4, 5, 0}, {5, 5, 0}, {6, 5, 0}, {7, 5, 0}, {0, 4, 2}, {1, 4, 0}, {2, 4, 0}, {3, 4, 0}, {4, 4, 0}, 
{5, 4, 0}, {6, 4, 0}, {7, 4, 0}, {0, 6, 6}, {1, 6, 0}, {2, 6, 0}, {3, 6, 0}, {4, 5, 0}, {4, 6, 0}, {5, 6, 0}, 
{6, 6, 0}, {7, 6, 0}, {0, 5, 3}, {1, 5, 0}, {2, 5, 0}, {3, 5, 0}, {5, 5, 0}, {6, 5, 0}, {7, 5, 0}, {1, 7, 6},   // +101.533s 101536
{2, 7, 0}, {3, 7, 0}, {4, 6, 0}, {4, 7, 0}, {5, 6, 0}, {5, 7, 0}, {6, 7, 0}, {0, 6, 2}, {1, 6, 0}, {2, 6, 0}, 
{3, 6, 0}, {6, 6, 0}, {7, 6, 0}, {0, 0, 6}, {1, 0, 0}, {2, 0, 0}, {3, 0, 0}, {4, 0, 0}, {4, 7, 0}, {5, 0, 0}, 
{5, 7, 0}, {6, 0, 0}, {6, 7, 0}, {7, 0, 0}, {1, 7, 2}, {2, 7, 0}, {3, 7, 0}, {0, 1, 6}, {1, 1, 0}, {2, 1, 0}, 
{3, 1, 0}, {4, 1, 0}, {5, 0, 0}, {5, 1, 0}, {6, 0, 0}, {6, 1, 0}, {7, 0, 0}, {7, 1, 0}, {0, 0, 3}, {1, 0, 0}, 
{2, 0, 0}, {3, 0, 0}, {4, 0, 0}, {0, 1, 6}, {0, 2, 0}, {1, 2, 0}, {2, 2, 0}, {3, 2, 0}, {4, 1, 0}, {4, 2, 0},   // +101.933s 101936
{5, 1, 0}, {5, 2, 0}, {6, 1, 0}, {6, 2, 0}, {7, 1, 0}, {7, 2, 0}, {1, 1, 2}, {2, 1, 0}, {3, 1, 0}, {0, 2, 6}, 
{0, 3, 0}, {1, 2, 0}, {1, 3, 0}, {2, 3, 0}, {3, 3, 0}, {4, 3, 0}, {5, 2, 0}, {5, 3, 0}, {6, 2, 0}, {6, 3, 0}, 
{7, 2, 0}, {7, 3, 0}, {2, 2, 2}, {3, 2, 0}, {4, 2, 0}, {0, 3, 6}, {0, 4, 0}, {1, 3, 0}, {1, 4, 0}, {2, 3, 0}, 
{2, 4, 0}, {3, 4, 0}, {4, 3, 0}, {4, 4, 0}, {5, 3, 0}, {5, 4, 0}, {6, 3, 0}, {6, 4, 0}, {7, 3, 0}, {7, 4, 0}, 
{3, 3, 3}, {0, 4, 6}, {0, 5, 0}, {1, 4, 0}, {1, 5, 0}, {2, 4, 0}, {2, 5, 0}, {3, 4, 0}, {3, 5, 0}, {4, 4, 0},   // +102.333s 102336
{4, 5, 0}, {5, 4, 0}, {5, 5, 0}, {6, 4, 0}, {6, 5, 0}, {7, 4, 0}, {7, 5, 0}, {4, 6, 6}, {0, 5, 2}, {0, 6, 0}, 
{1, 5, 0}, {1, 6, 0}, {2, 5, 0}, {2, 6, 0}, {3, 5, 0}, {3, 6, 0}, {4, 5, 0}, {5, 5, 0}, {5, 6, 0}, {6, 5, 0}, 
{6, 6, 0}, {7, 5, 0}, {7, 6, 0}, {5, 7, 6}, {0, 6, 3}, {1, 6, 0}, {1, 7, 0}, {2, 6, 0}, {2, 7, 0}, {3, 6, 0}, 
{3, 7, 0}, {4, 6, 0}, {4, 7, 0}, {5, 6, 0}, {6, 6, 0}, {6, 7, 0}, {7, 6, 0}, {4, 0, 6}, {5, 0, 0}, {6, 0, 0}, 
{0, 0, 2}, {1, 0, 0}, {1, 7, 0}, {2, 0, 0}, {2, 7, 0}, {3, 0, 0}, {3, 7, 0}, {4, 7, 0}, {5, 7, 0}, {6, 7, 0},   // +102.733s 102736
{7, 0, 0}, {5, 1, 6}, {6, 1, 0}, {7, 1, 0}, {0, 0, 2}, {0, 1, 0}, {1, 0, 0}, {1, 1, 0}, {2, 0, 0}, {2, 1, 0}, 
{3, 0, 0}, {3, 1, 0}, {4, 0, 0}, {4, 1, 0}, {5, 0, 0}, {6, 0, 0}, {7, 0, 0}, {0, 2, 6}, {4, 2, 0}, {5, 2, 0}, 
{6, 2, 0}, {7, 2, 0}, {0, 1, 2}, {1, 1, 0}, {1, 2, 0}, {2, 1, 0}, {2, 2, 0}, {3, 1, 0}, {3, 2, 0}, {4, 1, 0}, 
{5, 1, 0}, {6, 1, 0}, {7, 1, 0}, {0, 3, 7}, {1, 3, 0}, {4, 3, 0}, {5, 3, 0}, {6, 3, 0}, {7, 3, 0}, {0, 2, 2}, 
{1, 2, 0}, {2, 2, 0}, {2, 3, 0}, {3, 2, 0}, {3, 3, 0}, {4, 2, 0}, {5, 2, 0}, {6, 2, 0}, {7, 2, 0}, {0, 4, 6},   // +103.233s 103232
{1, 4, 0}, {2, 4, 0}, {4, 4, 0}, {5, 4, 0}, {6, 4, 0}, {7, 4, 0}, {0, 3, 2}, {1, 3, 0}, {2, 3, 0}, {3, 3, 0}, 
{3, 4, 0}, {4, 3, 0}, {5, 3, 0}, {6, 3, 0}, {7, 3, 0}, {0, 5, 6}, {1, 5, 0}, {2, 5, 0}, {3, 5, 0}, {4, 5, 0}, 
{5, 5, 0}, {6, 5, 0}, {7, 5, 0}, {0, 4, 2}, {1, 4, 0}, {2, 4, 0}, {3, 4, 0}, {4, 4, 0}, {5, 4, 0}, {6, 4, 0}, 
{7, 4, 0}, {0, 6, 7}, {1, 6, 0}, {2, 6, 0}, {3, 6, 0}, {4, 5, 0}, {4, 6, 0}, {5, 6, 0}, {6, 6, 0}, {7, 6, 0}, 
{0, 5, 2}, {1, 5, 0}, {2, 5, 0}, {3, 5, 0}, {5, 5, 0}, {6, 5, 0}, {7, 5, 0}, {1, 7, 6}, {2, 7, 0}, {3, 7, 0},   // +103.633s 103632
{4, 6, 0}, {4, 7, 0}, {5, 6, 0}, {5, 7, 0}, {6, 7, 0}, {0, 6, 2}, {1, 6, 0}, {2, 6, 0}, {3, 6, 0}, {6, 6, 0}, 
{7, 6, 0}, {0, 0, 6}, {1, 0, 0}, {2, 0, 0}, {3, 0, 0}, {4, 0, 0}, {4, 7, 0}, {5, 0, 0}, {5, 7, 0}, {6, 0, 0}, 
{6, 7, 0}, {7, 0, 0}, {1, 7, 3}, {2, 7, 0}, {3, 7, 0}, {0, 0, 6}, {0, 1, 0}, {1, 1, 0}, {2, 1, 0}, {3, 1, 0}, 
{4, 0, 0}, {4, 1, 0}, {5, 0, 0}, {5, 1, 0}, {6, 0, 0}, {6, 1, 0}, {7, 0, 0}, {7, 1, 0}, {1, 0, 2}, {2, 0, 0}, 
{3, 0, 0}, {0, 2, 4}, {1, 2, 0}, {0, 1, 2}, {1, 1, 0}, {2, 1, 0}, {2, 2, 0}, {3, 1, 0}, {3, 2, 0}, {4, 1, 0},   // +104.033s 104032
{4, 2, 0}, {5, 1, 0}, {5, 2, 0}, {6, 1, 0}, {6, 2, 0}, {7, 1, 0}, {7, 2, 0}, {0, 3, 6}, {1, 3, 0}, {2, 3, 0}, 
{0, 2, 2}, {1, 2, 0}, {2, 2, 0}, {3, 2, 0}, {3, 3, 0}, {4, 2, 0}, {4, 3, 0}, {5, 2, 0}, {5, 3, 0}, {6, 2, 0}, 
{6, 3, 0}, {7, 2, 0}, {7, 3, 0}, {0, 4, 7}, {1, 4, 0}, {2, 4, 0}, {3, 4, 0}, {0, 3, 2}, {1, 3, 0}, {2, 3, 0}, 
{3, 3, 0}, {4, 3, 0}, {4, 4, 0}, {5, 3, 0}, {5, 4, 0}, {6, 3, 0}, {6, 4, 0}, {7, 3, 0}, {7, 4, 0}, {0, 5, 6}, 
{1, 5, 0}, {2, 5, 0}, {3, 5, 0}, {4, 5, 0}, {0, 4, 2}, {1, 4, 0}, {2, 4, 0}, {3, 4, 0}, {4, 4, 0}, {5, 4, 0},   // +104.433s 104432
{5, 5, 0}, {6, 4, 0}, {6, 5, 0}, {7, 4, 0}, {7, 5, 0}, {0, 6, 6}, {1, 6, 0}, {2, 6, 0}, {3, 6, 0}, {4, 6, 0}, 
{5, 6, 0}, {0, 5, 2}, {1, 5, 0}, {2, 5, 0}, {3, 5, 0}, {4, 5, 0}, {5, 5, 0}, {6, 5, 0}, {6, 6, 0}, {7, 5, 0}, 
{7, 6, 0}, {1, 7, 7}, {2, 7, 0}, {3, 7, 0}, {4, 7, 0}, {5, 7, 0}, {6, 7, 0}, {0, 6, 2}, {1, 6, 0}, {2, 6, 0}, 
{3, 6, 0}, {4, 6, 0}, {5, 6, 0}, {6, 6, 0}, {7, 6, 0}, {0, 0, 6}, {1, 0, 0}, {2, 0, 0}, {3, 0, 0}, {4, 0, 0}, 
{5, 0, 0}, {6, 0, 0}, {7, 0, 0}, {1, 7, 2}, {2, 7, 0}, {3, 7, 0}, {4, 7, 0}, {5, 7, 0}, {6, 7, 0}, {0, 0, 6},   // +104.933s 104928
{0, 1, 0}, {1, 1, 0}, {2, 1, 0}, {3, 1, 0}, {4, 1, 0}, {5, 1, 0}, {6, 1, 0}, {7, 1, 0}, {1, 0, 2}, {2, 0, 0}, 
{3, 0, 0}, {4, 0, 0}, {5, 0, 0}, {6, 0, 0}, {7, 0, 0}, {0, 1, 7}, {0, 2, 0}, {1, 1, 0}, {1, 2, 0}, {2, 2, 0}, 
{3, 2, 0}, {4, 2, 0}, {5, 2, 0}, {6, 2, 0}, {7, 2, 0}, {2, 1, 2}, {3, 1, 0}, {4, 1, 0}, {5, 1, 0}, {6, 1, 0}, 
{7, 1, 0}, {0, 2, 6}, {0, 3, 0}, {1, 2, 0}, {1, 3, 0}, {2, 2, 0}, {2, 3, 0}, {3, 3, 0}, {4, 3, 0}, {5, 3, 0}, 
{6, 3, 0}, {7, 3, 0}, {3, 2, 2}, {4, 2, 0}, {5, 2, 0}, {6, 2, 0}, {7, 2, 0}, {0, 3, 6}, {0, 4, 0}, {1, 3, 0},   // +105.333s 105328
{1, 4, 0}, {2, 3, 0}, {2, 4, 0}, {3, 3, 0}, {3, 4, 0}, {4, 4, 0}, {5, 4, 0}, {6, 4, 0}, {7, 4, 0}, {4, 3, 2}, 
{5, 3, 0}, {6, 3, 0}, {7, 3, 0}, {0, 4, 7}, {0, 5, 0}, {1, 4, 0}, {1, 5, 0}, {2, 4, 0}, {2, 5, 0}, {3, 4, 0}, 
{3, 5, 0}, {4, 4, 0}, {4, 5, 0}, {5, 5, 0}, {6, 5, 0}, {7, 5, 0}, {5, 4, 2}, {6, 4, 0}, {7, 4, 0}, {0, 5, 6}, 
{0, 6, 0}, {1, 5, 0}, {1, 6, 0}, {2, 5, 0}, {2, 6, 0}, {3, 5, 0}, {3, 6, 0}, {4, 5, 0}, {4, 6, 0}, {5, 5, 0}, 
{5, 6, 0}, {6, 6, 0}, {7, 6, 0}, {6, 5, 2}, {7, 5, 0}, {0, 6, 6}, {1, 6, 0}, {1, 7, 0}, {2, 6, 0}, {2, 7, 0},   // +105.733s 105728
{3, 6, 0}, {3, 7, 0}, {4, 6, 0}, {4, 7, 0}, {5, 6, 0}, {5, 7, 0}, {6, 6, 0}, {6, 7, 0}, {7, 6, 2}, {0, 0, 7}, 
{1, 0, 0}, {1, 7, 0}, {2, 0, 0}, {2, 7, 0}, {3, 0, 0}, {3, 7, 0}, {4, 0, 0}, {4, 7, 0}, {5, 0, 0}, {5, 7, 0}, 
{6, 0, 0}, {6, 7, 0}, {7, 0, 0}, {0, 1, 6}, {0, 0, 2}, {1, 0, 0}, {1, 1, 0}, {2, 0, 0}, {2, 1, 0}, {3, 0, 0}, 
{3, 1, 0}, {4, 0, 0}, {4, 1, 0}, {5, 0, 0}, {5, 1, 0}, {6, 0, 0}, {6, 1, 0}, {7, 0, 0}, {7, 1, 0}, {0, 2, 6}, 
{1, 2, 0}, {0, 1, 2}, {1, 1, 0}, {2, 1, 0}, {2, 2, 0}, {3, 1, 0}, {3, 2, 0}, {4, 1, 0}, {4, 2, 0}, {5, 1, 0},   // +106.133s 106128
{5, 2, 0}, {6, 1, 0}, {6, 2, 0}, {7, 1, 0}, {7, 2, 0}, {0, 3, 7}, {1, 3, 0}, {2, 3, 0}, {0, 2, 2}, {1, 2, 0}, 
{2, 2, 0}, {3, 2, 0}, {3, 3, 0}, {4, 2, 0}, {4, 3, 0}, {5, 2, 0}, {5, 3, 0}, {6, 2, 0}, {6, 3, 0}, {7, 2, 0}, 
{7, 3, 0}, {0, 4, 12}, {1, 4, 0}, {2, 4, 0}, {3, 4, 0}, {4, 4, 0}, {6, 4, 0}, {7, 4, 0}, {0, 3, 2}, {1, 3, 0}, 
{2, 3, 0}, {3, 3, 0}, {4, 3, 0}, {5, 3, 0}, {5, 4, 0}, {6, 3, 0}, {7, 3, 0}, {0, 4, 6}, {0, 5, 0}, {1, 4, 0}, 
{1, 5, 0}, {2, 4, 0}, {2, 5, 0}, {3, 4, 0}, {3, 5, 0}, {4, 4, 0}, {4, 5, 0}, {5, 4, 0}, {5, 5, 0}, {6, 5, 0},   // +106.600s 106592
{7, 5, 0}, {6, 4, 3}, {7, 4, 0}, {0, 6, 10}, {1, 6, 0}, {2, 6, 0}, {3, 6, 0}, {4, 6, 0}, {5, 6, 0}, {6, 5, 0}, 
{6, 6, 0}, {7, 5, 0}, {7, 6, 0}, {0, 5, 2}, {1, 5, 0}, {2, 5, 0}, {3, 5, 0}, {4, 5, 0}, {5, 5, 0}, {0, 6, 6}, 
{1, 6, 0}, {1, 7, 0}, {2, 6, 0}, {2, 7, 0}, {3, 6, 0}, {3, 7, 0}, {4, 6, 0}, {4, 7, 0}, {5, 6, 0}, {5, 7, 0}, 
{6, 6, 0}, {6, 7, 0}, {7, 6, 0}, {0, 0, 7}, {1, 0, 0}, {1, 7, 2}, {2, 0, 0}, {2, 7, 0}, {3, 0, 0}, {3, 7, 0}, 
{4, 0, 0}, {4, 7, 0}, {5, 0, 0}, {5, 7, 0}, {6, 0, 0}, {6, 7, 0}, {7, 0, 0}, {0, 1, 6}, {1, 1, 0}, {2, 1, 0},   // +107.167s 107168
{0, 0, 2}, {1, 0, 0}, {2, 0, 0}, {3, 0, 0}, {3, 1, 0}, {4, 0, 0}, {4, 1, 0}, {5, 0, 0}, {5, 1, 0}, {6, 0, 0}, 
{6, 1, 0}, {7, 0, 0}, {7, 1, 0}, {0, 2, 6}, {1, 2, 0}, {2, 2, 0}, {3, 2, 0}, {0, 1, 2}, {1, 1, 0}, {2, 1, 0}, 
{3, 1, 0}, {4, 1, 0}, {4, 2, 0}, {5, 1, 0}, {5, 2, 0}, {6, 1, 0}, {6, 2, 0}, {7, 1, 0}, {7, 2, 0}, {0, 3, 7}, 
{1, 3, 0}, {2, 3, 0}, {3, 3, 0}, {4, 3, 0}, {0, 2, 2}, {1, 2, 0}, {2, 2, 0}, {3, 2, 0}, {4, 2, 0}, {5, 2, 0}, 
{5, 3, 0}, {6, 2, 0}, {6, 3, 0}, {7, 2, 0}, {7, 3, 0}, {0, 4, 6}, {1, 4, 0}, {2, 4, 0}, {3, 4, 0}, {4, 4, 0},   // +107.567s 107568
{5, 4, 0}, {0, 3, 2}, {1, 3, 0}, {2, 3, 0}, {3, 3, 0}, {4, 3, 0}, {5, 3, 0}, {6, 3, 0}, {6, 4, 0}, {7, 3, 0}, 
{7, 4, 0}, {0, 5, 6}, {1, 5, 0}, {2, 5, 0}, {3, 5, 0}, {4, 5, 0}, {5, 5, 0}, {6, 5, 0}, {0, 4, 2}, {1, 4, 0}, 
{2, 4, 0}, {3, 4, 0}, {4, 4, 0}, {5, 4, 0}, {6, 4, 0}, {7, 4, 0}, {7, 5, 0}, {0, 6, 7}, {1, 6, 0}, {2, 6, 0}, 
{3, 6, 0}, {4, 6, 0}, {5, 6, 0}, {6, 6, 0}, {7, 6, 0}, {0, 5, 2}, {1, 5, 0}, {2, 5, 0}, {3, 5, 0}, {4, 5, 0}, 
{5, 5, 0}, {6, 5, 0}, {7, 5, 0}, {0, 6, 6}, {1, 7, 0}, {2, 7, 0}, {3, 7, 0}, {4, 7, 0}, {5, 7, 0}, {6, 7, 0},   // +107.967s 107968
{1, 6, 2}, {2, 6, 0}, {3, 6, 0}, {4, 6, 0}, {5, 6, 0}, {6, 6, 0}, {7, 6, 0}, {0, 0, 6}, {1, 0, 0}, {1, 7, 0}, 
{2, 0, 0}, {3, 0, 0}, {4, 0, 0}, {5, 0, 0}, {6, 0, 0}, {7, 0, 0}, {2, 7, 2}, {3, 7, 0}, {4, 7, 0}, {5, 7, 0}, 
{6, 7, 0}, {0, 0, 7}, {0, 1, 0}, {1, 0, 0}, {1, 1, 0}, {2, 0, 0}, {2, 1, 0}, {3, 1, 0}, {4, 1, 0}, {5, 1, 0}, 
{6, 1, 0}, {7, 1, 0}, {3, 0, 2}, {4, 0, 0}, {5, 0, 0}, {6, 0, 0}, {7, 0, 0}, {0, 1, 6}, {0, 2, 0}, {1, 1, 0}, 
{1, 2, 0}, {2, 1, 0}, {2, 2, 0}, {3, 1, 0}, {3, 2, 0}, {4, 2, 0}, {5, 2, 0}, {6, 2, 0}, {7, 2, 0}, {4, 1, 2},   // +108.400s 108400
{5, 1, 0}, {6, 1, 0}, {7, 1, 0}, {0, 2, 6}, {0, 3, 0}, {1, 2, 0}, {1, 3, 0}, {2, 2, 0}, {2, 3, 0}, {3, 2, 0}, 
{3, 3, 0}, {4, 2, 0}, {4, 3, 0}, {5, 3, 0}, {6, 3, 0}, {7, 3, 0}, {5, 2, 2}, {6, 2, 0}, {7, 2, 0}, {0, 3, 7}, 
{0, 4, 0}, {1, 3, 0}, {1, 4, 0}, {2, 3, 0}, {2, 4, 0}, {3, 3, 0}, {3, 4, 0}, {4, 3, 0}, {4, 4, 0}, {5, 3, 0}, 
{5, 4, 0}, {6, 4, 0}, {7, 4, 0}, {6, 3, 2}, {7, 3, 0}, {0, 4, 6}, {0, 5, 0}, {1, 4, 0}, {1, 5, 0}, {2, 4, 0}, 
{2, 5, 0}, {3, 4, 0}, {3, 5, 0}, {4, 4, 0}, {4, 5, 0}, {5, 4, 0}, {5, 5, 0}, {6, 4, 0}, {6, 5, 0}, {7, 5, 0},   // +108.767s 108768
{7, 4, 2}, {0, 6, 4}, {0, 5, 2}, {1, 5, 0}, {1, 6, 0}, {2, 5, 0}, {2, 6, 0}, {3, 5, 0}, {3, 6, 0}, {4, 5, 0}, 
{4, 6, 0}, {5, 5, 0}, {5, 6, 0}, {6, 5, 0}, {6, 6, 0}, {7, 5, 0}, {7, 6, 0}, {1, 7, 6}, {0, 6, 3}, {1, 6, 0}, 
{2, 6, 0}, {2, 7, 0}, {3, 6, 0}, {3, 7, 0}, {4, 6, 0}, {4, 7, 0}, {5, 6, 0}, {5, 7, 0}, {6, 6, 0}, {6, 7, 0}, 
{7, 6, 0}, {0, 0, 6}, {1, 0, 0}, {2, 0, 0}, {1, 7, 2}, {2, 7, 0}, {3, 0, 0}, {3, 7, 0}, {4, 0, 0}, {4, 7, 0}, 
{5, 0, 0}, {5, 7, 0}, {6, 0, 0}, {6, 7, 0}, {7, 0, 0}, {0, 1, 12}, {1, 1, 0}, {2, 1, 0}, {3, 0, 0}, {3, 1, 0},   // +109.367s 109360
{4, 1, 0}, {5, 1, 0}, {6, 1, 0}, {7, 1, 0}, {0, 0, 2}, {1, 0, 0}, {2, 0, 0}, {4, 0, 0}, {5, 0, 0}, {6, 0, 0}, 
{7, 0, 0}, {0, 2, 9}, {1, 2, 0}, {2, 2, 0}, {3, 2, 0}, {4, 2, 0}, {5, 2, 0}, {7, 2, 0}, {0, 1, 2}, {1, 1, 0}, 
{2, 1, 0}, {3, 1, 0}, {4, 1, 0}, {5, 1, 0}, {6, 1, 0}, {6, 2, 0}, {7, 1, 0}, {0, 2, 6}, {0, 3, 0}, {1, 2, 0}, 
{1, 3, 0}, {2, 2, 0}, {2, 3, 0}, {3, 2, 0}, {3, 3, 0}, {4, 2, 0}, {4, 3, 0}, {5, 2, 0}, {5, 3, 0}, {6, 2, 0}, 
{6, 3, 0}, {7, 3, 0}, {7, 2, 2}, {0, 3, 7}, {0, 4, 0}, {1, 3, 0}, {1, 4, 0}, {2, 3, 0}, {2, 4, 0}, {3, 3, 0},   // +109.800s 109808
{3, 4, 0}, {4, 3, 0}, {4, 4, 0}, {5, 3, 0}, {5, 4, 0}, {6, 3, 0}, {6, 4, 0}, {7, 3, 0}, {7, 4, 0}, {0, 5, 6}, 
{0, 4, 2}, {1, 4, 0}, {1, 5, 0}, {2, 4, 0}, {2, 5, 0}, {3, 4, 0}, {3, 5, 0}, {4, 4, 0}, {4, 5, 0}, {5, 4, 0}, 
{5, 5, 0}, {6, 4, 0}, {6, 5, 0}, {7, 4, 0}, {7, 5, 0}, {0, 6, 6}, {1, 6, 0}, {0, 5, 2}, {1, 5, 0}, {2, 5, 0}, 
{2, 6, 0}, {3, 5, 0}, {3, 6, 0}, {4, 5, 0}, {4, 6, 0}, {5, 5, 0}, {5, 6, 0}, {6, 5, 0}, {6, 6, 0}, {7, 5, 0}, 
{7, 6, 0}, {1, 7, 6}, {2, 7, 0}, {0, 6, 2}, {1, 6, 0}, {2, 6, 0}, {3, 6, 0}, {3, 7, 0}, {4, 6, 0}, {4, 7, 0},   // +110.200s 110192
{5, 6, 0}, {5, 7, 0}, {6, 6, 0}, {6, 7, 0}, {7, 6, 0}, {5, 0, 9}, {7, 1, 0}, {7, 2, 0}, {5, 1, 2}, {7, 0, 0}, 
{7, 3, 0}, {7, 4, 0}, {4, 6, 2}, {5, 2, 0}, {5, 3, 0}, {5, 4, 0}, {4, 4, 2}, {5, 5, 0}, {5, 6, 0}, {0, 5, 2}, 
{0, 6, 0}, {3, 4, 0}, {4, 5, 0}, {0, 4, 2}, {2, 5, 0}, {2, 6, 0}, {3, 3, 0}, {0, 2, 2}, {0, 3, 0}, {2, 4, 0}, 
{0, 1, 2}, {3, 0, 0}, {3, 2, 0}, {3, 5, 0}, {0, 0, 2}, {6, 5, 3}, {6, 6, 0}, {3, 1, 4}, {3, 6, 0}, {6, 4, 0}, 
{4, 0, 2}, {6, 3, 0}, {2, 0, 2}, {4, 1, 0}, {6, 1, 0}, {6, 2, 0}, {7, 6, 0}, {4, 2, 2}, {4, 3, 0}, {6, 0, 0},   // +110.800s 110800
{1, 1, 2}, {1, 2, 0}, {2, 1, 0}, {2, 2, 0}, {2, 3, 0}, {1, 0, 2}, {1, 3, 0}, {1, 4, 0}, {1, 6, 0}, {1, 5, 2}, 
{7, 5, 0}, {5, 0, 2}, {5, 7, 0}, {7, 1, 0}, {7, 2, 0}, {5, 1, 2}, {7, 0, 0}, {7, 3, 0}, {7, 4, 0}, {1, 7, 2}, 
{5, 2, 0}, {5, 3, 0}, {5, 4, 0}, {4, 6, 3}, {5, 5, 0}, {5, 6, 0}, {0, 6, 2}, {4, 4, 0}, {4, 5, 0}, {0, 4, 2}, 
{0, 5, 0}, {2, 6, 0}, {3, 4, 0}, {2, 5, 2}, {3, 3, 0}, {0, 2, 2}, {0, 3, 0}, {2, 4, 0}, {2, 7, 0}, {0, 1, 2}, 
{3, 0, 0}, {3, 5, 0}, {0, 0, 2}, {3, 2, 0}, {6, 7, 0}, {3, 1, 2}, {6, 5, 0}, {6, 6, 0}, {3, 6, 2}, {3, 7, 0},   // +111.300s 111296
{4, 7, 0}, {6, 4, 0}, {6, 3, 2}, {4, 0, 2}, {7, 6, 0}, {2, 0, 2}, {6, 2, 0}, {4, 1, 3}, {6, 1, 0}, {4, 2, 4}, 
{2, 1, 2}, {4, 3, 0}, {6, 0, 0}, {2, 2, 6}, {2, 3, 0}, {1, 1, 2}, {1, 2, 0}, {1, 3, 4}, {1, 6, 0}, {1, 0, 2}, 
{1, 4, 0}, {1, 5, 5}, {7, 5, 0}, {0, 2, 2}, {0, 3, 0}, {0, 1, 2}, {0, 4, 0}, {0, 5, 0}, {1, 7, 0}, {0, 0, 2}, 
{0, 6, 0}, {2, 4, 2}, {2, 5, 0}, {2, 6, 0}, {2, 7, 0}, {4, 4, 2}, {6, 5, 0}, {6, 6, 0}, {6, 7, 0}, {4, 5, 2}, 
{4, 7, 0}, {3, 0, 2}, {3, 1, 0}, {4, 6, 0}, {5, 5, 0}, {5, 6, 0}, {6, 4, 0}, {3, 2, 2}, {3, 3, 0}, {5, 4, 0},   // +112.100s 112096
{7, 4, 0}, {3, 4, 2}, {5, 3, 0}, {6, 3, 0}, {7, 3, 0}, {3, 5, 2}, {3, 6, 0}, {5, 2, 0}, {6, 2, 0}, {3, 7, 2}, 
{5, 1, 0}, {7, 0, 0}, {7, 6, 0}, {2, 0, 3}, {5, 0, 0}, {6, 1, 0}, {7, 1, 0}, {2, 1, 2}, {4, 0, 0}, {6, 0, 0}, 
{7, 2, 0}, {2, 2, 2}, {2, 3, 0}, {4, 1, 0}, {4, 2, 0}, {4, 3, 0}, {5, 7, 0}, {1, 1, 2}, {1, 2, 0}, {1, 6, 0}, 
{7, 5, 0}, {1, 3, 2}, {1, 4, 0}, {1, 0, 2}, {1, 5, 0}, {0, 1, 2}, {0, 2, 0}, {0, 3, 0}, {0, 4, 0}, {1, 7, 0}, 
{0, 0, 2}, {0, 5, 0}, {0, 6, 0}, {2, 4, 0}, {2, 5, 0}, {2, 6, 2}, {2, 7, 0}, {4, 4, 2}, {4, 5, 0}, {4, 7, 0},   // +112.533s 112528
{6, 5, 0}, {6, 6, 0}, {6, 7, 0}, {3, 0, 2}, {4, 6, 0}, {5, 5, 0}, {5, 6, 0}, {3, 1, 2}, {3, 2, 0}, {3, 3, 0}, 
{6, 4, 0}, {3, 4, 3}, {5, 4, 0}, {7, 4, 0}, {3, 5, 2}, {5, 3, 0}, {6, 2, 0}, {6, 3, 0}, {7, 3, 0}, {3, 6, 2}, 
{5, 2, 0}, {3, 7, 2}, {5, 1, 0}, {7, 0, 0}, {7, 6, 0}, {2, 0, 2}, {4, 0, 0}, {5, 0, 0}, {6, 0, 0}, {6, 1, 0}, 
{7, 1, 0}, {2, 1, 2}, {4, 1, 0}, {4, 2, 0}, {7, 2, 0}, {2, 2, 2}, {2, 3, 0}, {4, 3, 0}, {5, 7, 0}, {1, 6, 2}, 
{7, 5, 0}, {1, 1, 2}, {1, 2, 0}, {1, 3, 0}, {1, 4, 0}, {1, 0, 2}, {1, 5, 0}, {5, 0, 0}, {7, 0, 0}, {7, 1, 0},   // +112.933s 112928
{7, 2, 0}, {5, 1, 2}, {5, 7, 0}, {1, 6, 3}, {4, 0, 0}, {4, 1, 0}, {4, 2, 0}, {5, 2, 0}, {2, 1, 2}, {4, 3, 0}, 
{7, 3, 0}, {3, 7, 2}, {5, 3, 0}, {5, 4, 0}, {7, 5, 0}, {3, 4, 2}, {3, 5, 0}, {2, 0, 2}, {2, 2, 0}, {2, 3, 0}, 
{3, 6, 0}, {6, 0, 0}, {1, 1, 2}, {1, 2, 0}, {5, 5, 0}, {7, 4, 0}, {7, 6, 0}, {1, 3, 2}, {1, 4, 0}, {1, 5, 2}, 
{5, 6, 0}, {3, 0, 2}, {3, 1, 0}, {3, 3, 0}, {4, 5, 0}, {4, 6, 0}, {1, 0, 2}, {3, 2, 0}, {6, 1, 2}, {6, 2, 0}, 
{6, 3, 0}, {2, 6, 2}, {2, 7, 0}, {4, 4, 0}, {6, 4, 0}, {4, 7, 2}, {6, 7, 0}, {0, 6, 3}, {2, 4, 0}, {2, 5, 0},   // +113.433s 113440
{6, 5, 0}, {6, 6, 0}, {0, 4, 2}, {0, 5, 0}, {0, 0, 2}, {0, 1, 0}, {0, 2, 0}, {0, 3, 0}, {1, 7, 2}, {5, 0, 0}, 
{5, 1, 2}, {5, 7, 0}, {7, 0, 0}, {7, 1, 0}, {7, 2, 0}, {4, 0, 2}, {4, 1, 0}, {4, 2, 0}, {5, 2, 0}, {1, 6, 2}, 
{4, 3, 0}, {2, 1, 2}, {3, 7, 0}, {5, 3, 0}, {7, 3, 0}, {3, 4, 2}, {3, 5, 0}, {5, 4, 0}, {7, 5, 0}, {2, 0, 2}, 
{3, 6, 0}, {2, 2, 2}, {2, 3, 0}, {6, 0, 0}, {1, 1, 2}, {1, 2, 0}, {5, 5, 0}, {7, 4, 0}, {7, 6, 0}, {1, 3, 3}, 
{1, 4, 0}, {1, 5, 0}, {5, 6, 0}, {3, 3, 2}, {4, 5, 0}, {4, 6, 0}, {1, 0, 2}, {3, 0, 0}, {3, 1, 0}, {3, 2, 0},   // +113.900s 113904
{6, 1, 2}, {6, 2, 0}, {6, 3, 0}, {4, 4, 2}, {6, 4, 0}, {2, 6, 2}, {2, 7, 0}, {4, 7, 0}, {6, 5, 0}, {6, 6, 0}, 
{6, 7, 0}, {0, 5, 2}, {0, 6, 0}, {2, 4, 0}, {2, 5, 0}, {0, 2, 2}, {0, 3, 0}, {0, 4, 0}, {1, 5, 0}, {0, 0, 2}, 
{0, 1, 0}, {1, 0, 0}, {1, 3, 0}, {1, 4, 0}, {1, 6, 0}, {1, 7, 0}, {7, 5, 0}, {1, 1, 2}, {1, 2, 0}, {2, 2, 0}, 
{2, 3, 0}, {2, 1, 2}, {4, 3, 0}, {2, 0, 3}, {4, 1, 0}, {4, 2, 0}, {6, 0, 0}, {6, 1, 0}, {4, 0, 2}, {6, 2, 0}, 
{7, 6, 0}, {3, 1, 2}, {3, 6, 0}, {3, 7, 0}, {4, 7, 0}, {6, 3, 0}, {6, 4, 0}, {0, 0, 2}, {6, 5, 0}, {6, 6, 0},   // +114.300s 114304
{0, 1, 2}, {3, 0, 0}, {3, 2, 0}, {3, 5, 0}, {6, 7, 0}, {0, 2, 2}, {2, 4, 0}, {2, 7, 0}, {0, 3, 2}, {2, 5, 0}, 
{3, 3, 0}, {0, 4, 2}, {0, 5, 0}, {0, 6, 0}, {2, 6, 0}, {3, 4, 0}, {4, 5, 0}, {1, 7, 2}, {4, 4, 0}, {4, 6, 2}, 
{5, 3, 0}, {5, 4, 0}, {5, 5, 0}, {5, 6, 0}, {5, 2, 2}, {5, 0, 2}, {5, 1, 0}, {7, 0, 0}, {7, 3, 0}, {7, 4, 0}, 
{5, 7, 2}, {7, 1, 0}, {7, 2, 0}, {1, 0, 3}, {1, 4, 0}, {1, 5, 0}, {7, 5, 0}, {1, 1, 2}, {1, 2, 0}, {1, 3, 0}, 
{1, 6, 0}, {2, 1, 2}, {2, 2, 0}, {2, 3, 0}, {4, 3, 0}, {4, 1, 2}, {4, 2, 0}, {6, 0, 0}, {6, 1, 0}, {2, 0, 2},   // +114.767s 114768
{6, 2, 0}, {3, 7, 2}, {4, 0, 0}, {4, 7, 0}, {6, 3, 0}, {6, 4, 0}, {7, 6, 0}, {3, 1, 2}, {3, 6, 0}, {6, 5, 0}, 
{6, 6, 0}, {0, 0, 2}, {0, 1, 0}, {3, 2, 0}, {6, 7, 0}, {0, 2, 2}, {0, 3, 0}, {2, 4, 0}, {2, 7, 0}, {3, 0, 0}, 
{3, 5, 0}, {0, 4, 2}, {2, 5, 0}, {3, 3, 0}, {0, 5, 2}, {0, 6, 0}, {2, 6, 0}, {3, 4, 0}, {4, 5, 2}, {1, 7, 3}, 
{4, 4, 0}, {4, 6, 0}, {5, 5, 0}, {5, 6, 0}, {5, 2, 2}, {5, 3, 0}, {5, 4, 0}, {5, 1, 2}, {7, 0, 0}, {7, 3, 0}, 
{7, 4, 0}, {1, 0, 2}, {1, 3, 0}, {1, 4, 0}, {1, 5, 0}, {5, 0, 0}, {5, 7, 0}, {7, 1, 0}, {7, 2, 0}, {1, 1, 2},   // +115.167s 115168
{1, 2, 0}, {1, 6, 0}, {2, 2, 2}, {2, 3, 0}, {7, 5, 0}, {2, 1, 2}, {4, 2, 0}, {4, 3, 0}, {5, 7, 0}, {2, 0, 2}, 
{4, 0, 0}, {4, 1, 0}, {6, 0, 0}, {7, 2, 0}, {3, 7, 2}, {5, 0, 0}, {6, 1, 0}, {7, 1, 0}, {3, 6, 2}, {5, 1, 0}, 
{7, 0, 0}, {7, 6, 0}, {3, 5, 2}, {5, 2, 0}, {6, 2, 0}, {3, 4, 2}, {5, 3, 0}, {6, 3, 0}, {7, 3, 0}, {3, 2, 3}, 
{3, 3, 0}, {5, 4, 0}, {7, 4, 0}, {3, 0, 2}, {3, 1, 0}, {4, 6, 0}, {5, 5, 0}, {6, 4, 0}, {4, 4, 2}, {4, 5, 0}, 
{4, 7, 0}, {5, 6, 0}, {2, 6, 2}, {2, 7, 0}, {6, 5, 0}, {6, 6, 0}, {6, 7, 0}, {0, 0, 2}, {0, 6, 0}, {2, 4, 0},   // +115.567s 115568
{2, 5, 0}, {0, 1, 2}, {0, 5, 0}, {1, 7, 0}, {0, 2, 2}, {0, 3, 0}, {0, 4, 0}, {1, 0, 2}, {1, 3, 0}, {1, 4, 0}, 
{1, 5, 0}, {1, 1, 2}, {1, 2, 0}, {1, 6, 0}, {2, 2, 2}, {2, 3, 0}, {4, 3, 0}, {5, 7, 0}, {7, 5, 0}, {2, 1, 2}, 
{4, 1, 0}, {4, 2, 0}, {4, 0, 3}, {6, 0, 0}, {7, 1, 0}, {7, 2, 0}, {2, 0, 2}, {3, 7, 0}, {5, 0, 0}, {5, 1, 0}, 
{6, 1, 0}, {3, 6, 2}, {5, 2, 0}, {7, 0, 0}, {7, 6, 0}, {3, 5, 2}, {5, 3, 0}, {6, 2, 0}, {7, 3, 0}, {3, 4, 2}, 
{5, 4, 0}, {6, 3, 0}, {7, 4, 0}, {3, 1, 2}, {3, 2, 0}, {3, 3, 0}, {6, 4, 0}, {3, 0, 2}, {4, 6, 0}, {5, 5, 0},   // +116.000s 116000
{5, 6, 0}, {4, 5, 2}, {4, 7, 0}, {6, 7, 0}, {2, 6, 2}, {2, 7, 0}, {4, 4, 0}, {6, 5, 0}, {6, 6, 0}, {0, 0, 2}, 
{0, 5, 0}, {0, 6, 0}, {2, 4, 0}, {2, 5, 0}, {0, 1, 2}, {0, 4, 0}, {1, 7, 0}, {0, 2, 15}, {0, 3, 0}, {0, 0, 6}, 
{0, 1, 0}, {0, 2, 0}, {1, 7, 0}, {0, 3, 6}, {0, 4, 0}, {0, 5, 0}, {0, 6, 9}, {2, 4, 0}, {2, 5, 0}, {6, 5, 0}, 
{2, 7, 4}, {4, 7, 0}, {6, 6, 0}, {6, 7, 0}, {2, 6, 2}, {4, 4, 0}, {6, 0, 0}, {6, 4, 0}, {1, 0, 2}, {6, 1, 0}, 
{6, 2, 0}, {6, 3, 0}, {3, 0, 2}, {3, 1, 0}, {3, 2, 0}, {4, 5, 0}, {3, 3, 2}, {3, 4, 0}, {4, 6, 0}, {5, 6, 0},   // +116.900s 116896
{1, 2, 2}, {1, 3, 0}, {1, 4, 0}, {1, 5, 0}, {1, 1, 2}, {5, 5, 0}, {7, 4, 0}, {7, 6, 0}, {2, 2, 2}, {2, 3, 0}, 
{2, 0, 3}, {3, 5, 0}, {3, 6, 0}, {5, 4, 0}, {1, 6, 2}, {3, 7, 0}, {5, 3, 0}, {7, 3, 0}, {7, 5, 0}, {2, 1, 2}, 
{4, 2, 0}, {4, 3, 0}, {4, 0, 2}, {4, 1, 0}, {5, 2, 0}, {5, 7, 0}, {5, 1, 2}, {7, 0, 0}, {7, 1, 0}, {7, 2, 0}, 
{0, 0, 2}, {0, 1, 0}, {0, 2, 0}, {1, 7, 0}, {5, 0, 0}, {0, 3, 2}, {0, 4, 0}, {0, 5, 0}, {0, 6, 2}, {2, 4, 0}, 
{2, 5, 0}, {2, 7, 2}, {4, 7, 0}, {6, 5, 0}, {6, 6, 0}, {2, 6, 2}, {6, 4, 0}, {6, 7, 0}, {1, 0, 2}, {4, 4, 0},   // +117.367s 117360
{6, 0, 0}, {6, 1, 0}, {6, 2, 0}, {3, 0, 2}, {3, 1, 0}, {3, 2, 0}, {4, 5, 0}, {6, 3, 0}, {3, 3, 3}, {3, 4, 0}, 
{4, 6, 0}, {1, 2, 2}, {1, 3, 0}, {1, 4, 0}, {1, 5, 0}, {5, 6, 0}, {1, 1, 2}, {7, 6, 0}, {2, 0, 2}, {2, 2, 0}, 
{2, 3, 0}, {5, 5, 0}, {7, 4, 0}, {3, 5, 2}, {3, 6, 0}, {5, 4, 0}, {3, 7, 2}, {5, 3, 0}, {7, 5, 0}, {1, 6, 2}, 
{2, 1, 0}, {4, 2, 0}, {4, 3, 0}, {7, 3, 0}, {4, 0, 2}, {4, 1, 0}, {5, 2, 0}, {5, 7, 0}, {5, 1, 2}, {7, 0, 0}, 
{1, 1, 2}, {1, 6, 0}, {2, 3, 0}, {4, 7, 0}, {5, 0, 0}, {5, 6, 0}, {6, 3, 0}, {6, 4, 0}, {6, 5, 0}, {7, 1, 0},   // +117.733s 117728
{7, 2, 0}, {0, 6, 2}, {2, 4, 0}, {7, 3, 0}, {7, 4, 0}, {7, 6, 0}, {6, 6, 2}, {0, 5, 3}, {1, 2, 0}, {2, 0, 0}, 
{2, 5, 0}, {5, 5, 0}, {6, 2, 0}, {7, 0, 0}, {0, 4, 4}, {1, 3, 0}, {2, 1, 0}, {5, 4, 0}, {6, 1, 0}, {6, 7, 0}, 
{7, 1, 0}, {2, 6, 2}, {7, 2, 2}, {7, 5, 0}, {0, 3, 2}, {1, 4, 0}, {4, 4, 0}, {5, 3, 0}, {6, 0, 0}, {6, 5, 0}, 
{7, 3, 0}, {0, 2, 4}, {1, 5, 0}, {5, 2, 0}, {6, 4, 0}, {4, 5, 2}, {6, 3, 0}, {6, 6, 0}, {1, 0, 2}, {2, 2, 0}, 
{7, 0, 0}, {0, 1, 2}, {2, 0, 0}, {2, 3, 0}, {3, 0, 0}, {4, 6, 0}, {5, 1, 0}, {7, 4, 0}, {7, 6, 0}, {6, 2, 3},   // +118.200s 118208
{0, 0, 2}, {1, 1, 0}, {5, 0, 0}, {6, 7, 0}, {7, 1, 0}, {3, 1, 2}, {2, 1, 2}, {0, 6, 2}, {1, 2, 0}, {3, 2, 0}, 
{6, 1, 0}, {6, 7, 0}, {7, 3, 0}, {5, 6, 2}, {6, 6, 2}, {3, 3, 2}, {6, 0, 0}, {7, 0, 0}, {7, 5, 0}, {0, 5, 2}, 
{1, 3, 0}, {1, 7, 0}, {5, 5, 0}, {3, 4, 2}, {6, 5, 0}, {7, 1, 0}, {2, 3, 2}, {6, 3, 0}, {6, 4, 0}, {0, 4, 2}, 
{2, 2, 0}, {4, 7, 0}, {5, 4, 0}, {7, 4, 0}, {7, 6, 0}, {1, 4, 2}, {2, 0, 0}, {3, 5, 0}, {4, 6, 0}, {6, 7, 0}, 
{7, 3, 0}, {6, 2, 3}, {3, 6, 2}, {5, 3, 0}, {0, 3, 2}, {1, 5, 0}, {2, 1, 0}, {2, 7, 0}, {6, 1, 0}, {6, 6, 0},   // +118.700s 118704
{7, 0, 2}, {3, 7, 2}, {6, 0, 0}, {7, 5, 0}, {0, 2, 2}, {5, 2, 0}, {6, 5, 0}, {1, 0, 2}, {4, 0, 0}, {7, 1, 0}, 
{6, 3, 2}, {6, 4, 0}, {0, 1, 2}, {5, 1, 0}, {6, 5, 0}, {1, 1, 2}, {4, 1, 0}, {7, 3, 0}, {2, 0, 2}, {2, 2, 0}, 
{2, 3, 0}, {4, 6, 0}, {5, 7, 0}, {1, 2, 2}, {6, 2, 0}, {6, 6, 0}, {7, 0, 0}, {7, 4, 0}, {7, 6, 0}, {0, 0, 3}, 
{4, 2, 0}, {5, 0, 0}, {1, 3, 2}, {6, 7, 0}, {2, 1, 2}, {4, 3, 0}, {6, 1, 0}, {7, 1, 0}, {0, 0, 2}, {5, 6, 0}, 
{1, 4, 2}, {6, 5, 0}, {0, 1, 2}, {1, 6, 0}, {2, 4, 0}, {5, 5, 0}, {7, 3, 0}, {7, 5, 0}, {1, 5, 2}, {6, 0, 0},   // +119.233s 119232
{0, 2, 4}, {5, 4, 0}, {6, 4, 0}, {6, 6, 0}, {7, 0, 0}, {1, 0, 2}, {2, 5, 0}, {6, 3, 0}, {0, 3, 2}, {2, 0, 0}, 
{2, 2, 0}, {2, 3, 0}, {4, 7, 0}, {5, 3, 0}, {6, 2, 2}, {6, 7, 0}, {7, 4, 0}, {7, 6, 0}, {1, 1, 3}, {2, 6, 0}, 
{7, 1, 0}, {0, 4, 2}, {2, 1, 0}, {5, 2, 0}, {7, 2, 0}, {6, 1, 2}, {6, 7, 0}, {0, 5, 2}, {1, 2, 0}, {5, 1, 0}, 
{7, 3, 0}, {4, 4, 2}, {6, 0, 0}, {7, 5, 0}, {5, 0, 2}, {6, 6, 0}, {7, 0, 0}, {0, 6, 2}, {1, 3, 0}, {4, 5, 2}, 
{6, 3, 0}, {6, 4, 0}, {6, 5, 0}, {7, 1, 2}, {0, 0, 2}, {5, 6, 0}, {1, 4, 2}, {2, 0, 0}, {2, 2, 0}, {6, 2, 0},   // +119.767s 119760
{6, 7, 0}, {2, 3, 2}, {3, 0, 0}, {4, 6, 0}, {4, 7, 0}, {7, 3, 0}, {7, 4, 0}, {7, 6, 0}, {0, 1, 3}, {5, 5, 0}, 
{6, 6, 2}, {1, 5, 2}, {2, 1, 0}, {3, 1, 0}, {6, 1, 0}, {7, 0, 0}, {0, 2, 2}, {5, 4, 0}, {1, 7, 2}, {1, 0, 2}, 
{6, 0, 0}, {6, 5, 0}, {7, 1, 0}, {7, 5, 0}, {3, 2, 2}, {0, 3, 2}, {5, 3, 0}, {1, 1, 2}, {6, 3, 0}, {6, 4, 0}, 
{6, 5, 0}, {7, 3, 0}, {3, 3, 2}, {0, 4, 2}, {1, 2, 0}, {2, 0, 0}, {2, 3, 0}, {5, 2, 0}, {6, 6, 0}, {2, 2, 2}, 
{2, 7, 0}, {4, 7, 0}, {6, 2, 0}, {7, 0, 0}, {7, 4, 0}, {7, 6, 0}, {3, 4, 3}, {1, 3, 2}, {2, 1, 0}, {6, 1, 0},   // +120.267s 120272
{6, 7, 0}, {7, 1, 0}, {0, 5, 2}, {5, 1, 0}, {1, 4, 2}, {7, 5, 0}, {3, 5, 2}, {6, 0, 0}, {6, 5, 0}, {7, 3, 0}, 
{0, 6, 2}, {5, 0, 0}, {1, 5, 2}, {6, 4, 0}, {5, 7, 2}, {6, 3, 0}, {6, 6, 0}, {0, 6, 2}, {1, 0, 0}, {3, 6, 0}, 
{5, 6, 0}, {7, 0, 0}, {2, 0, 2}, {6, 2, 2}, {0, 5, 2}, {1, 1, 0}, {2, 2, 0}, {2, 3, 0}, {3, 7, 0}, {4, 6, 0}, 
{5, 5, 0}, {6, 7, 0}, {7, 1, 0}, {7, 4, 3}, {7, 6, 0}, {0, 4, 2}, {2, 1, 0}, {5, 4, 0}, {6, 1, 0}, {6, 7, 0}, 
{1, 2, 2}, {1, 6, 0}, {7, 3, 0}, {4, 0, 2}, {5, 3, 0}, {0, 3, 2}, {6, 6, 0}, {7, 0, 0}, {6, 0, 2}, {7, 5, 0},   // +120.800s 120800
{0, 2, 2}, {1, 3, 0}, {5, 2, 0}, {6, 5, 0}, {4, 1, 2}, {6, 4, 0}, {7, 1, 0}, {0, 1, 2}, {5, 1, 0}, {6, 3, 0}, 
{6, 7, 2}, {7, 2, 0}, {1, 4, 2}, {2, 0, 0}, {4, 2, 0}, {6, 2, 0}, {7, 3, 0}, {0, 0, 2}, {2, 3, 0}, {5, 0, 0}, 
{2, 2, 3}, {4, 6, 0}, {4, 7, 0}, {7, 4, 0}, {7, 6, 0}, {1, 5, 2}, {2, 1, 0}, {6, 1, 0}, {6, 6, 0}, {7, 0, 0}, 
{0, 6, 2}, {4, 3, 0}, {5, 6, 0}, {6, 0, 2}, {7, 5, 0}, {1, 0, 2}, {2, 4, 0}, {6, 5, 0}, {0, 5, 2}, {5, 5, 0}, 
{7, 1, 0}, {2, 5, 2}, {6, 3, 0}, {6, 4, 0}, {1, 1, 2}, {6, 5, 0}, {7, 3, 2}, {0, 4, 2}, {2, 0, 0}, {2, 6, 0},   // +121.333s 121328
{5, 4, 0}, {1, 2, 2}, {6, 2, 0}, {6, 6, 0}, {7, 0, 0}, {1, 7, 3}, {2, 2, 0}, {0, 3, 2}, {1, 3, 0}, {2, 3, 0}, 
{4, 4, 0}, {4, 6, 0}, {5, 3, 0}, {6, 7, 0}, {2, 1, 2}, {6, 1, 0}, {7, 1, 0}, {7, 4, 0}, {7, 6, 0}, {4, 5, 2}, 
{1, 4, 2}, {5, 2, 0}, {6, 5, 0}, {0, 2, 2}, {3, 0, 0}, {7, 3, 0}, {7, 5, 0}, {1, 5, 2}, {6, 0, 0}, {0, 1, 4}, 
{2, 7, 0}, {3, 1, 0}, {5, 1, 0}, {6, 3, 0}, {6, 4, 0}, {6, 6, 0}, {7, 0, 0}, {1, 0, 2}, {2, 0, 2}, {3, 2, 2}, 
{6, 2, 0}, {6, 7, 0}, {0, 0, 3}, {1, 1, 0}, {5, 0, 0}, {7, 1, 0}, {2, 1, 2}, {2, 2, 0}, {2, 3, 0}, {3, 3, 0},   // +121.833s 121840
{4, 7, 0}, {6, 1, 0}, {7, 4, 0}, {7, 6, 0}, {0, 0, 2}, {6, 7, 0}, {7, 3, 0}, {1, 2, 2}, {5, 6, 0}, {5, 7, 0}, 
{7, 5, 0}, {3, 4, 2}, {6, 0, 0}, {6, 6, 0}, {0, 1, 2}, {5, 5, 0}, {7, 0, 0}, {3, 5, 2}, {6, 4, 0}, {0, 2, 2}, 
{1, 3, 0}, {6, 3, 0}, {6, 5, 0}, {7, 1, 0}, {3, 6, 2}, {5, 4, 0}, {0, 3, 4}, {1, 4, 0}, {2, 0, 0}, {5, 3, 0}, 
{6, 2, 0}, {6, 7, 0}, {7, 3, 0}, {1, 6, 2}, {3, 7, 0}, {0, 4, 3}, {1, 5, 2}, {2, 1, 0}, {2, 2, 0}, {2, 3, 0}, 
{5, 2, 0}, {6, 6, 0}, {4, 0, 2}, {4, 6, 0}, {4, 7, 0}, {6, 1, 0}, {7, 0, 0}, {7, 4, 0}, {7, 6, 0}, {0, 5, 2},   // +122.300s 122304
{5, 1, 0}, {4, 1, 2}, {0, 6, 2}, {1, 0, 0}, {6, 0, 0}, {6, 5, 0}, {7, 1, 0}, {7, 5, 0}, {4, 2, 2}, {5, 0, 0}, 
{7, 2, 2}, {1, 1, 2}, {6, 4, 0}, {6, 5, 0}, {7, 3, 0}, {0, 0, 2}, {4, 3, 0}, {5, 6, 0}, {6, 3, 0}, {1, 2, 2}, 
{2, 0, 0}, {6, 6, 0}, {6, 2, 2}, {7, 0, 0}, {0, 1, 2}, {2, 4, 0}, {5, 5, 0}, {1, 3, 3}, {2, 1, 0}, {2, 3, 0}, 
{6, 1, 0}, {6, 7, 0}, {7, 1, 0}, {2, 2, 2}, {4, 7, 0}, {7, 4, 0}, {7, 6, 0}, {1, 4, 2}, {2, 5, 0}, {6, 5, 0}, 
{7, 5, 0}, {0, 2, 2}, {5, 4, 0}, {6, 0, 0}, {7, 3, 0}, {1, 5, 4}, {2, 6, 0}, {6, 4, 0}, {0, 3, 2}, {5, 3, 0},   // +122.833s 122832
{6, 3, 0}, {6, 6, 0}, {7, 0, 0}, {1, 0, 2}, {1, 7, 2}, {2, 0, 0}, {0, 4, 2}, {4, 4, 0}, {5, 2, 0}, {6, 2, 0}, 
{6, 7, 0}, {1, 1, 2}, {7, 1, 0}, {2, 1, 5}, {4, 5, 0}, {6, 7, 0}, {0, 5, 2}, {1, 2, 0}, {2, 2, 0}, {2, 3, 0}, 
{4, 6, 0}, {5, 1, 0}, {6, 1, 0}, {7, 3, 0}, {7, 4, 2}, {7, 6, 0}, {6, 6, 2}, {7, 0, 0}, {7, 5, 0}, {0, 6, 2}, 
{1, 3, 0}, {2, 7, 0}, {3, 0, 0}, {5, 0, 0}, {6, 0, 0}, {6, 5, 2}, {6, 4, 2}, {7, 1, 0}, {0, 6, 2}, {3, 1, 0}, 
{6, 3, 0}, {1, 4, 2}, {2, 0, 0}, {5, 6, 0}, {6, 7, 0}, {7, 3, 2}, {0, 5, 2}, {3, 2, 0}, {5, 5, 0}, {6, 2, 0},   // +123.367s 123360
{2, 1, 2}, {5, 7, 0}, {0, 4, 3}, {1, 5, 0}, {5, 4, 0}, {6, 1, 0}, {6, 6, 0}, {7, 0, 0}, {2, 2, 2}, {2, 3, 0}, 
{3, 3, 2}, {4, 6, 0}, {4, 7, 0}, {7, 4, 0}, {7, 5, 0}, {7, 6, 0}, {0, 3, 2}, {1, 0, 0}, {5, 3, 0}, {6, 0, 0}, 
{6, 5, 0}, {7, 1, 2}, {0, 2, 2}, {5, 2, 0}, {6, 4, 0}, {1, 1, 2}, {1, 6, 0}, {3, 4, 0}, {6, 3, 0}, {6, 5, 0}, 
{5, 1, 2}, {7, 3, 0}, {0, 1, 2}, {2, 0, 0}, {6, 6, 0}, {1, 2, 2}, {6, 2, 0}, {7, 0, 0}, {0, 0, 2}, {3, 5, 0}, 
{5, 0, 0}, {1, 3, 2}, {6, 7, 0}, {7, 1, 0}, {2, 1, 3}, {6, 1, 0}, {0, 6, 2}, {3, 6, 0}, {5, 6, 0}, {7, 2, 0},   // +123.867s 123872
{1, 4, 2}, {2, 2, 0}, {2, 3, 0}, {4, 6, 0}, {6, 5, 0}, {7, 3, 0}, {7, 4, 0}, {7, 6, 0}, {7, 5, 2}, {0, 5, 2}, 
{1, 5, 0}, {5, 5, 0}, {6, 0, 0}, {3, 7, 2}, {6, 6, 0}, {1, 0, 2}, {6, 4, 0}, {7, 0, 0}, {6, 3, 2}, {0, 4, 2}, 
{2, 0, 0}, {4, 0, 0}, {5, 4, 0}, {1, 1, 2}, {6, 2, 0}, {6, 7, 0}, {7, 1, 0}, {2, 1, 2}, {0, 3, 2}, {5, 3, 0}, 
{1, 2, 3}, {4, 1, 0}, {6, 1, 0}, {6, 7, 0}, {7, 3, 0}, {2, 3, 2}, {7, 5, 0}, {2, 2, 2}, {4, 7, 0}, {6, 0, 0}, 
{6, 6, 0}, {7, 4, 0}, {7, 6, 0}, {0, 2, 2}, {5, 2, 0}, {7, 0, 0}, {1, 3, 2}, {1, 7, 0}, {4, 2, 0}, {6, 4, 0},   // +124.367s 124368
{6, 3, 2}, {6, 5, 0}, {7, 1, 0}, {0, 1, 2}, {4, 3, 0}, {5, 1, 0}, {1, 4, 4}, {2, 0, 0}, {6, 7, 0}, {7, 3, 0}, 
{5, 0, 2}, {6, 2, 0}, {0, 0, 2}, {2, 4, 0}, {1, 5, 2}, {2, 1, 0}, {2, 7, 0}, {6, 6, 0}, {2, 5, 3}, {5, 6, 0}, 
{6, 1, 0}, {7, 0, 0}, {0, 0, 2}, {2, 2, 2}, {6, 5, 0}, {0, 1, 2}, {1, 0, 0}, {2, 3, 0}, {2, 6, 0}, {4, 6, 0}, 
{4, 7, 0}, {5, 5, 0}, {6, 0, 0}, {7, 1, 0}, {7, 4, 0}, {7, 5, 0}, {7, 6, 0}, {4, 4, 4}, {6, 4, 0}, {6, 5, 0}, 
{0, 2, 2}, {1, 1, 0}, {5, 4, 0}, {6, 3, 0}, {7, 3, 0}, {4, 5, 2}, {5, 7, 0}, {0, 3, 2}, {1, 2, 0}, {2, 0, 0},   // +124.900s 124896
{5, 3, 0}, {6, 6, 0}, {7, 0, 0}, {6, 2, 2}, {3, 0, 2}, {6, 7, 0}, {0, 4, 2}, {1, 3, 0}, {2, 1, 0}, {5, 2, 0}, 
{6, 1, 0}, {7, 1, 0}, {3, 1, 3}, {0, 5, 2}, {1, 4, 0}, {5, 1, 0}, {6, 5, 0}, {7, 5, 0}, {1, 6, 2}, {2, 3, 0}, 
{6, 0, 0}, {7, 3, 0}, {1, 5, 2}, {2, 2, 0}, {3, 2, 0}, {4, 7, 0}, {7, 4, 0}, {7, 6, 0}, {0, 6, 2}, {5, 0, 0}, 
{6, 4, 0}, {3, 3, 2}, {6, 3, 0}, {6, 6, 0}, {7, 0, 0}, {1, 0, 2}, {0, 0, 2}, {2, 0, 0}, {5, 6, 0}, {3, 4, 2}, 
{6, 2, 0}, {6, 7, 0}, {1, 1, 2}, {7, 1, 0}, {0, 1, 2}, {2, 1, 0}, {3, 5, 0}, {5, 5, 0}, {7, 2, 0}, {6, 7, 2},   // +125.400s 125392
{1, 2, 3}, {3, 6, 0}, {6, 1, 0}, {7, 3, 0}, {0, 2, 4}, {2, 2, 0}, {5, 4, 0}, {6, 6, 0}, {7, 0, 0}, {7, 5, 0}, 
{1, 3, 2}, {2, 3, 0}, {3, 7, 0}, {4, 6, 0}, {6, 0, 0}, {6, 5, 2}, {7, 4, 0}, {7, 6, 0}, {0, 3, 2}, {4, 0, 0}, 
{5, 3, 0}, {6, 4, 0}, {7, 1, 0}, {6, 3, 2}, {1, 4, 2}, {2, 0, 0}, {6, 7, 0}, {4, 1, 2}, {5, 2, 0}, {6, 2, 0}, 
{7, 3, 0}, {0, 4, 2}, {2, 1, 2}, {6, 6, 0}, {1, 5, 2}, {4, 2, 0}, {6, 1, 0}, {7, 0, 0}, {0, 5, 3}, {5, 1, 0}, 
{7, 5, 0}, {1, 7, 2}, {4, 3, 0}, {6, 0, 0}, {1, 0, 2}, {6, 5, 0}, {7, 1, 0}, {0, 6, 2}, {2, 2, 0}, {2, 3, 0},   // +125.933s 125936
{6, 4, 0}, {7, 4, 0}, {7, 6, 0}, {2, 4, 2}, {4, 6, 0}, {4, 7, 0}, {5, 0, 0}, {6, 3, 0}, {1, 1, 2}, {6, 5, 0}, 
{7, 3, 0}, {0, 6, 2}, {1, 2, 2}, {2, 0, 0}, {2, 5, 0}, {5, 6, 0}, {6, 6, 0}, {2, 7, 2}, {6, 2, 0}, {7, 0, 0}, 
{0, 5, 2}, {5, 5, 0}, {1, 3, 2}, {2, 1, 0}, {2, 6, 0}, {6, 7, 0}, {7, 1, 0}, {0, 4, 3}, {5, 4, 0}, {6, 1, 0}, 
{1, 4, 2}, {6, 5, 2}, {7, 3, 0}, {0, 3, 2}, {5, 3, 0}, {7, 5, 0}, {1, 5, 2}, {2, 2, 0}, {2, 3, 0}, {4, 4, 0}, 
{6, 0, 0}, {0, 2, 2}, {4, 6, 0}, {5, 2, 0}, {5, 7, 0}, {6, 6, 0}, {7, 4, 0}, {7, 6, 0}, {1, 0, 2}, {4, 5, 0},   // +126.400s 126400
{6, 4, 0}, {7, 0, 0}, {6, 3, 2}, {0, 1, 2}, {2, 0, 0}, {5, 1, 0}, {1, 1, 2}, {6, 2, 0}, {6, 7, 0}, {7, 1, 0}, 
{0, 0, 2}, {2, 1, 0}, {3, 0, 0}, {5, 0, 0}, {1, 2, 4}, {1, 6, 0}, {6, 1, 0}, {6, 7, 0}, {7, 3, 0}, {0, 6, 3}, 
{3, 1, 0}, {5, 6, 0}, {7, 5, 0}, {6, 0, 2}, {6, 6, 0}, {7, 0, 2}, {1, 3, 2}, {2, 3, 0}, {5, 5, 0}, {6, 4, 0}, 
{6, 5, 0}, {0, 5, 2}, {2, 2, 0}, {3, 2, 0}, {4, 7, 0}, {6, 3, 0}, {7, 1, 0}, {7, 4, 0}, {7, 6, 0}, {6, 7, 4}, 
{7, 2, 0}, {0, 4, 2}, {1, 4, 0}, {2, 0, 0}, {3, 3, 0}, {5, 4, 0}, {6, 2, 0}, {7, 3, 0}, {0, 3, 6}, {1, 5, 0},   // +126.967s 126960
{2, 1, 0}, {5, 3, 0}, {6, 6, 0}, {7, 0, 0}, {3, 4, 2}, {6, 1, 0}, {1, 0, 5}, {6, 5, 0}, {7, 5, 0}, {0, 2, 2}, 
{5, 2, 0}, {6, 0, 0}, {7, 1, 0}, {2, 2, 2}, {3, 5, 0}, {1, 1, 2}, {2, 3, 0}, {6, 4, 0}, {6, 5, 0}, {0, 1, 2}, 
{4, 6, 0}, {4, 7, 0}, {5, 1, 0}, {6, 3, 0}, {7, 3, 0}, {7, 4, 0}, {7, 6, 0}, {3, 6, 2}, {1, 2, 2}, {2, 0, 0}, 
{6, 6, 0}, {7, 0, 0}, {1, 7, 2}, {6, 2, 0}, {0, 0, 2}, {1, 3, 0}, {2, 1, 0}, {3, 7, 0}, {5, 0, 0}, {6, 7, 0}, 
{6, 1, 2}, {7, 1, 0}, {0, 0, 5}, {1, 4, 0}, {5, 6, 0}, {6, 5, 0}, {7, 5, 0}, {4, 0, 2}, {6, 0, 0}, {7, 3, 0},   // +127.467s 127472
{0, 1, 2}, {1, 5, 0}, {5, 5, 0}, {6, 4, 2}, {2, 2, 2}, {2, 3, 0}, {2, 7, 0}, {4, 1, 0}, {6, 3, 0}, {6, 6, 0}, 
{0, 2, 2}, {1, 0, 0}, {4, 7, 0}, {5, 4, 0}, {7, 0, 0}, {7, 4, 0}, {7, 6, 0}, {2, 0, 2}, {0, 3, 2}, {5, 3, 0}, 
{6, 2, 0}, {6, 7, 0}, {1, 1, 2}, {4, 2, 0}, {7, 1, 0}, {2, 1, 2}, {0, 4, 2}, {5, 2, 0}, {6, 1, 0}, {6, 7, 0}, 
{1, 2, 3}, {5, 7, 0}, {7, 3, 0}, {0, 5, 2}, {4, 3, 0}, {5, 1, 0}, {6, 6, 0}, {7, 0, 2}, {7, 5, 0}, {0, 6, 2}, 
{1, 3, 0}, {6, 0, 0}, {2, 4, 2}, {5, 0, 0}, {6, 5, 0}, {1, 4, 14}, {1, 6, 0}, {6, 4, 0}, {6, 7, 0}, {2, 2, 2},   // +128.200s 128192
{2, 3, 0}, {6, 3, 0}, {7, 1, 0}, {0, 0, 3}, {4, 6, 0}, {7, 4, 0}, {7, 6, 0}, {2, 5, 2}, {5, 6, 0}, {1, 5, 2}, 
{2, 0, 0}, {6, 6, 0}, {6, 2, 2}, {7, 3, 0}, {2, 1, 2}, {2, 6, 0}, {0, 1, 2}, {5, 5, 0}, {1, 0, 2}, {4, 4, 0}, 
{6, 1, 0}, {6, 5, 0}, {7, 2, 0}, {7, 0, 2}, {7, 5, 0}, {0, 2, 2}, {5, 4, 2}, {6, 5, 0}, {1, 1, 2}, {4, 5, 0}, 
{6, 0, 0}, {2, 3, 2}, {5, 3, 0}, {6, 4, 0}, {7, 1, 0}, {0, 3, 3}, {1, 2, 0}, {2, 2, 0}, {4, 6, 0}, {4, 7, 0}, 
{6, 6, 0}, {7, 4, 0}, {7, 6, 0}, {3, 0, 2}, {6, 3, 0}, {1, 3, 2}, {2, 0, 0}, {6, 7, 0}, {7, 3, 0}, {0, 4, 2},   // +128.733s 128736
{3, 1, 0}, {5, 2, 0}, {6, 2, 2}, {7, 0, 0}, {1, 4, 2}, {3, 2, 0}, {6, 5, 0}, {2, 1, 2}, {5, 1, 0}, {7, 1, 0}, 
{0, 5, 2}, {1, 5, 0}, {1, 7, 2}, {3, 3, 0}, {6, 1, 0}, {6, 6, 0}, {7, 3, 2}, {7, 5, 0}, {0, 6, 2}, {1, 0, 0}, 
{3, 4, 0}, {5, 0, 0}, {6, 0, 2}, {2, 2, 3}, {2, 3, 0}, {4, 6, 0}, {6, 4, 0}, {6, 7, 0}, {7, 4, 0}, {7, 6, 0}, 
{0, 6, 2}, {1, 1, 0}, {3, 5, 0}, {5, 6, 0}, {7, 0, 0}, {2, 0, 2}, {6, 3, 0}, {0, 5, 2}, {5, 5, 0}, {6, 7, 0}, 
{1, 2, 2}, {2, 1, 0}, {2, 7, 0}, {3, 6, 0}, {7, 1, 0}, {6, 2, 2}, {6, 6, 0}, {0, 4, 2}, {3, 7, 0}, {5, 4, 0},   // +129.233s 129232
{1, 3, 2}, {6, 1, 0}, {7, 3, 0}, {7, 5, 0}, {0, 3, 2}, {5, 3, 0}, {6, 5, 0}, {4, 0, 2}, {6, 0, 2}, {6, 4, 0}, 
{7, 0, 0}, {0, 2, 3}, {1, 4, 0}, {4, 1, 0}, {5, 2, 0}, {5, 7, 0}, {6, 7, 0}, {2, 2, 2}, {2, 3, 0}, {4, 7, 0}, 
{7, 1, 0}, {7, 4, 0}, {7, 6, 0}, {0, 1, 2}, {2, 0, 0}, {5, 1, 0}, {6, 3, 0}, {4, 2, 2}, {6, 6, 0}, {1, 5, 2}, 
{5, 0, 0}, {7, 3, 0}, {0, 0, 2}, {4, 3, 0}, {6, 2, 0}, {2, 1, 2}, {1, 0, 2}, {1, 6, 0}, {6, 5, 0}, {0, 6, 2}, 
{2, 4, 0}, {5, 6, 0}, {6, 1, 0}, {7, 0, 0}, {6, 5, 2}, {1, 1, 2}, {7, 5, 0}, {0, 5, 2}, {2, 5, 0}, {5, 5, 0},   // +129.767s 129760
{7, 1, 0}, {1, 2, 3}, {6, 0, 0}, {6, 4, 0}, {6, 6, 0}, {2, 2, 2}, {2, 3, 2}, {4, 6, 0}, {4, 7, 0}, {5, 4, 0}, 
{6, 7, 0}, {7, 2, 0}, {7, 3, 0}, {7, 4, 0}, {7, 6, 0}, {0, 4, 2}, {1, 3, 0}, {2, 0, 0}, {2, 6, 0}, {6, 3, 0}, 
{7, 0, 2}, {1, 4, 2}, {6, 2, 0}, {6, 5, 0}, {0, 3, 2}, {2, 1, 0}, {4, 4, 0}, {5, 3, 0}, {7, 1, 2}, {1, 5, 2}, 
{6, 1, 0}, {7, 5, 0}, {0, 2, 2}, {5, 2, 0}, {6, 6, 0}, {1, 0, 2}, {4, 5, 0}, {6, 0, 0}, {7, 3, 0}, {6, 4, 2}, 
{6, 7, 2}, {0, 1, 3}, {1, 1, 0}, {2, 3, 0}, {5, 1, 0}, {6, 3, 0}, {7, 0, 0}, {2, 0, 2}, {2, 2, 0}, {3, 0, 0},   // +130.267s 130272
{4, 7, 0}, {7, 4, 0}, {7, 6, 0}, {6, 7, 2}, {0, 0, 2}, {1, 2, 0}, {5, 0, 0}, {1, 7, 2}, {2, 1, 0}, {3, 1, 0}, 
{6, 2, 0}, {7, 1, 0}, {6, 6, 2}, {0, 0, 2}, {5, 6, 0}, {1, 3, 2}, {3, 2, 0}, {6, 1, 0}, {6, 5, 0}, {7, 3, 0}, 
{5, 5, 2}, {7, 5, 0}, {0, 1, 2}, {7, 0, 0}, {6, 7, 2}, {0, 2, 3}, {1, 4, 0}, {2, 7, 0}, {3, 3, 0}, {5, 4, 0}, 
{6, 0, 0}, {6, 4, 0}, {7, 1, 2}, {2, 2, 2}, {2, 3, 0}, {4, 6, 0}, {6, 3, 0}, {6, 6, 0}, {0, 3, 2}, {1, 5, 0}, 
{2, 0, 0}, {3, 4, 0}, {5, 3, 0}, {7, 4, 0}, {7, 6, 0}, {7, 3, 2}, {0, 4, 2}, {2, 1, 0}, {5, 2, 0}, {6, 2, 0},   // +130.767s 130768
{6, 5, 2}, {1, 0, 2}, {3, 5, 0}, {6, 1, 0}, {7, 0, 0}, {7, 5, 0}, {0, 5, 2}, {5, 1, 0}, {5, 7, 0}, {6, 5, 2}, 
{0, 6, 2}, {1, 1, 0}, {5, 0, 0}, {6, 0, 0}, {6, 4, 0}, {7, 1, 0}, {3, 6, 2}, {6, 6, 0}, {1, 2, 2}, {0, 0, 3}, 
{5, 6, 0}, {6, 3, 0}, {1, 3, 6}, {1, 6, 0}, {2, 3, 0}, {6, 7, 0}, {7, 3, 0}, {7, 4, 0}, {7, 6, 0}, {0, 1, 2}, 
{2, 0, 0}, {2, 2, 0}, {3, 7, 0}, {4, 6, 0}, {4, 7, 0}, {5, 5, 0}, {6, 2, 0}, {1, 4, 2}, {7, 0, 0}, {2, 1, 2}, 
{6, 5, 0}, {0, 2, 2}, {5, 4, 0}, {6, 1, 0}, {1, 5, 2}, {4, 0, 0}, {7, 1, 0}, {6, 6, 2}, {1, 0, 2}, {7, 5, 0},   // +131.367s 131360
{0, 3, 2}, {4, 1, 0}, {5, 3, 0}, {6, 0, 0}, {7, 2, 0}, {7, 3, 0}, {1, 1, 5}, {6, 4, 0}, {6, 7, 0}, {0, 4, 2}, 
{6, 3, 0}, {7, 0, 0}, {2, 2, 2}, {2, 3, 0}, {4, 2, 0}, {4, 6, 0}, {5, 2, 0}, {1, 2, 2}, {2, 0, 0}, {6, 2, 0}, 
{6, 7, 0}, {7, 4, 0}, {7, 6, 0}, {0, 5, 4}, {2, 1, 0}, {4, 3, 0}, {5, 1, 0}, {6, 1, 0}, {6, 6, 0}, {7, 1, 0}, 
{1, 3, 4}, {6, 5, 0}, {0, 6, 2}, {2, 4, 0}, {5, 0, 0}, {6, 0, 0}, {7, 3, 0}, {7, 5, 0}, {6, 4, 5}, {6, 7, 0}, 
{7, 0, 0}, {0, 6, 2}, {1, 4, 0}, {2, 5, 0}, {5, 6, 0}, {6, 3, 0}, {1, 7, 2}, {7, 1, 0}, {0, 5, 2}, {1, 5, 2},   // +131.933s 131936
{2, 0, 0}, {2, 3, 0}, {2, 6, 0}, {5, 5, 0}, {6, 2, 0}, {6, 6, 0}, {7, 4, 0}, {7, 6, 0}, {2, 2, 2}, {4, 7, 0}, 
{7, 3, 0}, {0, 4, 2}, {4, 4, 0}, {5, 4, 0}, {1, 0, 2}, {6, 5, 0}, {0, 3, 2}, {2, 1, 0}, {4, 5, 0}, {6, 1, 0}, 
{5, 3, 2}, {7, 0, 0}, {1, 1, 2}, {2, 7, 0}, {6, 5, 0}, {0, 2, 2}, {3, 0, 0}, {5, 2, 0}, {6, 0, 0}, {7, 5, 0}, 
{7, 1, 2}, {0, 1, 3}, {1, 2, 0}, {6, 6, 0}, {3, 1, 2}, {5, 1, 0}, {6, 3, 0}, {6, 4, 0}, {1, 3, 2}, {6, 7, 0}, 
{7, 3, 0}, {0, 0, 2}, {2, 0, 0}, {2, 2, 0}, {3, 2, 0}, {5, 0, 0}, {2, 3, 2}, {5, 7, 0}, {6, 2, 0}, {7, 4, 0},   // +132.367s 132368
{7, 6, 0}, {1, 4, 2}, {2, 1, 0}, {3, 3, 0}, {4, 6, 0}, {4, 7, 0}, {6, 5, 0}, {7, 0, 0}, {0, 6, 2}, {5, 6, 0}, 
{6, 1, 0}, {1, 5, 2}, {3, 4, 0}, {7, 1, 0}, {7, 5, 2}, {0, 5, 2}, {5, 5, 0}, {6, 0, 0}, {6, 6, 0}, {1, 0, 2}, 
{3, 5, 0}, {7, 3, 0}, {1, 6, 2}, {6, 4, 0}, {6, 3, 3}, {6, 7, 0}, {0, 4, 2}, {1, 1, 0}, {3, 6, 0}, {5, 4, 0}, 
{2, 0, 2}, {7, 0, 0}, {3, 7, 2}, {6, 2, 0}, {6, 7, 0}, {0, 3, 2}, {1, 2, 0}, {2, 2, 0}, {2, 3, 0}, {4, 7, 0}, 
{5, 3, 0}, {7, 4, 0}, {7, 6, 0}, {6, 6, 2}, {7, 1, 0}, {2, 1, 2}, {4, 0, 0}, {6, 1, 0}, {7, 2, 0}, {1, 3, 2},   // +132.867s 132864
{0, 2, 2}, {4, 1, 0}, {5, 2, 0}, {6, 5, 0}, {7, 3, 0}, {7, 5, 2}, {6, 0, 2}, {7, 0, 0}, {0, 1, 2}, {1, 4, 0}, 
{4, 2, 0}, {5, 1, 0}, {6, 7, 0}, {6, 4, 3}, {4, 3, 2}, {6, 3, 0}, {7, 1, 0}, {0, 0, 2}, {1, 5, 0}, {6, 6, 0}, 
{2, 0, 2}, {5, 0, 0}, {6, 2, 0}, {2, 2, 2}, {2, 3, 0}, {2, 4, 0}, {7, 3, 0}, {0, 0, 2}, {2, 1, 0}, {4, 6, 0}, 
{7, 4, 0}, {7, 6, 0}, {1, 0, 2}, {5, 6, 0}, {6, 1, 0}, {6, 5, 0}, {2, 5, 2}, {7, 0, 0}, {0, 1, 2}, {5, 5, 0}, 
{6, 0, 0}, {7, 5, 0}, {1, 1, 2}, {1, 7, 0}, {6, 5, 0}, {0, 2, 2}, {1, 2, 2}, {2, 6, 0}, {5, 4, 0}, {6, 3, 0},   // +133.400s 133392
{6, 4, 0}, {6, 6, 0}, {7, 1, 0}, {0, 3, 5}, {5, 3, 0}, {1, 3, 2}, {2, 0, 0}, {4, 4, 0}, {6, 7, 0}, {7, 3, 0}, 
{0, 4, 2}, {5, 2, 0}, {6, 2, 0}, {1, 4, 8}, {2, 3, 0}, {2, 7, 0}, {5, 1, 0}, {6, 5, 0}, {7, 0, 0}, {7, 4, 0}, 
{7, 6, 0}, {0, 5, 2}, {2, 1, 0}, {2, 2, 0}, {4, 5, 0}, {4, 6, 0}, {4, 7, 0}, {6, 1, 0}, {1, 5, 2}, {0, 6, 2}, 
{5, 0, 0}, {6, 6, 0}, {7, 1, 0}, {3, 0, 2}, {6, 0, 0}, {7, 5, 0}, {1, 0, 3}, {7, 3, 0}, {0, 0, 2}, {5, 6, 0}, 
{5, 7, 2}, {6, 3, 0}, {6, 4, 0}, {6, 7, 0}, {1, 1, 2}, {3, 1, 0}, {0, 1, 2}, {2, 0, 0}, {5, 5, 0}, {6, 2, 0},   // +133.967s 133968
{7, 0, 0}, {6, 7, 2}, {1, 2, 2}, {3, 2, 0}, {2, 1, 2}, {2, 2, 0}, {4, 6, 0}, {6, 1, 0}, {6, 6, 0}, {7, 1, 0}, 
{0, 2, 2}, {2, 3, 0}, {5, 4, 0}, {7, 4, 0}, {7, 6, 0}, {1, 3, 2}, {6, 0, 0}, {7, 5, 0}, {1, 6, 2}, {3, 3, 0}, 
{6, 5, 0}, {7, 3, 0}, {0, 3, 2}, {5, 3, 0}, {6, 3, 3}, {6, 4, 0}, {1, 4, 2}, {6, 7, 0}, {7, 0, 0}, {3, 4, 2}, 
{0, 4, 2}, {2, 0, 0}, {5, 2, 0}, {7, 1, 0}, {6, 2, 2}, {6, 6, 0}, {1, 5, 2}, {3, 5, 0}, {0, 5, 2}, {5, 1, 0}, 
{7, 2, 0}, {7, 3, 0}, {2, 1, 2}, {2, 3, 0}, {4, 7, 0}, {6, 1, 0}, {1, 0, 2}, {2, 2, 0}, {3, 6, 0}, {6, 5, 0},   // +134.500s 134496
{7, 4, 0}, {7, 6, 0}, {5, 0, 2}, {0, 6, 2}, {7, 0, 0}, {1, 1, 2}, {6, 0, 0}, {6, 5, 0}, {7, 5, 0}, {3, 7, 3}, 
{0, 6, 2}, {1, 2, 0}, {5, 6, 0}, {6, 3, 0}, {6, 4, 0}, {6, 6, 0}, {7, 1, 0}, {0, 5, 4}, {4, 0, 0}, {5, 5, 0}, 
{6, 7, 0}, {1, 3, 2}, {2, 0, 0}, {6, 2, 0}, {7, 3, 0}, {5, 4, 2}, {0, 4, 2}, {1, 4, 0}, {6, 1, 0}, {6, 5, 0}, 
{7, 0, 0}, {2, 1, 2}, {4, 1, 0}, {0, 3, 2}, {1, 7, 0}, {2, 2, 0}, {2, 3, 0}, {4, 6, 0}, {4, 7, 0}, {5, 3, 0}, 
{7, 4, 0}, {7, 6, 0}, {1, 5, 2}, {6, 0, 0}, {7, 1, 0}, {7, 5, 0}, {0, 2, 2}, {4, 2, 0}, {5, 2, 0}, {6, 6, 0},   // +134.967s 134960
{1, 0, 2}, {6, 3, 3}, {6, 4, 0}, {7, 3, 0}, {0, 1, 2}, {5, 1, 0}, {6, 7, 0}, {1, 1, 2}, {4, 3, 0}, {0, 0, 2}, 
{2, 0, 0}, {5, 0, 0}, {7, 0, 0}, {2, 7, 2}, {6, 2, 0}, {6, 7, 0}, {1, 2, 2}, {2, 4, 0}, {0, 6, 2}, {2, 1, 0}, 
{5, 6, 0}, {2, 5, 2}, {6, 1, 0}, {6, 6, 0}, {7, 1, 0}, {2, 2, 2}, {2, 3, 0}, {4, 7, 0}, {7, 4, 0}, {7, 6, 0}, 
{1, 3, 2}, {6, 5, 0}, {0, 5, 2}, {2, 6, 0}, {5, 5, 0}, {7, 3, 0}, {7, 5, 0}, {5, 7, 2}, {6, 0, 0}, {1, 4, 3}, 
{4, 4, 0}, {6, 7, 0}, {7, 0, 0}, {0, 4, 2}, {5, 4, 0}, {6, 4, 0}, {4, 5, 2}, {6, 3, 0}, {7, 1, 2}, {1, 5, 2},   // +135.567s 135568
{2, 0, 0}, {5, 3, 0}, {6, 2, 0}, {6, 6, 0}, {0, 3, 2}, {3, 0, 0}, {1, 6, 2}, {2, 1, 0}, {6, 1, 0}, {7, 3, 0}, 
{6, 5, 2}, {0, 2, 2}, {1, 0, 0}, {2, 2, 0}, {3, 1, 0}, {4, 6, 0}, {5, 2, 0}, {2, 3, 2}, {6, 0, 0}, {7, 0, 0}, 
{7, 4, 0}, {7, 5, 0}, {7, 6, 0}, {3, 2, 2}, {6, 5, 0}, {1, 1, 2}, {5, 1, 0}, {6, 4, 0}, {0, 1, 3}, {3, 3, 0}, 
{6, 3, 0}, {6, 6, 0}, {7, 1, 0}, {1, 2, 2}, {7, 2, 0}, {0, 0, 4}, {1, 3, 0}, {2, 0, 0}, {3, 4, 0}, {5, 0, 0}, 
{6, 2, 0}, {6, 7, 0}, {7, 3, 0}, {0, 0, 6}, {1, 4, 0}, {3, 5, 0}, {5, 6, 0}, {6, 5, 0}, {7, 0, 0}, {2, 1, 2},   // +136.067s 136064
{6, 1, 0}, {3, 6, 2}, {0, 1, 7}, {1, 5, 0}, {2, 2, 0}, {2, 3, 0}, {6, 6, 0}, {7, 1, 0}, {7, 4, 0}, {7, 6, 0}, 
{3, 7, 2}, {4, 6, 0}, {4, 7, 0}, {5, 5, 0}, {6, 0, 0}, {7, 5, 0}, {1, 0, 2}, {0, 2, 2}, {5, 4, 0}, {7, 3, 0}, 
{4, 0, 2}, {6, 3, 0}, {6, 4, 0}, {6, 7, 0}, {0, 3, 2}, {1, 1, 0}, {2, 0, 2}, {4, 1, 0}, {5, 3, 0}, {6, 2, 0}, 
{1, 7, 2}, {6, 7, 0}, {7, 0, 0}, {0, 4, 2}, {1, 2, 0}, {2, 1, 0}, {4, 2, 0}, {5, 2, 0}, {6, 1, 2}, {6, 6, 0}, 
{0, 5, 2}, {5, 1, 0}, {7, 1, 0}, {4, 3, 2}, {6, 0, 0}, {7, 5, 0}, {1, 3, 3}, {2, 2, 0}, {4, 6, 0}, {6, 5, 0},   // +136.600s 136608
{0, 6, 2}, {2, 3, 0}, {5, 0, 0}, {7, 3, 0}, {7, 4, 0}, {7, 6, 0}, {2, 4, 2}, {2, 7, 0}, {6, 3, 0}, {6, 4, 0}, 
{6, 7, 0}, {1, 4, 2}, {0, 0, 2}, {5, 6, 0}, {7, 0, 0}, {2, 0, 2}, {6, 2, 0}, {2, 5, 2}, {6, 6, 0}, {7, 1, 0}, 
{0, 1, 2}, {1, 5, 0}, {5, 5, 0}, {2, 1, 4}, {2, 6, 0}, {6, 1, 0}, {6, 5, 0}, {7, 3, 0}, {0, 2, 2}, {5, 7, 0}, 
{1, 0, 2}, {5, 4, 0}, {6, 0, 3}, {6, 5, 0}, {7, 0, 0}, {7, 5, 0}, {1, 1, 2}, {2, 2, 0}, {2, 3, 0}, {4, 4, 0}, 
{4, 7, 0}, {7, 4, 0}, {7, 6, 0}, {0, 3, 2}, {5, 3, 0}, {1, 2, 2}, {6, 3, 0}, {6, 4, 0}, {6, 6, 0}, {4, 5, 2},   // +137.133s 137136
{7, 1, 0}, {0, 4, 2}, {1, 6, 0}, {2, 0, 0}, {6, 7, 0}, {1, 3, 2}, {5, 2, 0}, {6, 2, 0}, {7, 3, 2}, {1, 4, 2}, 
{2, 1, 0}, {3, 0, 0}, {6, 1, 0}, {6, 5, 0}, {0, 5, 2}, {5, 1, 0}, {7, 0, 0}, {3, 1, 2}, {1, 5, 2}, {6, 0, 0}, 
{7, 5, 0}, {0, 6, 2}, {6, 6, 0}, {7, 1, 0}, {2, 2, 3}, {2, 3, 0}, {5, 0, 0}, {7, 2, 0}, {1, 0, 2}, {3, 2, 0}, 
{4, 6, 0}, {4, 7, 0}, {6, 3, 0}, {6, 4, 0}, {7, 4, 0}, {7, 6, 0}, {0, 6, 2}, {5, 6, 0}, {6, 7, 0}, {7, 3, 0}, 
{1, 1, 4}, {2, 0, 0}, {3, 3, 0}, {6, 2, 0}, {0, 5, 2}, {5, 5, 0}, {6, 7, 0}, {7, 0, 0}, {0, 4, 4}, {1, 2, 0},   // +137.667s 137664
{2, 1, 0}, {5, 4, 0}, {6, 1, 0}, {6, 6, 0}, {3, 4, 2}, {7, 1, 2}, {0, 3, 2}, {5, 3, 0}, {6, 5, 0}, {1, 3, 2}, 
{3, 5, 0}, {6, 0, 0}, {7, 3, 0}, {7, 5, 0}, {0, 2, 3}, {2, 3, 0}, {4, 7, 0}, {5, 2, 0}, {7, 4, 0}, {7, 6, 0}, 
{2, 2, 2}, {6, 7, 0}, {0, 1, 2}, {1, 4, 0}, {6, 3, 0}, {6, 4, 0}, {7, 0, 0}, {1, 7, 2}, {3, 6, 0}, {5, 1, 0}, 
{2, 0, 2}, {6, 2, 0}, {6, 6, 0}, {7, 1, 0}, {0, 0, 2}, {5, 0, 0}, {1, 5, 2}, {3, 7, 0}, {2, 1, 2}, {6, 1, 0}, 
{7, 3, 0}, {0, 6, 2}, {5, 6, 0}, {6, 5, 0}, {1, 0, 2}, {6, 0, 0}, {7, 5, 0}, {2, 7, 2}, {4, 0, 0}, {0, 5, 2},   // +138.200s 138192
{6, 5, 0}, {7, 0, 0}, {1, 1, 3}, {2, 2, 0}, {5, 5, 0}, {6, 4, 0}, {2, 3, 2}, {4, 6, 0}, {6, 3, 0}, {6, 6, 0}, 
{7, 4, 0}, {7, 6, 0}, {1, 2, 2}, {4, 1, 0}, {7, 1, 0}, {0, 4, 2}, {2, 0, 0}, {5, 4, 0}, {1, 3, 2}, {6, 2, 0}, 
{6, 7, 0}, {4, 2, 2}, {7, 3, 0}, {0, 3, 2}, {5, 7, 0}, {1, 4, 2}, {2, 1, 0}, {5, 3, 0}, {6, 1, 0}, {6, 5, 0}, 
{4, 3, 2}, {7, 0, 0}, {0, 2, 4}, {1, 5, 0}, {5, 2, 0}, {6, 6, 0}, {7, 1, 0}, {2, 4, 2}, {6, 0, 0}, {7, 5, 0}, 
{1, 0, 3}, {1, 1, 8}, {1, 6, 0}, {2, 3, 0}, {6, 3, 0}, {6, 4, 0}, {6, 7, 0}, {7, 4, 0}, {7, 6, 0}, {0, 1, 2},   // +138.800s 138800
{2, 2, 0}, {2, 5, 0}, {4, 6, 0}, {4, 7, 0}, {5, 1, 0}, {7, 3, 0}, {2, 0, 2}, {6, 2, 0}, {2, 6, 2}, {6, 7, 0}, 
{0, 0, 2}, {1, 2, 0}, {5, 0, 0}, {7, 0, 0}, {2, 1, 2}, {4, 4, 0}, {6, 1, 0}, {6, 6, 0}, {0, 0, 4}, {1, 3, 0}, 
{6, 0, 0}, {7, 1, 0}, {7, 2, 0}, {7, 5, 0}, {4, 5, 3}, {5, 6, 0}, {6, 5, 0}, {0, 1, 4}, {6, 3, 0}, {6, 4, 0}, 
{7, 3, 0}, {1, 4, 2}, {3, 0, 0}, {5, 5, 0}, {6, 7, 0}, {0, 2, 2}, {2, 2, 0}, {2, 3, 0}, {2, 0, 2}, {4, 6, 0}, 
{5, 4, 0}, {7, 0, 0}, {7, 4, 0}, {7, 6, 0}, {1, 5, 2}, {3, 1, 0}, {6, 2, 0}, {6, 6, 0}, {0, 3, 2}, {3, 2, 0},   // +139.267s 139264
{5, 3, 0}, {7, 1, 0}, {0, 4, 4}, {2, 1, 0}, {6, 1, 0}, {1, 0, 2}, {3, 3, 0}, {5, 2, 0}, {6, 5, 0}, {7, 3, 0}, 
{0, 5, 5}, {5, 1, 0}, {1, 1, 2}, {1, 7, 0}, {3, 4, 0}, {6, 0, 0}, {6, 5, 0}, {7, 5, 0}, {0, 6, 2}, {5, 0, 0}, 
{7, 0, 0}, {1, 2, 2}, {3, 5, 0}, {6, 6, 0}, {2, 3, 2}, {6, 3, 0}, {6, 4, 0}, {0, 0, 2}, {2, 2, 0}, {4, 7, 0}, 
{5, 6, 0}, {7, 1, 0}, {7, 4, 0}, {7, 6, 0}, {1, 3, 2}, {2, 0, 0}, {3, 6, 0}, {6, 2, 0}, {6, 7, 0}, {1, 4, 4}, 
{3, 7, 0}, {6, 1, 0}, {6, 5, 0}, {7, 3, 0}, {0, 1, 2}, {2, 1, 0}, {2, 7, 0}, {5, 5, 0}, {7, 0, 2}, {1, 5, 2},   // +139.800s 139792
{4, 0, 0}, {6, 0, 0}, {7, 5, 0}, {0, 2, 3}, {5, 4, 0}, {6, 6, 0}, {1, 0, 2}, {7, 1, 0}, {4, 1, 2}, {6, 3, 0}, 
{6, 4, 0}, {0, 3, 2}, {6, 7, 0}, {1, 1, 2}, {4, 2, 0}, {5, 3, 0}, {5, 7, 0}, {7, 3, 0}, {2, 0, 2}, {2, 2, 0}, 
{2, 3, 0}, {6, 2, 0}, {7, 4, 0}, {7, 6, 0}, {4, 6, 2}, {4, 7, 0}, {6, 7, 0}, {0, 4, 2}, {1, 2, 0}, {4, 3, 0}, 
{5, 2, 0}, {7, 0, 0}, {2, 1, 4}, {6, 1, 0}, {6, 6, 0}, {0, 5, 2}, {1, 3, 0}, {2, 4, 0}, {5, 1, 3}, {6, 5, 0}, 
{7, 1, 0}, {1, 6, 2}, {6, 0, 0}, {7, 5, 0}, {0, 6, 4}, {1, 4, 0}, {2, 5, 0}, {6, 7, 0}, {7, 3, 0}, {5, 0, 2},   // +140.333s 140336
{6, 3, 0}, {6, 4, 0}, {7, 0, 2}, {0, 6, 2}, {2, 0, 0}, {2, 3, 0}, {2, 6, 0}, {4, 7, 0}, {7, 4, 0}, {7, 6, 0}, 
{1, 5, 2}, {2, 2, 0}, {5, 6, 0}, {6, 2, 0}, {6, 6, 0}, {0, 5, 2}, {7, 1, 0}, {2, 1, 2}, {4, 4, 0}, {5, 5, 0}, 
{6, 1, 0}, {7, 2, 0}, {1, 0, 2}, {6, 5, 0}, {0, 4, 2}, {5, 4, 0}, {7, 3, 0}, {6, 0, 2}, {7, 5, 0}, {0, 3, 3}, 
{1, 1, 0}, {4, 5, 0}, {5, 3, 0}, {6, 5, 0}, {6, 3, 2}, {6, 4, 0}, {7, 0, 0}, {6, 6, 2}, {0, 2, 2}, {1, 2, 0}, 
{5, 2, 0}, {3, 0, 2}, {0, 1, 2}, {1, 3, 0}, {2, 0, 0}, {2, 2, 0}, {5, 1, 0}, {6, 2, 0}, {6, 7, 0}, {7, 1, 0},   // +140.800s 140800
{2, 3, 2}, {4, 6, 0}, {7, 4, 0}, {7, 6, 0}, {0, 0, 2}, {3, 1, 0}, {7, 3, 0}, {1, 4, 2}, {2, 1, 0}, {5, 0, 0}, 
{6, 1, 0}, {6, 5, 0}, {0, 6, 4}, {1, 5, 0}, {1, 7, 0}, {7, 0, 0}, {3, 2, 2}, {5, 6, 0}, {6, 6, 0}, {1, 0, 3}, 
{6, 0, 0}, {7, 1, 0}, {7, 5, 0}, {0, 5, 4}, {5, 5, 0}, {1, 1, 2}, {3, 3, 0}, {6, 3, 0}, {6, 4, 0}, {6, 7, 0}, 
{7, 3, 0}, {0, 4, 4}, {2, 0, 0}, {2, 3, 0}, {2, 7, 0}, {1, 2, 2}, {2, 2, 0}, {3, 4, 0}, {5, 4, 0}, {6, 2, 0}, 
{6, 7, 0}, {7, 4, 0}, {7, 6, 0}, {2, 1, 2}, {4, 6, 0}, {4, 7, 0}, {7, 0, 0}, {6, 1, 2}, {6, 6, 0}, {0, 3, 2},   // +141.333s 141328
{3, 5, 0}, {5, 3, 0}, {1, 3, 2}, {7, 1, 0}, {7, 5, 0}, {6, 0, 2}, {6, 5, 0}, {0, 2, 3}, {3, 6, 2}, {5, 2, 0}, 
{5, 7, 0}, {6, 4, 0}, {7, 3, 0}, {1, 4, 2}, {6, 3, 0}, {6, 7, 0}, {0, 1, 4}, {2, 0, 0}, {3, 7, 0}, {5, 1, 0}, 
{7, 0, 0}, {1, 5, 2}, {6, 2, 0}, {6, 6, 0}, {2, 2, 2}, {2, 3, 0}, {4, 6, 0}, {7, 1, 0}, {0, 0, 2}, {5, 0, 0}, 
{7, 4, 0}, {7, 6, 0}, {1, 6, 2}, {2, 1, 0}, {4, 0, 0}, {6, 1, 0}, {6, 5, 0}, {1, 0, 2}, {7, 3, 0}, {0, 0, 2}, 
{5, 6, 0}, {4, 1, 2}, {6, 5, 0}, {7, 5, 0}, {1, 1, 3}, {6, 0, 0}, {7, 0, 0}, {0, 1, 2}, {5, 5, 0}, {1, 2, 2},   // +141.900s 141904
{6, 4, 0}, {6, 6, 0}, {0, 2, 2}, {4, 2, 0}, {5, 4, 0}, {6, 3, 0}, {7, 2, 0}, {6, 7, 2}, {7, 1, 0}, {0, 3, 2}, 
{1, 3, 0}, {2, 0, 0}, {6, 2, 0}, {2, 3, 2}, {4, 3, 0}, {5, 3, 0}, {7, 4, 0}, {7, 6, 0}, {1, 4, 2}, {2, 1, 0}, 
{2, 2, 0}, {4, 7, 0}, {6, 5, 0}, {7, 3, 0}, {0, 4, 2}, {5, 2, 0}, {6, 1, 0}, {1, 5, 2}, {2, 4, 0}, {7, 0, 0}, 
{0, 5, 2}, {6, 0, 0}, {7, 5, 0}, {6, 6, 2}, {1, 0, 13}, {7, 1, 0}, {0, 6, 2}, {2, 5, 0}, {5, 1, 0}, {6, 3, 0}, 
{6, 4, 0}, {6, 7, 0}, {2, 2, 6}, {6, 7, 0}, {1, 1, 2}, {2, 0, 0}, {2, 3, 0}, {6, 2, 0}, {7, 3, 0}, {7, 4, 0},   // +142.567s 142560
{7, 6, 0}, {5, 0, 5}, {0, 0, 2}, {1, 7, 0}, {2, 6, 0}, {4, 6, 0}, {4, 7, 0}, {6, 6, 0}, {1, 2, 6}, {2, 1, 0}, 
{6, 1, 0}, {0, 1, 2}, {5, 6, 0}, {7, 0, 0}, {4, 4, 2}, {6, 5, 0}, {0, 2, 4}, {1, 3, 0}, {2, 7, 0}, {5, 5, 0}, 
{6, 0, 0}, {7, 1, 0}, {7, 5, 0}, {2, 3, 2}, {4, 5, 0}, {6, 7, 0}, {7, 4, 0}, {7, 6, 0}, {2, 2, 2}, {1, 4, 2}, 
{6, 3, 0}, {6, 4, 0}, {7, 3, 0}, {0, 3, 3}, {3, 0, 0}, {5, 4, 0}, {4, 7, 2}, {6, 2, 0}, {6, 6, 0}, {7, 0, 0}, 
{2, 0, 2}, {3, 1, 0}, {0, 4, 2}, {1, 5, 0}, {5, 3, 0}, {2, 1, 2}, {3, 2, 0}, {5, 7, 0}, {6, 1, 0}, {6, 5, 0},   // +143.167s 143168
{7, 1, 0}, {1, 0, 4}, {6, 0, 0}, {7, 5, 0}, {0, 5, 2}, {3, 3, 0}, {5, 2, 0}, {6, 5, 0}, {7, 3, 0}, {1, 1, 4}, 
{2, 2, 0}, {3, 4, 0}, {6, 3, 0}, {6, 4, 0}, {0, 6, 2}, {2, 3, 0}, {6, 6, 0}, {7, 0, 0}, {7, 4, 0}, {7, 6, 0}, 
{1, 2, 2}, {1, 6, 0}, {5, 1, 0}, {2, 0, 3}, {3, 5, 0}, {6, 7, 0}, {0, 6, 2}, {4, 6, 0}, {6, 2, 0}, {1, 3, 2}, 
{3, 6, 0}, {5, 0, 0}, {7, 1, 0}, {6, 5, 2}, {0, 5, 2}, {1, 4, 0}, {2, 1, 0}, {6, 1, 0}, {3, 7, 2}, {5, 6, 0}, 
{7, 3, 0}, {0, 4, 2}, {1, 5, 2}, {4, 0, 0}, {5, 5, 0}, {6, 0, 0}, {6, 6, 0}, {7, 0, 0}, {7, 2, 0}, {7, 5, 2},   // +143.700s 143696
{0, 3, 2}, {1, 0, 2}, {2, 2, 0}, {2, 3, 0}, {4, 1, 0}, {5, 4, 0}, {6, 3, 0}, {6, 4, 0}, {6, 7, 0}, {7, 1, 0}, 
{7, 4, 0}, {7, 6, 0}, {0, 2, 3}, {5, 3, 2}, {7, 3, 0}, {0, 1, 2}, {1, 1, 0}, {2, 0, 0}, {4, 2, 0}, {6, 2, 0}, 
{6, 7, 0}, {4, 6, 2}, {4, 7, 0}, {4, 3, 2}, {5, 2, 0}, {6, 1, 0}, {6, 6, 0}, {0, 0, 2}, {1, 2, 0}, {2, 1, 0}/*, 
{7, 0, 0}, {5, 1, 2}, {2, 4, 2}, {6, 0, 0}, {6, 5, 0}, {7, 5, 0}, {0, 6, 2}, {1, 3, 0}, {7, 1, 0}, {5, 0, 2}, 
{1, 7, 2}, {2, 5, 0}, {6, 3, 0}, {6, 4, 0}, {6, 7, 0}, {0, 5, 2}, {1, 4, 0}, {2, 2, 0}, {2, 3, 0}, {7, 3, 0},   // +144.167s 144160
{5, 6, 3}, {7, 4, 0}, {7, 6, 0}, {2, 0, 2}, {2, 6, 0}, {6, 2, 0}, {6, 6, 0}, {7, 0, 2}, {0, 4, 2}, {1, 5, 0}, 
{4, 6, 0}, {2, 1, 2}, {5, 5, 0}, {7, 1, 0}, {4, 4, 2}, {6, 1, 0}, {6, 5, 0}, {0, 3, 2}, {2, 7, 0}, {1, 0, 2}, 
{5, 4, 0}, {7, 3, 0}, {4, 5, 2}, {6, 0, 0}, {6, 5, 0}, {7, 5, 0}, {0, 2, 4}, {1, 1, 0}, {5, 3, 0}, {6, 6, 0}, 
{2, 3, 2}, {6, 3, 0}, {6, 4, 0}, {7, 0, 0}, {1, 2, 2}, {2, 2, 0}, {3, 0, 0}, {7, 4, 0}, {7, 6, 0}, {0, 1, 3}, 
{2, 0, 0}, {5, 7, 0}, {6, 2, 0}, {6, 7, 0}, {1, 3, 2}, {5, 2, 0}, {7, 1, 0}, {4, 7, 2}, {6, 5, 0}, {2, 1, 2},   // +144.733s 144736
{3, 1, 0}, {6, 1, 0}, {0, 0, 2}, {1, 4, 0}, {5, 1, 0}, {7, 3, 0}, {6, 0, 2}, {7, 5, 0}, {3, 2, 2}, {6, 6, 0}, 
{7, 0, 0}, {0, 0, 2}, {1, 5, 0}, {1, 6, 0}, {5, 0, 2}, {6, 3, 0}, {6, 4, 0}, {0, 1, 2}, {1, 0, 0}, {6, 7, 0}, 
{7, 1, 0}, {3, 3, 2}, {0, 2, 2}, {2, 2, 0}, {2, 3, 0}, {5, 6, 0}, {7, 4, 0}, {7, 6, 0}, {1, 1, 3}, {2, 0, 0}, 
{6, 2, 0}, {6, 7, 0}, {7, 3, 0}, {3, 4, 2}, {5, 5, 0}, {0, 3, 2}, {1, 2, 2}, {4, 6, 0}, {4, 7, 0}, {6, 1, 0}, 
{6, 6, 0}, {7, 0, 0}, {7, 2, 0}, {2, 1, 2}, {5, 4, 0}, {0, 4, 2}, {3, 5, 0}, {6, 5, 0}, {1, 3, 2}, {0, 5, 2},   // +145.267s 145264
{5, 3, 0}, {6, 0, 0}, {7, 1, 0}, {7, 5, 0}, {3, 6, 2}, {6, 7, 0}, {5, 2, 2}, {0, 6, 2}, {1, 4, 0}, {6, 3, 0}, 
{6, 4, 0}, {7, 3, 0}, {2, 2, 2}, {2, 3, 0}, {5, 1, 0}, {7, 4, 0}, {7, 6, 0}, {2, 0, 3}, {3, 7, 0}, {6, 2, 0}, 
{6, 6, 0}, {7, 0, 0}, {0, 0, 2}, {1, 5, 2}, {2, 1, 0}, {5, 0, 0}, {4, 0, 2}, {4, 7, 0}, {6, 1, 0}, {6, 5, 0}, 
{7, 1, 0}, {5, 6, 2}, {0, 1, 2}, {1, 0, 0}, {1, 7, 0}, {6, 0, 0}, {7, 5, 0}, {6, 5, 2}, {7, 3, 0}, {4, 1, 2}, 
{0, 2, 2}, {1, 1, 0}, {5, 5, 0}, {6, 3, 0}, {6, 4, 0}, {6, 6, 0}, {7, 0, 2}, {1, 2, 2}, {4, 2, 0}, {2, 0, 2},   // +145.800s 145792
{2, 2, 0}, {6, 2, 0}, {6, 7, 0}, {0, 3, 3}, {2, 3, 0}, {5, 4, 0}, {7, 1, 0}, {7, 4, 0}, {7, 6, 0}, {1, 3, 2}, 
{2, 7, 0}, {4, 3, 2}, {6, 5, 0}, {0, 4, 2}, {1, 4, 0}, {2, 1, 0}, {4, 6, 0}, {5, 3, 0}, {6, 1, 0}, {7, 3, 0}, 
{2, 4, 4}, {6, 6, 0}, {0, 5, 2}, {1, 5, 0}, {6, 0, 0}, {7, 0, 0}, {7, 5, 0}, {2, 5, 2}, {5, 2, 0}, {1, 0, 2}, 
{5, 7, 0}, {7, 1, 0}, {6, 3, 2}, {6, 4, 0}, {6, 7, 0}, {0, 6, 2}, {2, 6, 0}, {5, 1, 0}, {1, 1, 2}, {7, 3, 0}, 
{2, 0, 3}, {2, 2, 0}, {2, 3, 0}, {4, 4, 0}, {6, 2, 0}, {6, 7, 0}, {7, 4, 0}, {7, 6, 0}, {0, 6, 2}, {2, 1, 2},   // +146.300s 146304
{5, 0, 0}, {6, 1, 0}, {6, 6, 0}, {0, 5, 2}, {1, 2, 0}, {4, 5, 0}, {7, 0, 0}, {1, 6, 2}, {4, 6, 0}, {4, 7, 0}, 
{3, 0, 2}, {5, 6, 0}, {6, 0, 0}, {6, 5, 0}, {7, 5, 0}, {0, 4, 2}, {1, 3, 0}, {7, 1, 0}, {3, 1, 2}, {0, 3, 2}, 
{5, 5, 0}, {6, 3, 0}, {6, 4, 0}, {6, 7, 0}, {1, 4, 2}, {5, 4, 0}, {7, 3, 0}, {3, 2, 2}, {0, 2, 2}, {2, 0, 0}, 
{6, 2, 0}, {6, 6, 0}, {2, 2, 3}, {2, 3, 0}, {5, 3, 0}, {7, 0, 0}, {7, 2, 0}, {0, 1, 2}, {1, 5, 0}, {3, 3, 0}, 
{7, 4, 0}, {7, 6, 0}, {2, 1, 2}, {5, 2, 0}, {6, 1, 0}, {6, 5, 0}, {7, 1, 0}, {3, 4, 2}, {4, 6, 0}, {0, 0, 2},   // +146.767s 146768
{1, 0, 2}, {5, 1, 0}, {6, 5, 0}, {7, 3, 0}, {3, 5, 2}, {6, 0, 0}, {7, 5, 0}, {0, 6, 2}, {5, 0, 0}, {1, 1, 2}, 
{3, 6, 0}, {6, 6, 0}, {7, 0, 0}, {6, 3, 2}, {6, 4, 0}, {0, 5, 2}, {1, 2, 0}, {5, 6, 0}, {6, 7, 0}, {2, 0, 2}, 
{3, 7, 0}, {6, 2, 0}, {1, 3, 3}, {2, 3, 0}, {7, 1, 0}, {7, 4, 0}, {7, 6, 0}, {0, 4, 2}, {2, 2, 0}, {4, 0, 0}, 
{6, 5, 0}, {1, 4, 2}, {1, 7, 0}, {2, 1, 0}, {5, 5, 0}, {6, 1, 0}, {7, 3, 2}, {4, 1, 2}, {4, 7, 0}, {6, 0, 0}, 
{7, 5, 0}, {0, 3, 2}, {1, 5, 0}, {5, 4, 0}, {6, 6, 0}, {7, 0, 0}, {4, 2, 2}, {6, 3, 2}, {6, 4, 0}, {0, 2, 2},   // +147.300s 147296
{1, 0, 0}, {6, 7, 0}, {7, 1, 0}, {2, 7, 2}, {4, 3, 0}, {5, 3, 0}, {1, 1, 4}, {2, 0, 0}, {6, 2, 0}, {6, 7, 0}, 
{7, 3, 0}, {0, 1, 3}, {2, 2, 0}, {2, 4, 0}, {5, 2, 0}, {2, 3, 2}, {7, 4, 0}, {7, 6, 0}, {1, 2, 2}, {2, 1, 0}, 
{6, 1, 0}, {6, 6, 0}, {7, 0, 0}, {0, 0, 2}, {5, 1, 0}, {2, 5, 2}, {4, 6, 0}, {4, 7, 0}, {6, 5, 0}, {1, 3, 2}, 
{5, 7, 0}, {0, 0, 2}, {6, 0, 0}, {7, 1, 0}, {7, 5, 0}, {2, 6, 2}, {5, 0, 0}, {6, 7, 0}, {6, 4, 2}, {0, 1, 2}, 
{1, 4, 0}, {6, 3, 0}, {7, 3, 0}, {5, 6, 2}, {6, 6, 0}, {2, 0, 2}, {4, 4, 0}, {6, 2, 0}, {7, 0, 0}, {0, 2, 3},   // +147.833s 147840
{1, 6, 0}, {5, 5, 0}, {1, 5, 2}, {2, 1, 0}, {2, 2, 0}, {2, 3, 0}, {6, 1, 0}, {7, 1, 0}, {7, 4, 0}, {7, 6, 0}, 
{0, 3, 2}, {4, 5, 0}, {6, 5, 0}, {5, 4, 2}, {0, 4, 2}, {1, 0, 0}, {4, 7, 0}, {6, 0, 0}, {7, 3, 0}, {7, 5, 0}, 
{5, 3, 2}, {6, 5, 0}, {3, 0, 2}, {0, 5, 2}, {1, 1, 0}, {6, 3, 0}, {6, 4, 0}, {6, 6, 0}, {7, 2, 0}, {5, 2, 2}, 
{7, 0, 0}, {0, 6, 2}, {1, 2, 0}, {3, 1, 0}, {2, 0, 2}, {5, 1, 0}, {6, 2, 0}, {6, 7, 0}, {1, 3, 2}, {7, 1, 0}, 
{0, 0, 3}, {2, 1, 2}, {2, 2, 0}, {2, 3, 0}, {3, 2, 0}, {5, 0, 0}, {6, 5, 0}, {1, 4, 2}, {6, 1, 0}, {7, 3, 0},   // +148.300s 148304
{7, 4, 0}, {7, 6, 0}, {0, 1, 4}, {3, 3, 0}, {5, 6, 0}, {6, 6, 0}, {1, 5, 2}, {4, 6, 0}, {6, 0, 0}, {7, 0, 0}, 
{7, 5, 0}, {0, 2, 4}, {1, 0, 0}, {3, 4, 0}, {5, 5, 0}, {7, 1, 0}, {6, 3, 2}, {6, 4, 0}, {6, 7, 0}, {1, 1, 4}, 
{1, 7, 0}, {5, 4, 0}, {7, 3, 0}, {0, 3, 2}, {2, 0, 0}, {3, 5, 0}, {6, 2, 0}, {6, 7, 0}, {2, 1, 5}, {2, 3, 0}, 
{6, 1, 0}, {6, 6, 0}, {0, 4, 2}, {1, 2, 0}, {2, 2, 0}, {3, 6, 0}, {5, 3, 0}, {7, 0, 0}, {7, 4, 0}, {7, 6, 0}, 
{6, 0, 2}, {6, 5, 0}, {7, 5, 2}, {1, 3, 2}, {2, 7, 0}, {4, 6, 0}, {4, 7, 0}, {5, 2, 0}, {7, 1, 0}, {0, 5, 2},   // +148.833s 148832
{3, 7, 0}, {6, 3, 0}, {6, 4, 0}, {6, 7, 0}, {1, 4, 4}, {7, 3, 0}, {0, 6, 2}, {4, 0, 0}, {5, 1, 0}, {2, 0, 2}, 
{6, 2, 0}, {6, 6, 0}, {7, 0, 0}, {0, 6, 5}, {1, 5, 0}, {5, 0, 0}, {4, 1, 2}, {5, 7, 0}, {6, 1, 0}, {6, 5, 0}, 
{7, 1, 0}, {0, 5, 2}, {2, 1, 0}, {2, 2, 0}, {2, 3, 0}, {7, 4, 0}, {7, 6, 0}, {1, 0, 2}, {5, 6, 0}, {4, 2, 2}, 
{6, 5, 0}, {7, 3, 0}, {0, 4, 2}, {4, 6, 0}, {6, 0, 0}, {7, 5, 0}, {1, 1, 2}, {5, 5, 0}, {0, 3, 2}, {6, 6, 0}, 
{7, 0, 0}, {1, 2, 2}, {1, 6, 0}, {4, 3, 0}, {5, 4, 0}, {6, 3, 0}, {6, 4, 0}, {6, 7, 2}, {0, 2, 2}, {2, 0, 0},   // +149.367s 149360
{6, 2, 0}, {1, 3, 2}, {2, 4, 0}, {5, 3, 0}, {7, 1, 0}, {0, 1, 3}, {2, 1, 0}, {6, 5, 0}, {1, 4, 2}, {2, 5, 0}, 
{5, 2, 0}, {6, 1, 0}, {2, 2, 2}, {2, 3, 0}, {7, 3, 0}, {7, 4, 0}, {7, 6, 0}, {0, 0, 2}, {5, 1, 0}, {6, 0, 0}, 
{7, 5, 0}, {1, 5, 2}, {2, 6, 0}, {6, 6, 0}, {7, 0, 0}, {7, 2, 0}, {4, 7, 2}, {0, 6, 2}, {4, 4, 0}, {5, 0, 0}, 
{6, 3, 0}, {6, 4, 0}, {1, 0, 2}, {6, 7, 0}, {7, 1, 0}, {0, 5, 4}, {4, 5, 0}, {5, 6, 0}, {6, 2, 0}, {1, 1, 2}, 
{2, 0, 0}, {6, 7, 0}, {7, 3, 0}, {3, 0, 2}, {5, 5, 3}, {6, 6, 0}, {0, 4, 2}, {1, 2, 0}, {2, 1, 0}, {3, 1, 0},   // +149.867s 149872
{6, 1, 0}, {7, 0, 0}, {2, 2, 2}, {2, 3, 2}, {6, 5, 0}, {7, 4, 0}, {7, 6, 0}, {0, 3, 2}, {1, 3, 0}, {3, 2, 0}, 
{5, 4, 0}, {6, 0, 0}, {7, 1, 0}, {7, 5, 0}, {1, 7, 4}, {4, 6, 0}, {4, 7, 0}, {6, 7, 0}, {0, 2, 2}, {1, 4, 0}, 
{3, 3, 0}, {5, 3, 0}, {6, 4, 0}, {7, 3, 0}, {6, 3, 2}, {2, 0, 2}, {3, 4, 0}, {6, 6, 0}, {6, 2, 2}, {7, 0, 0}, 
{0, 1, 2}, {1, 5, 0}, {5, 2, 0}, {2, 1, 3}, {3, 5, 0}, {6, 1, 0}, {7, 1, 0}, {6, 5, 2}, {0, 0, 2}, {2, 3, 0}, 
{2, 7, 0}, {3, 6, 0}, {5, 1, 0}, {1, 0, 2}, {2, 2, 0}, {6, 0, 0}, {7, 3, 0}, {7, 4, 0}, {7, 5, 0}, {7, 6, 0},   // +150.333s 150336
{3, 7, 2}, {6, 5, 0}, {0, 0, 2}, {1, 1, 2}, {4, 7, 0}, {5, 0, 0}, {6, 3, 0}, {6, 4, 0}, {6, 6, 0}, {4, 0, 2}, 
{7, 0, 0}, {0, 1, 2}, {1, 2, 0}, {2, 0, 2}, {5, 6, 0}, {5, 7, 0}, {6, 2, 0}, {6, 7, 0}, {0, 2, 2}, {1, 3, 0}, 
{4, 1, 0}, {7, 1, 0}, {5, 5, 2}, {2, 1, 3}, {4, 2, 0}, {6, 5, 0}, {0, 3, 2}, {1, 4, 0}, {6, 1, 0}, {7, 3, 0}, 
{5, 4, 2}, {0, 4, 2}, {2, 2, 0}, {2, 3, 0}, {4, 3, 0}, {6, 6, 0}, {1, 5, 2}, {1, 6, 0}, {5, 3, 0}, {6, 0, 0}, 
{7, 0, 0}, {7, 4, 0}, {7, 5, 0}, {7, 6, 0}, {0, 5, 4}, {1, 0, 0}, {2, 4, 0}, {4, 6, 0}, {6, 7, 0}, {7, 1, 0},   // +150.833s 150832
{5, 2, 2}, {6, 3, 0}, {6, 4, 0}, {0, 6, 2}, {1, 1, 2}, {2, 5, 0}, {6, 2, 0}, {6, 7, 0}, {7, 3, 0}, {2, 0, 2}, 
{5, 1, 0}, {0, 0, 3}, {1, 2, 2}, {2, 1, 0}, {5, 0, 0}, {6, 1, 0}, {6, 6, 0}, {7, 0, 0}, {7, 2, 0}, {2, 6, 2}, 
{0, 1, 2}, {6, 0, 0}, {6, 5, 0}, {7, 5, 0}, {1, 3, 2}, {2, 3, 0}, {5, 6, 0}, {7, 4, 0}, {7, 6, 0}, {2, 2, 2}, 
{4, 4, 0}, {7, 1, 0}, {6, 3, 2}, {6, 4, 0}, {6, 7, 0}, {0, 2, 2}, {1, 4, 2}, {4, 6, 0}, {4, 7, 0}, {5, 5, 0}, 
{7, 3, 0}, {2, 0, 2}, {4, 5, 0}, {0, 3, 2}, {6, 2, 0}, {6, 6, 0}, {7, 0, 0}, {1, 5, 2}, {5, 4, 0}, {3, 0, 3},   // +151.400s 151408
{2, 1, 2}, {6, 1, 0}, {6, 5, 0}, {7, 1, 0}, {0, 4, 2}, {1, 0, 2}, {1, 7, 0}, {5, 3, 0}, {2, 2, 2}, {3, 1, 0}, 
{6, 5, 0}, {7, 3, 0}, {0, 5, 2}, {2, 3, 0}, {6, 0, 0}, {7, 4, 0}, {7, 5, 0}, {7, 6, 0}, {1, 1, 2}, {5, 2, 0}, 
{3, 2, 2}, {6, 3, 0}, {6, 4, 0}, {6, 6, 0}, {7, 0, 0}, {1, 2, 2}, {4, 6, 0}, {0, 6, 2}, {5, 1, 0}, {6, 7, 0}, 
{2, 0, 2}, {6, 2, 0}, {1, 3, 2}, {2, 7, 0}, {3, 3, 0}, {7, 1, 0}, {0, 6, 3}, {2, 1, 0}, {6, 1, 0}, {6, 5, 0}, 
{1, 4, 2}, {5, 0, 0}, {7, 3, 0}, {0, 5, 2}, {3, 4, 0}, {6, 0, 2}, {6, 6, 0}, {7, 5, 0}, {0, 4, 2}, {1, 5, 0},   // +151.933s 151936
{5, 6, 0}, {7, 0, 0}, {2, 2, 2}, {2, 3, 0}, {7, 4, 0}, {7, 6, 0}, {1, 0, 2}, {3, 5, 0}, {5, 7, 0}, {6, 3, 0}, 
{6, 4, 0}, {7, 1, 0}, {0, 3, 2}, {5, 5, 0}, {6, 7, 0}, {4, 7, 2}, {0, 2, 2}, {1, 1, 0}, {3, 6, 0}, {5, 4, 0}, 
{6, 2, 0}, {7, 3, 0}, {2, 0, 2}, {6, 7, 0}, {0, 1, 4}, {5, 3, 0}, {6, 6, 0}, {1, 2, 3}, {2, 1, 0}, {3, 7, 0}, 
{6, 1, 0}, {7, 0, 0}, {0, 0, 2}, {1, 6, 0}, {5, 2, 0}, {6, 5, 2}, {1, 3, 2}, {4, 0, 0}, {6, 0, 0}, {7, 1, 0}, 
{7, 5, 0}, {0, 6, 2}, {2, 2, 0}, {2, 3, 0}, {5, 1, 0}, {6, 7, 2}, {7, 4, 0}, {7, 6, 0}, {1, 4, 2}, {4, 1, 0},   // +152.433s 152432
{6, 4, 0}, {7, 3, 0}, {5, 0, 2}, {6, 3, 0}, {0, 5, 2}, {2, 0, 0}, {4, 6, 0}, {4, 7, 0}, {6, 6, 0}, {5, 6, 2}, 
{6, 2, 0}, {7, 0, 0}, {7, 2, 0}, {1, 5, 2}, {4, 2, 0}, {0, 4, 2}, {2, 1, 0}, {6, 1, 0}, {7, 1, 0}, {6, 5, 3}, 
{4, 3, 2}, {5, 5, 0}, {6, 0, 0}, {1, 0, 2}, {6, 5, 0}, {7, 3, 0}, {7, 5, 0}, {0, 3, 2}, {2, 4, 2}, {5, 4, 0}, 
{6, 3, 0}, {6, 4, 0}, {7, 4, 0}, {7, 6, 0}, {1, 1, 2}, {2, 2, 0}, {2, 3, 0}, {6, 6, 0}, {7, 0, 0}, {0, 2, 2}, 
{1, 2, 2}, {2, 5, 0}, {6, 7, 0}, {2, 0, 2}, {4, 7, 0}, {5, 3, 0}, {6, 2, 0}, {0, 1, 2}, {1, 3, 0}, {7, 1, 0},   // +152.933s 152928
{2, 6, 2}, {6, 5, 0}, {1, 4, 3}, {1, 7, 0}, {2, 1, 0}, {5, 2, 0}, {6, 1, 0}, {4, 4, 2}, {7, 3, 0}, {0, 0, 2}, 
{1, 5, 2}, {6, 6, 0}, {7, 0, 0}, {4, 5, 2}, {5, 1, 0}, {6, 0, 0}, {7, 5, 0}, {0, 0, 2}, {1, 0, 2}, {2, 2, 0}, 
{2, 3, 0}, {3, 0, 0}, {6, 7, 0}, {7, 1, 0}, {7, 4, 0}, {7, 6, 0}, {0, 1, 2}, {2, 7, 0}, {6, 3, 0}, {6, 4, 0}, 
{3, 1, 2}, {5, 0, 0}, {1, 1, 2}, {2, 0, 0}, {4, 6, 0}, {6, 2, 0}, {6, 7, 0}, {7, 3, 0}, {0, 2, 2}, {3, 2, 2}, 
{5, 6, 0}, {0, 3, 2}, {1, 2, 0}, {2, 1, 0}, {6, 1, 0}, {6, 6, 0}, {7, 0, 0}, {3, 3, 3}, {5, 5, 0}, {6, 0, 2},   // +153.467s 153472
{6, 5, 0}, {7, 5, 0}, {0, 4, 2}, {1, 3, 0}, {5, 4, 0}, {5, 7, 0}, {3, 4, 2}, {7, 1, 0}, {0, 5, 2}, {6, 3, 0}, 
{6, 4, 0}, {6, 7, 0}, {2, 2, 2}, {2, 3, 0}, {5, 3, 0}, {7, 4, 0}, {7, 6, 0}, {1, 4, 2}, {3, 5, 0}, {7, 3, 0}, 
{0, 6, 2}, {2, 0, 0}, {5, 2, 0}, {6, 2, 0}, {6, 6, 0}, {3, 6, 2}, {7, 0, 0}, {1, 6, 2}, {4, 6, 0}, {4, 7, 0}, 
{0, 0, 2}, {1, 5, 0}, {5, 1, 0}, {7, 1, 0}, {2, 1, 2}, {3, 7, 0}, {6, 1, 0}, {6, 5, 0}, {5, 0, 3}, {0, 1, 2}, 
{1, 0, 0}, {4, 0, 0}, {7, 3, 0}, {6, 0, 2}, {6, 5, 0}, {7, 5, 0}, {5, 6, 2}, {0, 2, 2}, {1, 1, 0}, {4, 1, 0},   // +153.967s 153968
{6, 6, 0}, {7, 2, 0}, {2, 2, 2}, {6, 3, 0}, {6, 4, 0}, {7, 0, 0}, {1, 2, 2}, {2, 3, 0}, {7, 4, 0}, {7, 6, 0}, 
{4, 2, 2}, {5, 5, 0}, {6, 7, 0}, {0, 3, 2}, {2, 0, 0}, {6, 2, 0}, {7, 1, 0}, {1, 3, 2}, {4, 6, 0}, {2, 1, 2}, 
{4, 3, 0}, {5, 4, 0}, {6, 5, 0}, {0, 4, 2}, {1, 4, 0}, {6, 1, 0}, {7, 3, 0}, {6, 6, 5}, {7, 5, 0}, {1, 5, 2}, 
{2, 4, 0}, {6, 0, 0}, {7, 0, 0}, {0, 5, 2}, {5, 3, 0}, {1, 0, 2}, {6, 4, 0}, {2, 5, 2}, {6, 7, 0}, {7, 1, 0}, 
{0, 6, 2}, {2, 2, 0}, {2, 3, 0}, {6, 3, 0}, {7, 4, 0}, {7, 6, 0}, {1, 1, 2}, {1, 7, 0}, {2, 0, 0}, {5, 2, 0},   // +154.467s 154464
{6, 7, 2}, {7, 3, 0}, {0, 6, 2}, {2, 6, 0}, {4, 7, 0}, {6, 2, 0}, {5, 1, 2}, {6, 6, 0}, {1, 2, 2}, {2, 1, 0}, 
{7, 0, 0}, {0, 5, 3}, {6, 1, 0}, {6, 5, 0}, {4, 4, 2}, {5, 0, 2}, {0, 4, 2}, {1, 3, 0}, {2, 7, 0}, {6, 7, 0}, 
{7, 1, 0}, {7, 5, 0}, {4, 5, 2}, {6, 0, 0}, {0, 3, 2}, {5, 6, 0}, {1, 4, 2}, {2, 2, 0}, {6, 4, 0}, {7, 3, 0}, 
{7, 4, 0}, {7, 6, 0}, {0, 2, 2}, {2, 3, 0}, {5, 5, 0}, {6, 3, 0}, {6, 6, 0}, {2, 0, 2}, {3, 0, 0}, {7, 0, 0}, 
{5, 4, 2}, {6, 2, 0}, {0, 1, 2}, {1, 5, 0}, {4, 6, 0}, {4, 7, 0}, {5, 7, 0}, {6, 5, 0}, {2, 1, 3}, {3, 1, 0},   // +155.000s 155008
{6, 1, 0}, {7, 1, 0}, {0, 0, 2}, {5, 3, 0}, {1, 0, 2}, {6, 5, 0}, {7, 5, 0}, {6, 0, 2}, {7, 3, 0}, {3, 2, 2}, 
{5, 2, 0}, {0, 6, 2}, {1, 1, 0}, {6, 4, 0}, {6, 6, 0}, {5, 1, 2}, {6, 3, 0}, {1, 6, 2}, {2, 3, 0}, {7, 0, 0}, 
{7, 4, 0}, {7, 6, 0}, {1, 2, 2}, {2, 0, 0}, {2, 2, 0}, {3, 3, 0}, {6, 7, 0}, {0, 5, 2}, {5, 0, 0}, {6, 2, 2}, 
{7, 1, 0}, {1, 3, 2}, {3, 4, 0}, {4, 7, 0}, {6, 5, 0}, {0, 4, 2}, {2, 1, 0}, {5, 6, 0}, {1, 4, 3}, {6, 1, 0}, 
{7, 3, 0}, {6, 6, 2}, {7, 2, 0}, {3, 5, 2}, {7, 5, 0}, {0, 3, 2}, {1, 5, 0}, {5, 5, 0}, {7, 0, 0}, {6, 0, 2},   // +155.567s 155568
{1, 0, 2}, {3, 6, 0}, {6, 7, 0}, {0, 2, 2}, {5, 4, 0}, {6, 4, 0}, {7, 1, 0}, {2, 2, 2}, {2, 3, 0}, {6, 3, 0}, 
{6, 7, 0}, {7, 4, 0}, {7, 6, 0}, {1, 1, 2}, {2, 0, 0}, {3, 7, 2}, {6, 2, 0}, {7, 3, 0}, {0, 1, 2}, {4, 6, 0}, 
{5, 3, 0}, {6, 6, 0}, {2, 1, 2}, {1, 2, 3}, {6, 1, 0}, {6, 5, 0}, {7, 0, 0}, {4, 0, 2}, {7, 5, 0}, {0, 0, 2}, 
{5, 2, 0}, {6, 0, 0}, {6, 7, 2}, {1, 3, 2}, {6, 4, 0}, {7, 1, 0}, {0, 0, 2}, {1, 7, 0}, {4, 1, 0}, {6, 3, 0}, 
{5, 1, 2}, {1, 4, 2}, {2, 3, 0}, {6, 6, 0}, {7, 3, 0}, {7, 4, 0}, {7, 6, 0}, {0, 1, 2}, {2, 0, 0}, {2, 2, 0},   // +156.100s 156096
{4, 2, 2}, {6, 2, 0}, {0, 2, 2}, {1, 5, 0}, {5, 0, 0}, {6, 5, 0}, {7, 0, 0}, {2, 1, 3}, {4, 6, 0}, {4, 7, 0}, 
{2, 7, 2}, {6, 1, 0}, {0, 3, 2}, {4, 3, 0}, {5, 6, 0}, {7, 1, 0}, {1, 0, 2}, {6, 5, 0}, {5, 5, 2}, {7, 5, 0}, 
{0, 4, 2}, {2, 4, 0}, {6, 0, 0}, {6, 6, 0}, {7, 3, 0}, {5, 4, 2}, {0, 5, 2}, {1, 1, 0}, {6, 4, 0}, {2, 2, 2}, 
{2, 5, 0}, {6, 3, 0}, {6, 7, 0}, {1, 2, 2}, {2, 3, 0}, {5, 3, 0}, {5, 7, 0}, {7, 0, 0}, {7, 4, 0}, {7, 6, 0}, 
{0, 6, 2}, {2, 0, 0}, {2, 6, 2}, {5, 2, 0}, {6, 2, 0}, {6, 5, 0}, {1, 3, 2}, {2, 1, 0}, {4, 6, 0}, {7, 1, 0},   // +156.600s 156592
{0, 0, 3}, {6, 1, 0}, {1, 4, 2}, {4, 4, 0}, {5, 1, 0}, {6, 6, 0}, {6, 0, 2}, {7, 3, 0}, {7, 5, 0}, {1, 6, 2}, 
{5, 0, 0}, {0, 1, 2}, {1, 5, 0}, {4, 5, 0}, {6, 7, 0}, {7, 0, 0}, {6, 3, 2}, {6, 4, 0}, {1, 0, 2}, {3, 0, 0}, 
{5, 6, 0}, {0, 2, 2}, {6, 7, 0}, {7, 1, 0}, {2, 0, 2}, {2, 2, 0}, {2, 3, 0}, {7, 4, 0}, {7, 6, 0}, {1, 1, 2}, 
{3, 1, 0}, {5, 5, 0}, {6, 2, 0}, {0, 3, 2}, {6, 6, 0}, {7, 2, 0}, {7, 3, 0}, {4, 7, 3}, {1, 2, 2}, {2, 1, 0}, 
{3, 2, 0}, {6, 1, 0}, {6, 5, 0}, {5, 4, 2}, {7, 0, 0}, {0, 4, 2}, {3, 3, 2}, {6, 7, 0}, {7, 5, 0}, {1, 3, 2},   // +157.167s 157168
{6, 0, 0}, {0, 5, 2}, {5, 3, 0}, {7, 1, 0}, {3, 4, 2}, {6, 4, 0}, {1, 4, 2}, {6, 3, 0}, {6, 6, 0}, {2, 0, 2}, 
{2, 2, 0}, {2, 3, 0}, {3, 5, 0}, {5, 2, 0}, {7, 3, 0}, {0, 6, 2}, {6, 2, 0}, {7, 4, 0}, {7, 6, 0}, {2, 1, 2}, 
{6, 5, 0}, {1, 5, 2}, {3, 6, 0}, {7, 0, 0}, {0, 6, 3}, {4, 6, 0}, {4, 7, 0}, {5, 1, 0}, {6, 1, 0}, {1, 7, 2}, 
{6, 5, 0}, {3, 7, 2}, {7, 1, 0}, {7, 5, 0}, {0, 5, 2}, {1, 0, 0}, {5, 0, 0}, {6, 0, 0}, {4, 0, 2}, {6, 6, 0}, 
{0, 4, 2}, {6, 4, 0}, {7, 3, 0}, {1, 1, 2}, {6, 3, 0}, {6, 7, 0}, {4, 1, 2}, {5, 6, 0}, {0, 3, 2}, {2, 0, 0},   // +157.700s 157696
{2, 3, 0}, {2, 7, 0}, {7, 0, 0}, {1, 2, 2}, {2, 2, 0}, {5, 5, 0}, {6, 2, 0}, {6, 5, 0}, {7, 4, 0}, {7, 6, 0}, 
{4, 2, 2}, {0, 2, 2}, {7, 1, 0}, {1, 3, 3}, {2, 1, 0}, {4, 7, 0}, {5, 4, 0}, {6, 6, 0}, {0, 1, 2}, {4, 3, 0}, 
{6, 1, 0}, {1, 4, 2}, {5, 3, 0}, {7, 3, 0}, {0, 0, 2}, {5, 7, 0}, {7, 5, 0}, {6, 7, 2}, {1, 5, 2}, {2, 4, 0}, 
{5, 2, 0}, {7, 0, 0}, {6, 0, 2}, {6, 4, 0}, {0, 6, 2}, {1, 0, 0}, {6, 7, 0}, {7, 1, 0}, {5, 1, 2}, {2, 0, 2}, 
{2, 2, 0}, {2, 3, 0}, {2, 5, 0}, {6, 3, 0}, {6, 6, 0}, {7, 4, 0}, {7, 6, 0}, {1, 1, 2}, {0, 5, 3}, {1, 6, 0},   // +158.200s 158208
{2, 1, 0}, {5, 0, 0}, {6, 2, 0}, {7, 3, 0}, {2, 6, 2}, {6, 5, 0}, {4, 6, 2}, {7, 5, 0}, {0, 4, 2}, {1, 2, 0}, 
{6, 1, 0}, {5, 6, 2}, {6, 7, 0}, {7, 0, 0}, {4, 4, 2}, {6, 0, 0}, {6, 4, 0}, {1, 3, 2}, {0, 3, 2}, {5, 5, 0}, 
{4, 5, 2}, {6, 3, 0}, {6, 6, 0}, {7, 1, 0}, {7, 2, 0}, {2, 0, 2}, {0, 2, 2}, {1, 4, 0}, {2, 2, 0}, {2, 3, 0}, 
{7, 4, 0}, {7, 6, 0}, {5, 4, 2}, {6, 5, 0}, {7, 3, 0}, {2, 1, 2}, {3, 0, 0}, {6, 2, 0}, {1, 5, 3}, {0, 1, 2}, 
{4, 6, 0}, {4, 7, 0}, {5, 3, 0}, {6, 5, 0}, {7, 0, 0}, {6, 1, 2}, {3, 1, 2}, {6, 6, 0}, {7, 1, 0}, {7, 5, 0},   // +158.733s 158736
{0, 0, 2}, {1, 0, 0}, {5, 2, 0}, {6, 0, 2}, {6, 4, 0}, {3, 2, 2}, {6, 7, 0}, {7, 3, 0}, {0, 0, 2}, {1, 1, 0}, 
{2, 0, 2}, {5, 1, 0}, {0, 1, 2}, {1, 2, 0}, {2, 2, 0}, {6, 3, 0}, {6, 5, 0}, {1, 7, 2}, {2, 3, 0}, {3, 3, 0}, 
{7, 0, 0}, {7, 4, 0}, {7, 6, 0}, {1, 3, 3}, {2, 1, 0}, {6, 2, 0}, {0, 2, 2}, {5, 0, 0}, {6, 6, 0}, {4, 6, 2}, 
{7, 1, 0}, {7, 5, 0}, {0, 3, 2}, {1, 4, 0}, {3, 4, 0}, {6, 1, 0}, {5, 6, 2}, {1, 5, 2}, {6, 4, 0}, {6, 7, 0}, 
{7, 3, 0}, {0, 4, 2}, {2, 7, 0}, {6, 0, 0}, {3, 5, 2}, {5, 5, 0}, {1, 0, 2}, {2, 0, 0}, {6, 7, 0}, {7, 0, 0},   // +159.267s 159264
{0, 5, 2}, {5, 4, 0}, {6, 3, 0}, {3, 6, 2}, {6, 6, 0}, {0, 6, 2}, {1, 1, 0}, {2, 2, 0}, {2, 3, 0}, {7, 1, 0}, 
{7, 4, 0}, {7, 6, 0}, {2, 1, 2}, {5, 3, 0}, {5, 7, 3}, {6, 2, 0}, {6, 5, 0}, {0, 0, 2}, {3, 7, 0}, {4, 7, 0}, 
{5, 2, 0}, {7, 3, 0}, {1, 2, 2}, {6, 1, 2}, {6, 7, 0}, {7, 5, 0}, {5, 1, 2}, {7, 0, 0}, {0, 1, 2}, {1, 3, 0}, 
{4, 0, 0}, {5, 0, 2}, {6, 4, 0}, {6, 6, 0}, {1, 6, 2}, {6, 0, 0}, {1, 4, 2}, {2, 0, 0}, {4, 1, 0}, {7, 1, 0}, 
{0, 2, 2}, {5, 6, 0}, {6, 3, 0}, {6, 5, 0}, {2, 2, 2}, {2, 3, 0}, {7, 3, 0}, {2, 1, 3}, {7, 4, 0}, {7, 6, 0},   // +159.800s 159808
{0, 3, 2}, {1, 5, 0}, {4, 2, 0}, {5, 5, 0}, {6, 2, 0}, {6, 5, 0}, {7, 0, 2}, {7, 5, 0}, {4, 6, 2}, {4, 7, 0}, 
{6, 1, 0}, {7, 2, 0}, {0, 4, 2}, {4, 3, 0}, {6, 6, 0}, {7, 1, 0}, {1, 0, 2}, {5, 4, 0}, {6, 4, 0}, {6, 0, 2}, 
{6, 7, 0}, {1, 1, 2}, {2, 4, 0}, {7, 3, 0}, {0, 5, 2}, {5, 3, 0}, {1, 2, 2}, {2, 0, 0}, {2, 5, 0}, {6, 3, 0}, 
{6, 5, 0}, {0, 6, 4}, {2, 2, 0}, {2, 3, 0}, {7, 0, 0}, {1, 3, 3}, {2, 1, 0}, {2, 6, 0}, {5, 2, 0}, {6, 2, 0}, 
{7, 4, 0}, {7, 6, 0}, {6, 6, 2}, {0, 6, 2}, {1, 4, 0}, {4, 4, 0}, {7, 1, 0}, {4, 7, 2}, {5, 1, 0}, {6, 1, 2},   // +160.333s 160336
{6, 7, 0}, {7, 5, 0}, {0, 5, 2}, {1, 5, 0}, {4, 5, 0}, {7, 3, 0}, {1, 7, 2}, {0, 4, 2}, {3, 0, 0}, {5, 0, 0}, 
{6, 0, 0}, {6, 4, 0}, {6, 7, 0}, {1, 0, 2}, {7, 0, 0}, {2, 0, 2}, {5, 6, 0}, {0, 3, 2}, {3, 1, 0}, {6, 3, 0}, 
{6, 6, 0}, {7, 1, 0}, {1, 1, 2}, {2, 1, 0}, {2, 2, 0}, {0, 2, 2}, {2, 3, 0}, {3, 2, 0}, {5, 5, 0}, {6, 5, 0}, 
{2, 7, 3}, {6, 2, 0}, {7, 3, 0}, {7, 4, 0}, {7, 6, 0}, {1, 2, 2}, {5, 4, 0}, {7, 5, 0}, {0, 1, 2}, {3, 3, 0}, 
{4, 6, 0}, {6, 1, 0}, {6, 7, 0}, {0, 0, 4}, {1, 3, 0}, {3, 4, 0}, {5, 3, 0}, {6, 0, 0}, {6, 4, 0}, {7, 0, 0},   // +160.767s 160768
{6, 6, 2}, {5, 2, 2}, {0, 6, 2}, {2, 0, 0}, {3, 5, 0}, {6, 3, 0}, {7, 1, 0}, {1, 4, 2}, {5, 7, 0}, {3, 6, 2}, 
{5, 1, 0}, {6, 5, 0}, {0, 5, 2}, {7, 3, 0}, {2, 1, 2}, {2, 2, 0}, {2, 3, 0}, {5, 0, 0}, {6, 2, 0}, {7, 4, 0}, 
{7, 6, 0}, {1, 5, 3}, {3, 7, 0}, {6, 5, 0}, {7, 0, 0}, {0, 4, 4}, {5, 6, 0}, {6, 1, 0}, {6, 6, 0}, {7, 5, 0}, 
{1, 0, 2}, {1, 6, 0}, {4, 0, 0}, {4, 6, 0}, {4, 7, 0}, {7, 1, 0}, {0, 3, 4}, {4, 1, 0}, {6, 4, 0}, {6, 7, 0}, 
{1, 1, 2}, {5, 5, 0}, {6, 0, 0}, {7, 3, 0}, {2, 0, 2}, {1, 2, 2}, {4, 2, 0}, {6, 5, 0}, {0, 2, 2}, {5, 4, 0},   // +161.333s 161328
{6, 3, 0}, {7, 0, 0}, {2, 1, 2}, {4, 3, 0}, {1, 3, 3}, {2, 2, 0}, {2, 3, 0}, {6, 2, 0}, {6, 6, 0}, {7, 2, 0}, 
{0, 1, 2}, {7, 4, 0}, {7, 6, 0}, {1, 4, 2}, {2, 4, 0}, {5, 3, 0}, {7, 1, 0}, {7, 5, 0}, {4, 6, 2}, {6, 1, 0}, 
{6, 7, 2}, {0, 0, 2}, {1, 5, 0}, {5, 2, 0}, {6, 0, 0}, {6, 4, 0}, {7, 3, 0}, {2, 5, 2}, {1, 0, 2}, {6, 7, 0}, 
{7, 0, 0}, {0, 0, 2}, {2, 0, 0}, {5, 1, 0}, {6, 3, 0}, {2, 6, 2}, {6, 6, 0}, {0, 1, 2}, {1, 1, 0}, {7, 1, 0}, 
{2, 1, 2}, {6, 2, 0}, {6, 5, 0}, {2, 3, 2}, {4, 4, 0}, {5, 0, 0}, {7, 3, 0}, {0, 2, 3}, {1, 2, 0}, {2, 2, 0},   // +161.833s 161840
{7, 4, 0}, {7, 6, 0}, {6, 7, 2}, {0, 3, 2}, {1, 7, 0}, {5, 6, 0}, {6, 1, 0}, {7, 5, 0}, {4, 5, 2}, {4, 7, 0}, 
{7, 0, 0}, {0, 4, 2}, {1, 3, 0}, {5, 5, 0}, {0, 0, 2}, {0, 1, 0}, {0, 2, 0}, {0, 3, 0}, {0, 4, 0}, {6, 6, 0}, 
{7, 2, 0}, {1, 0, 2}, {1, 4, 0}, {1, 5, 0}, {2, 3, 0}, {2, 7, 0}, {3, 0, 0}, {3, 1, 0}, {3, 2, 0}, {3, 3, 0}, 
{3, 4, 0}, {3, 5, 0}, {3, 6, 0}, {3, 7, 0}, {4, 0, 0}, {4, 1, 0}, {4, 2, 0}, {4, 3, 0}, {4, 6, 0}, {4, 7, 0}, 
{5, 0, 0}, {5, 1, 0}, {5, 5, 0}, {5, 6, 0}, {6, 0, 0}, {6, 5, 0}, {7, 0, 0}, {7, 4, 0}, {7, 6, 0}, {1, 7, 8},   // +162.167s 162160
{5, 2, 0}, {7, 3, 0}, {7, 4, 0}, {5, 3, 7}, {5, 4, 0}, {4, 6, 2}, {5, 5, 0}, {4, 4, 6}, {5, 6, 0}, {0, 5, 6}, 
{0, 6, 0}, {3, 4, 0}, {4, 5, 0}, {0, 2, 2}, {0, 3, 0}, {0, 4, 0}, {2, 5, 0}, {2, 6, 0}, {3, 3, 0}, {2, 4, 2}, 
{2, 7, 0}, {3, 0, 0}, {3, 5, 0}, {0, 0, 2}, {0, 1, 0}, {3, 2, 0}, {3, 1, 3}, {3, 6, 0}, {6, 5, 0}, {6, 6, 0}, 
{6, 7, 0}, {3, 7, 2}, {4, 7, 0}, {6, 3, 0}, {6, 4, 0}, {2, 0, 2}, {4, 0, 0}, {6, 2, 0}, {7, 6, 0}, {4, 1, 2}, 
{4, 2, 0}, {6, 0, 0}, {6, 1, 0}, {2, 1, 2}, {2, 2, 0}, {2, 3, 0}, {4, 3, 0}, {1, 1, 2}, {1, 2, 0}, {1, 3, 0},   // +162.800s 162800
{1, 6, 0}, {1, 0, 2}, {1, 4, 0}, {1, 5, 0}, {7, 5, 0}, {5, 7, 2}, {7, 2, 0}, {5, 0, 2}, {5, 1, 0}, {7, 0, 0}, 
{7, 1, 0}, {1, 7, 2}, {5, 2, 0}, {7, 3, 0}, {7, 4, 0}, {4, 6, 2}, {5, 3, 0}, {5, 4, 0}, {5, 5, 0}, {0, 6, 2}, 
{4, 4, 0}, {5, 6, 0}, {0, 5, 3}, {3, 4, 0}, {4, 5, 0}, {0, 3, 2}, {0, 4, 0}, {2, 5, 0}, {2, 6, 0}, {3, 3, 0}, 
{0, 2, 2}, {2, 4, 0}, {2, 7, 0}, {0, 1, 2}, {3, 0, 0}, {3, 2, 0}, {3, 5, 0}, {0, 0, 2}, {6, 5, 0}, {6, 6, 0}, 
{6, 7, 0}, {3, 1, 2}, {3, 6, 0}, {4, 7, 0}, {6, 4, 0}, {2, 0, 2}, {3, 7, 0}, {4, 0, 0}, {6, 3, 0}, {7, 6, 0},   // +163.233s 163232
{4, 1, 2}, {6, 1, 0}, {6, 2, 0}, {2, 1, 2}, {2, 2, 0}, {2, 3, 0}, {4, 2, 0}, {4, 3, 0}, {6, 0, 0}, {1, 1, 2}, 
{1, 2, 0}, {1, 3, 0}, {1, 6, 0}, {1, 0, 2}, {1, 4, 0}, {1, 5, 0}, {0, 1, 3}, {0, 2, 0}, {0, 3, 0}, {0, 4, 0}, 
{1, 7, 0}, {7, 5, 0}, {0, 0, 2}, {0, 5, 0}, {0, 6, 0}, {2, 4, 0}, {2, 5, 0}, {2, 6, 2}, {2, 7, 0}, {4, 4, 2}, 
{4, 5, 0}, {4, 7, 0}, {6, 5, 0}, {6, 6, 0}, {6, 7, 0}, {3, 0, 2}, {4, 6, 0}, {5, 5, 0}, {5, 6, 0}, {3, 1, 2}, 
{3, 2, 0}, {3, 3, 0}, {6, 4, 0}, {3, 4, 2}, {5, 4, 0}, {7, 4, 0}, {3, 5, 2}, {5, 3, 0}, {6, 2, 0}, {6, 3, 0},   // +163.633s 163632
{7, 3, 0}, {3, 6, 2}, {5, 2, 0}, {3, 7, 2}, {5, 0, 0}, {5, 1, 0}, {6, 1, 0}, {7, 0, 0}, {7, 6, 0}, {2, 0, 2}, 
{4, 0, 0}, {6, 0, 0}, {7, 1, 0}, {2, 1, 2}, {4, 1, 0}, {4, 2, 0}, {7, 2, 0}, {2, 2, 2}, {2, 3, 0}, {4, 3, 0}, 
{5, 7, 0}, {1, 1, 3}, {1, 2, 0}, {1, 6, 0}, {7, 5, 0}, {1, 0, 2}, {1, 3, 0}, {1, 4, 0}, {1, 5, 0}, {0, 2, 2}, 
{0, 3, 0}, {0, 4, 0}, {0, 0, 2}, {0, 1, 0}, {0, 5, 0}, {0, 6, 0}, {1, 7, 0}, {2, 4, 2}, {2, 5, 0}, {2, 6, 2}, 
{2, 7, 0}, {6, 5, 0}, {6, 6, 0}, {4, 4, 2}, {4, 5, 0}, {4, 7, 0}, {6, 7, 0}, {3, 0, 2}, {4, 6, 0}, {5, 5, 0},   // +164.067s 164064
{5, 6, 0}, {3, 1, 2}, {3, 2, 0}, {3, 3, 0}, {6, 4, 0}, {3, 4, 2}, {5, 4, 0}, {6, 3, 0}, {7, 4, 0}, {3, 5, 2}, 
{5, 3, 0}, {6, 2, 0}, {7, 3, 0}, {3, 6, 3}, {5, 2, 0}, {7, 0, 0}, {7, 6, 0}, {2, 0, 2}, {3, 7, 0}, {5, 0, 0}, 
{5, 1, 0}, {6, 1, 0}, {4, 0, 2}, {6, 0, 0}, {7, 1, 0}, {2, 1, 2}, {4, 1, 0}, {4, 2, 0}, {7, 2, 0}, {1, 6, 2}, 
{2, 2, 0}, {2, 3, 0}, {4, 3, 0}, {5, 7, 0}, {1, 1, 2}, {1, 2, 0}, {1, 3, 0}, {1, 4, 0}, {7, 5, 0}, {1, 4, 2}, 
{7, 5, 0}, {1, 1, 2}, {1, 2, 0}, {1, 3, 0}, {1, 6, 0}, {2, 1, 2}, {2, 2, 0}, {2, 3, 0}, {4, 3, 0}, {4, 1, 2},   // +164.500s 164496
{4, 2, 0}, {6, 0, 0}, {2, 0, 2}, {4, 0, 0}, {6, 1, 0}, {6, 2, 0}, {3, 7, 2}, {4, 7, 0}, {6, 3, 0}, {7, 6, 0}, 
{3, 1, 2}, {3, 6, 0}, {6, 4, 0}, {0, 0, 3}, {0, 1, 0}, {3, 2, 0}, {6, 5, 0}, {6, 6, 0}, {6, 7, 0}, {2, 4, 2}, 
{2, 7, 0}, {3, 0, 0}, {3, 5, 0}, {0, 2, 2}, {0, 3, 0}, {2, 5, 0}, {3, 3, 0}, {0, 4, 2}, {0, 5, 0}, {2, 6, 0}, 
{3, 4, 0}, {0, 6, 2}, {1, 7, 0}, {4, 4, 0}, {4, 5, 0}, {4, 6, 2}, {5, 5, 0}, {5, 6, 0}, {5, 2, 2}, {5, 3, 0}, 
{5, 4, 0}, {5, 1, 2}, {7, 0, 0}, {7, 3, 0}, {7, 4, 0}, {5, 0, 2}, {5, 7, 0}, {7, 1, 0}, {7, 2, 0}, {1, 5, 2},   // +164.933s 164928
{7, 5, 0}, {1, 0, 2}, {1, 3, 0}, {1, 4, 0}, {1, 6, 0}, {1, 1, 7}, {1, 2, 0}, {2, 2, 2}, {2, 3, 0}, {2, 1, 4}, 
{4, 2, 0}, {4, 3, 0}, {6, 0, 0}, {2, 0, 4}, {4, 1, 2}, {6, 1, 0}, {6, 2, 0}, {3, 7, 4}, {4, 0, 0}, {6, 3, 0}, 
{7, 6, 0}, {3, 1, 2}, {3, 6, 0}, {4, 7, 0}, {6, 4, 0}, {0, 0, 2}, {0, 1, 0}, {3, 2, 0}, {6, 5, 0}, {6, 6, 0}, 
{6, 7, 0}, {2, 4, 3}, {2, 7, 0}, {3, 0, 0}, {3, 5, 0}, {0, 2, 2}, {0, 3, 0}, {2, 5, 0}, {3, 3, 0}, {0, 4, 2}, 
{0, 5, 0}, {2, 6, 0}, {3, 4, 0}, {0, 6, 2}, {1, 7, 0}, {4, 4, 0}, {4, 5, 0}, {4, 6, 2}, {5, 5, 0}, {5, 6, 0},   // +165.567s 165568
{5, 2, 2}, {5, 3, 0}, {5, 4, 0}, {5, 1, 2}, {7, 3, 0}, {7, 4, 0}, {1, 0, 2}, {1, 5, 0}, {5, 0, 0}, {5, 7, 0}, 
{7, 0, 0}, {7, 1, 0}, {1, 1, 2}, {1, 2, 0}, {1, 3, 0}, {1, 4, 0}, {7, 2, 0}, {1, 6, 2}, {2, 2, 0}, {2, 3, 0}, 
{7, 5, 0}, {2, 1, 2}, {4, 2, 0}, {4, 3, 0}, {5, 7, 0}, {4, 0, 2}, {4, 1, 0}, {6, 0, 0}, {7, 2, 0}, {2, 0, 3}, 
{5, 0, 0}, {6, 1, 0}, {7, 1, 0}, {3, 7, 2}, {5, 1, 0}, {7, 0, 0}, {7, 6, 0}, {3, 5, 2}, {3, 6, 0}, {5, 2, 0}, 
{6, 2, 0}, {3, 4, 2}, {5, 3, 0}, {6, 3, 0}, {7, 3, 0}, {3, 2, 2}, {3, 3, 0}, {5, 4, 0}, {7, 4, 0}, {3, 0, 2},   // +166.000s 166000
{3, 1, 0}, {5, 5, 0}, {6, 4, 0}, {4, 5, 2}, {4, 6, 0}, {4, 7, 0}, {5, 6, 0}, {4, 4, 2}, {6, 5, 0}, {6, 6, 0}, 
{6, 7, 0}, {0, 0, 2}, {0, 6, 0}, {2, 4, 0}, {2, 5, 0}, {2, 6, 0}, {2, 7, 0}, {0, 1, 2}, {0, 5, 0}, {1, 7, 0}, 
{0, 2, 2}, {0, 3, 0}, {0, 4, 0}, {1, 0, 2}, {1, 3, 0}, {1, 4, 0}, {1, 5, 0}, {1, 1, 3}, {1, 2, 0}, {1, 6, 0}, 
{2, 2, 2}, {2, 3, 0}, {7, 5, 0}, {2, 1, 2}, {4, 2, 0}, {4, 3, 0}, {5, 7, 0}, {4, 1, 2}, {7, 2, 0}, {4, 0, 4}, 
{6, 0, 0}, {2, 0, 2}, {3, 7, 0}, {5, 0, 0}, {6, 1, 0}, {7, 1, 0}, {3, 6, 2}, {5, 1, 0}, {7, 0, 0}, {7, 6, 0},   // +166.467s 166464
{3, 5, 2}, {5, 2, 0}, {6, 2, 0}, {3, 4, 2}, {5, 3, 0}, {5, 4, 0}, {6, 3, 0}, {7, 3, 0}, {3, 1, 2}, {3, 2, 0}, 
{3, 3, 0}, {7, 4, 0}, {3, 0, 3}, {4, 6, 0}, {5, 5, 0}, {5, 6, 0}, {6, 4, 0}, {4, 5, 2}, {4, 7, 0}, {2, 6, 2}, 
{2, 7, 0}, {4, 4, 0}, {6, 5, 0}, {6, 6, 0}, {6, 7, 0}, {0, 0, 2}, {0, 6, 0}, {2, 4, 0}, {2, 5, 0}, {0, 1, 2}, 
{0, 5, 0}, {0, 2, 2}, {0, 3, 0}, {0, 4, 0}, {1, 7, 0}, {6, 5, 0}, {0, 6, 2}, {1, 1, 0}, {1, 6, 0}, {2, 3, 0}, 
{4, 7, 0}, {5, 6, 0}, {6, 3, 0}, {6, 4, 0}, {7, 3, 0}, {7, 4, 0}, {7, 6, 0}, {2, 4, 2}, {0, 5, 2}, {1, 2, 0},   // +166.867s 166864
{6, 2, 0}, {6, 6, 0}, {7, 0, 0}, {2, 0, 2}, {2, 5, 2}, {5, 5, 0}, {6, 7, 0}, {0, 4, 2}, {1, 3, 0}, {2, 1, 0}, 
{6, 1, 0}, {7, 1, 0}, {5, 4, 3}, {1, 4, 2}, {2, 6, 0}, {6, 5, 0}, {7, 2, 0}, {0, 3, 2}, {6, 0, 0}, {7, 5, 0}, 
{1, 5, 2}, {5, 3, 0}, {7, 3, 0}, {4, 4, 2}, {2, 2, 4}, {6, 4, 0}, {2, 3, 2}, {7, 0, 0}, {7, 4, 0}, {7, 6, 0}, 
{0, 2, 6}, {4, 5, 0}, {4, 6, 0}, {5, 2, 0}, {6, 3, 0}, {1, 0, 5}, {0, 1, 2}, {6, 6, 0}, {2, 0, 2}, {6, 2, 0}, 
{7, 1, 0}, {3, 0, 2}, {5, 1, 0}, {1, 1, 6}, {2, 1, 0}, {6, 7, 0}, {7, 3, 0}, {0, 0, 2}, {1, 7, 0}, {2, 3, 0},   // +167.633s 167632
{3, 1, 0}, {5, 0, 0}, {6, 1, 0}, {7, 4, 0}, {7, 6, 0}, {2, 2, 2}, {6, 7, 2}, {7, 0, 0}, {0, 6, 2}, {1, 2, 0}, 
{3, 2, 0}, {5, 6, 0}, {6, 0, 0}, {7, 5, 0}, {4, 6, 2}, {4, 7, 0}, {6, 6, 0}, {7, 1, 0}, {3, 3, 2}, {6, 4, 0}, 
{0, 5, 3}, {1, 3, 0}, {5, 5, 0}, {3, 4, 2}, {6, 3, 0}, {6, 5, 0}, {7, 3, 0}, {2, 0, 2}, {2, 7, 0}, {0, 4, 2}, 
{1, 4, 0}, {6, 2, 0}, {3, 5, 2}, {5, 4, 0}, {6, 7, 0}, {7, 0, 0}, {2, 1, 2}, {2, 2, 2}, {6, 1, 0}, {0, 3, 2}, 
{1, 5, 0}, {2, 3, 0}, {3, 6, 0}, {5, 3, 0}, {6, 6, 0}, {7, 4, 0}, {7, 5, 0}, {7, 6, 0}, {6, 0, 2}, {7, 1, 0},   // +168.100s 168096
{4, 6, 2}, {5, 7, 0}, {0, 2, 2}, {1, 0, 0}, {3, 7, 0}, {6, 4, 0}, {5, 2, 3}, {6, 5, 0}, {7, 3, 0}, {4, 0, 2}, 
{6, 3, 0}, {1, 1, 2}, {2, 0, 0}, {0, 1, 2}, {5, 1, 0}, {6, 5, 0}, {7, 0, 0}, {1, 2, 2}, {4, 1, 0}, {6, 2, 0}, 
{2, 1, 2}, {6, 6, 0}, {7, 1, 0}, {0, 0, 2}, {1, 6, 0}, {4, 2, 0}, {1, 3, 2}, {2, 3, 0}, {5, 0, 0}, {2, 2, 2}, 
{6, 1, 0}, {6, 7, 0}, {7, 3, 0}, {7, 4, 0}, {7, 6, 0}, {0, 0, 2}, {1, 4, 0}, {4, 3, 0}, {5, 6, 0}, {7, 5, 0}, 
{4, 7, 2}, {6, 0, 2}, {6, 5, 0}, {0, 1, 3}, {1, 5, 0}, {2, 4, 0}, {5, 5, 0}, {6, 4, 0}, {7, 0, 0}, {0, 2, 4},   // +168.667s 168672
{5, 4, 0}, {6, 3, 0}, {6, 6, 0}, {7, 2, 0}, {1, 0, 2}, {2, 0, 0}, {2, 5, 0}, {7, 1, 0}, {6, 2, 2}, {0, 3, 2}, 
{2, 1, 0}, {5, 3, 0}, {1, 1, 2}, {6, 7, 0}, {7, 3, 0}, {0, 4, 2}, {2, 6, 0}, {5, 2, 0}, {6, 1, 0}, {7, 5, 0}, 
{2, 2, 2}, {2, 3, 0}, {7, 0, 0}, {7, 4, 0}, {7, 6, 0}, {1, 2, 2}, {0, 5, 2}, {5, 1, 0}, {6, 0, 0}, {6, 4, 0}, 
{6, 7, 0}, {4, 4, 2}, {4, 6, 0}, {4, 7, 0}, {7, 1, 0}, {0, 6, 3}, {1, 3, 0}, {6, 6, 0}, {5, 0, 2}, {6, 3, 0}, 
{2, 0, 2}, {7, 3, 0}, {0, 0, 2}, {1, 4, 0}, {4, 5, 0}, {6, 5, 0}, {1, 7, 2}, {5, 6, 0}, {6, 2, 0}, {2, 1, 4},   // +169.200s 169200
{6, 7, 0}, {7, 0, 0}, {0, 1, 2}, {1, 5, 0}, {3, 0, 0}, {5, 5, 0}, {2, 2, 2}, {2, 3, 0}, {6, 1, 0}, {7, 4, 0}, 
{7, 6, 0}, {0, 2, 4}, {3, 1, 0}, {6, 6, 0}, {7, 1, 0}, {7, 5, 0}, {1, 0, 2}, {2, 7, 0}, {4, 7, 0}, {5, 4, 0}, 
{6, 0, 0}, {6, 4, 5}, {7, 3, 0}, {0, 3, 2}, {1, 1, 0}, {3, 2, 0}, {5, 3, 0}, {6, 3, 0}, {6, 5, 0}, {2, 0, 2}, 
{1, 2, 2}, {7, 0, 0}, {0, 4, 2}, {5, 2, 0}, {6, 2, 0}, {6, 5, 0}, {2, 1, 2}, {3, 3, 0}, {7, 1, 0}, {1, 3, 2}, 
{5, 7, 0}, {6, 1, 0}, {6, 6, 0}, {2, 2, 2}, {0, 5, 2}, {2, 3, 0}, {3, 4, 0}, {5, 1, 0}, {6, 0, 0}, {6, 7, 0},   // +169.700s 169696
{7, 3, 0}, {7, 4, 0}, {7, 5, 0}, {7, 6, 0}, {1, 4, 2}, {4, 6, 2}, {0, 6, 2}, {1, 5, 0}, {6, 3, 0}, {6, 4, 0}, 
{6, 5, 0}, {7, 0, 0}, {3, 5, 3}, {5, 0, 0}, {1, 6, 2}, {0, 6, 2}, {1, 0, 0}, {2, 0, 0}, {5, 6, 2}, {6, 2, 0}, 
{6, 6, 0}, {7, 1, 0}, {0, 5, 2}, {3, 6, 0}, {1, 1, 2}, {5, 5, 0}, {2, 1, 2}, {6, 1, 0}, {6, 7, 0}, {7, 3, 0}, 
{0, 4, 2}, {1, 2, 0}, {3, 7, 0}, {2, 2, 2}, {2, 3, 0}, {5, 4, 0}, {7, 0, 0}, {7, 2, 0}, {7, 4, 0}, {7, 6, 0}, 
{0, 3, 2}, {6, 7, 0}, {7, 5, 0}, {4, 0, 2}, {5, 3, 0}, {6, 0, 0}, {1, 3, 3}, {4, 6, 0}, {4, 7, 0}, {7, 1, 0},   // +170.200s 170208
{0, 2, 2}, {6, 4, 0}, {6, 6, 0}, {5, 2, 2}, {6, 3, 0}, {2, 0, 2}, {4, 1, 0}, {6, 5, 0}, {7, 3, 0}, {0, 1, 2}, 
{1, 4, 0}, {6, 2, 0}, {5, 1, 2}, {0, 0, 2}, {2, 1, 0}, {4, 2, 0}, {6, 7, 0}, {1, 5, 2}, {5, 0, 0}, {6, 1, 0}, 
{7, 0, 0}, {0, 6, 4}, {2, 2, 0}, {2, 3, 0}, {6, 0, 0}, {7, 5, 0}, {4, 3, 2}, {5, 6, 0}, {6, 6, 0}, {7, 1, 0}, 
{7, 4, 0}, {7, 6, 0}, {1, 0, 2}, {1, 7, 2}, {4, 6, 0}, {6, 3, 0}, {6, 4, 0}, {0, 5, 3}, {2, 4, 0}, {5, 5, 0}, 
{6, 5, 0}, {7, 3, 0}, {1, 1, 2}, {2, 0, 2}, {6, 2, 0}, {0, 4, 2}, {1, 2, 0}, {2, 5, 0}, {6, 5, 0}, {7, 0, 0},   // +170.733s 170736
{5, 4, 2}, {6, 6, 2}, {7, 1, 0}, {1, 3, 2}, {2, 1, 0}, {2, 6, 0}, {2, 7, 0}, {6, 1, 0}, {0, 3, 2}, {5, 3, 0}, 
{1, 4, 2}, {4, 4, 0}, {6, 7, 0}, {7, 3, 0}, {7, 4, 0}, {7, 6, 0}, {2, 2, 2}, {2, 3, 0}, {6, 0, 0}, {7, 5, 0}, 
{0, 2, 2}, {1, 5, 3}, {4, 5, 0}, {4, 7, 0}, {5, 2, 0}, {6, 5, 0}, {7, 0, 0}, {6, 3, 2}, {6, 4, 0}, {0, 1, 2}, 
{1, 0, 0}, {3, 0, 0}, {5, 1, 2}, {5, 7, 0}, {6, 6, 0}, {2, 0, 2}, {6, 2, 0}, {7, 1, 0}, {1, 1, 2}, {3, 1, 0}, 
{0, 0, 2}, {2, 1, 0}, {6, 1, 0}, {3, 2, 2}, {5, 0, 0}, {6, 7, 0}, {7, 3, 0}, {1, 2, 2}, {0, 0, 2}, {2, 2, 0},   // +171.300s 171296
{6, 0, 0}, {7, 0, 0}, {7, 5, 0}, {2, 3, 2}, {3, 3, 0}, {5, 6, 0}, {6, 7, 0}, {7, 4, 0}, {7, 6, 0}, {1, 6, 2}, 
{0, 1, 3}, {1, 3, 0}, {3, 4, 0}, {5, 5, 0}, {6, 3, 0}, {6, 4, 0}, {6, 6, 0}, {7, 1, 0}, {4, 6, 2}, {4, 7, 0}, 
{0, 2, 2}, {3, 5, 2}, {5, 4, 0}, {6, 5, 0}, {7, 3, 0}, {1, 4, 2}, {2, 0, 0}, {6, 2, 0}, {0, 3, 2}, {5, 3, 0}, 
{3, 6, 2}, {6, 7, 0}, {7, 0, 0}, {7, 2, 0}, {0, 4, 2}, {1, 5, 0}, {2, 1, 0}, {6, 1, 0}, {3, 7, 2}, {5, 2, 0}, 
{6, 6, 2}, {0, 5, 2}, {2, 3, 0}, {5, 1, 0}, {6, 0, 0}, {7, 1, 0}, {7, 4, 0}, {7, 6, 0}, {1, 0, 2}, {2, 2, 0},   // +171.767s 171760
{4, 0, 0}, {7, 5, 0}, {0, 6, 2}, {6, 5, 0}, {7, 3, 0}, {1, 1, 15}, {6, 5, 0}, {0, 0, 2}, {4, 7, 0}, {5, 0, 0}, 
{6, 3, 0}, {6, 4, 0}, {7, 0, 0}, {1, 2, 6}, {2, 0, 0}, {2, 2, 0}, {4, 1, 0}, {6, 6, 0}, {2, 3, 3}, {7, 1, 0}, 
{7, 4, 0}, {7, 6, 0}, {0, 1, 6}, {4, 2, 0}, {5, 6, 0}, {6, 2, 0}, {1, 3, 6}, {2, 1, 0}, {6, 7, 0}, {1, 7, 2}, 
{7, 3, 0}, {4, 6, 2}, {0, 2, 2}, {1, 4, 0}, {4, 3, 0}, {5, 5, 0}, {6, 1, 0}, {7, 5, 0}, {6, 5, 2}, {6, 0, 2}, 
{7, 0, 0}, {1, 5, 3}, {2, 2, 0}, {2, 3, 0}, {2, 4, 0}, {5, 4, 0}, {6, 4, 0}, {7, 4, 0}, {7, 6, 0}, {0, 3, 2},   // +172.633s 172640
{6, 6, 0}, {1, 0, 2}, {2, 7, 0}, {6, 3, 0}, {7, 1, 0}, {2, 0, 2}, {0, 4, 2}, {2, 5, 0}, {5, 3, 0}, {6, 7, 0}, 
{1, 1, 2}, {6, 2, 0}, {7, 3, 0}, {2, 1, 4}, {5, 2, 0}, {6, 7, 0}, {7, 0, 0}, {0, 5, 2}, {1, 2, 0}, {2, 6, 0}, 
{4, 6, 2}, {4, 7, 0}, {5, 7, 0}, {6, 1, 0}, {6, 6, 2}, {7, 1, 0}, {7, 5, 0}, {0, 6, 2}, {4, 4, 0}, {5, 1, 0}, 
{1, 3, 2}, {2, 2, 0}, {2, 3, 0}, {6, 0, 0}, {6, 5, 0}, {7, 4, 0}, {7, 6, 0}, {6, 4, 3}, {7, 3, 0}, {0, 6, 2}, 
{5, 0, 0}, {1, 4, 2}, {4, 5, 0}, {6, 3, 0}, {6, 7, 0}, {0, 5, 2}, {1, 6, 0}, {2, 0, 0}, {7, 0, 0}, {5, 6, 2},   // +173.167s 173168
{2, 1, 2}, {3, 0, 0}, {6, 2, 0}, {0, 4, 2}, {1, 5, 0}, {6, 6, 0}, {5, 5, 2}, {6, 1, 0}, {7, 1, 0}, {0, 3, 2}, 
{3, 1, 0}, {4, 6, 0}, {7, 5, 0}, {1, 0, 2}, {5, 4, 0}, {6, 5, 0}, {6, 0, 2}, {7, 3, 0}, {0, 2, 2}, {2, 2, 0}, 
{2, 3, 0}, {6, 4, 0}, {7, 2, 0}, {7, 4, 0}, {7, 6, 0}, {1, 1, 3}, {3, 2, 0}, {5, 3, 0}, {6, 5, 0}, {7, 0, 0}, 
{0, 1, 2}, {6, 3, 0}, {2, 0, 2}, {5, 2, 0}, {1, 2, 2}, {3, 3, 0}, {6, 6, 0}, {7, 1, 0}, {0, 0, 2}, {5, 1, 0}, 
{6, 2, 0}, {1, 3, 2}, {6, 7, 0}, {2, 1, 2}, {7, 3, 0}, {0, 6, 2}, {3, 4, 0}, {5, 0, 0}, {6, 1, 0}, {1, 4, 2},   // +173.700s 173696
{4, 7, 0}, {6, 5, 0}, {7, 0, 2}, {7, 5, 0}, {0, 5, 2}, {1, 5, 0}, {3, 5, 0}, {5, 6, 0}, {2, 2, 3}, {6, 0, 0}, 
{6, 6, 0}, {2, 3, 2}, {6, 4, 0}, {7, 1, 0}, {7, 4, 0}, {7, 6, 0}, {1, 0, 2}, {1, 7, 0}, {5, 5, 0}, {0, 4, 2}, 
{2, 0, 0}, {3, 6, 0}, {6, 3, 0}, {6, 7, 2}, {7, 3, 0}, {1, 1, 2}, {6, 2, 0}, {0, 3, 2}, {2, 1, 0}, {3, 7, 0}, 
{5, 4, 0}, {1, 2, 2}, {6, 7, 0}, {7, 0, 0}, {6, 1, 2}, {7, 5, 0}, {0, 2, 2}, {5, 3, 0}, {6, 6, 0}, {7, 1, 0}, 
{2, 7, 2}, {4, 0, 0}, {4, 6, 0}, {4, 7, 0}, {6, 0, 0}, {1, 3, 2}, {6, 4, 0}, {2, 3, 3}, {6, 5, 0}, {7, 3, 0},   // +174.200s 174208
{0, 1, 2}, {2, 2, 0}, {4, 1, 0}, {5, 2, 0}, {6, 3, 0}, {7, 4, 0}, {7, 6, 0}, {1, 4, 4}, {2, 0, 0}, {6, 7, 0}, 
{0, 0, 2}, {5, 1, 0}, {7, 0, 0}, {4, 2, 2}, {5, 7, 0}, {6, 2, 0}, {1, 5, 2}, {2, 1, 0}, {6, 6, 0}, {0, 0, 2}, 
{7, 1, 0}, {4, 3, 2}, {5, 0, 0}, {6, 1, 0}, {0, 1, 4}, {1, 0, 0}, {4, 7, 0}, {5, 6, 0}, {6, 5, 0}, {7, 3, 0}, 
{7, 5, 0}, {2, 4, 2}, {0, 2, 2}, {6, 0, 0}, {1, 1, 3}, {1, 6, 0}, {2, 2, 0}, {2, 3, 0}, {2, 5, 0}, {5, 5, 0}, 
{6, 4, 0}, {6, 5, 0}, {7, 0, 0}, {6, 3, 2}, {7, 4, 0}, {7, 6, 0}, {0, 3, 2}, {1, 2, 0}, {2, 0, 0}, {5, 4, 0},   // +174.700s 174704
{6, 6, 0}, {2, 6, 2}, {7, 1, 0}, {0, 4, 2}, {6, 2, 0}, {6, 7, 0}, {1, 3, 2}, {2, 1, 0}, {4, 4, 0}, {5, 3, 0}, 
{7, 3, 0}, {6, 1, 2}, {0, 5, 2}, {1, 4, 0}, {5, 2, 0}, {6, 5, 0}, {7, 2, 0}, {7, 5, 0}, {4, 5, 2}, {0, 6, 2}, 
{4, 6, 0}, {6, 0, 0}, {7, 0, 0}, {1, 5, 2}, {3, 0, 0}, {5, 1, 0}, {6, 4, 0}, {6, 6, 3}, {0, 0, 2}, {1, 0, 0}, 
{2, 3, 0}, {5, 0, 0}, {6, 3, 0}, {7, 1, 0}, {7, 4, 0}, {7, 6, 0}, {2, 0, 2}, {2, 2, 0}, {3, 1, 0}, {6, 7, 2}, 
{0, 1, 2}, {1, 1, 0}, {5, 6, 0}, {6, 2, 0}, {7, 3, 0}, {2, 1, 2}, {3, 2, 0}, {6, 7, 2}, {1, 2, 2}, {3, 3, 0},   // +175.233s 175232
{7, 0, 0}, {0, 2, 2}, {5, 5, 0}, {6, 1, 0}, {3, 4, 2}, {6, 6, 0}, {7, 1, 0}, {7, 5, 0}, {1, 3, 2}, {0, 3, 2}, 
{1, 7, 0}, {4, 6, 0}, {4, 7, 0}, {5, 4, 0}, {6, 0, 0}, {6, 5, 0}, {3, 5, 3}, {6, 4, 0}, {7, 3, 0}, {1, 4, 4}, 
{2, 0, 0}, {2, 2, 0}, {2, 3, 0}, {3, 6, 0}, {6, 3, 0}, {6, 7, 0}, {7, 4, 0}, {7, 6, 0}, {0, 4, 2}, {5, 3, 0}, 
{7, 0, 2}, {2, 1, 2}, {3, 7, 0}, {6, 2, 0}, {0, 5, 2}, {1, 5, 0}, {2, 7, 0}, {5, 2, 0}, {6, 6, 0}, {6, 1, 2}, 
{7, 1, 0}, {4, 0, 2}, {7, 5, 0}, {1, 0, 2}, {5, 1, 0}, {6, 0, 0}, {6, 5, 0}, {0, 6, 2}, {4, 1, 0}, {6, 4, 0},   // +175.733s 175728
{7, 3, 0}, {4, 6, 2}, {1, 1, 3}, {6, 3, 0}, {6, 5, 0}, {7, 0, 0}, {0, 6, 2}, {4, 2, 0}, {5, 0, 0}, {2, 0, 2}, 
{2, 2, 0}, {2, 3, 0}, {5, 7, 0}, {6, 6, 0}, {7, 4, 0}, {7, 6, 0}, {0, 5, 2}, {1, 2, 0}, {7, 1, 0}, {4, 3, 2}, 
{5, 6, 0}, {6, 2, 0}, {0, 4, 2}, {1, 3, 0}, {6, 7, 0}, {2, 1, 2}, {5, 5, 0}, {7, 3, 0}, {1, 4, 2}, {2, 4, 0}, 
{6, 1, 0}, {0, 3, 2}, {1, 6, 0}, {6, 5, 0}, {5, 4, 2}, {7, 0, 0}, {7, 5, 0}, {0, 2, 2}, {1, 5, 0}, {2, 5, 0}, 
{4, 7, 2}, {5, 3, 0}, {6, 0, 0}, {6, 6, 0}, {1, 0, 3}, {6, 4, 0}, {0, 1, 2}, {2, 6, 0}, {7, 1, 0}, {2, 0, 2},   // +176.267s 176272
{2, 2, 0}, {5, 2, 0}, {6, 3, 0}, {0, 0, 2}, {1, 1, 0}, {2, 3, 0}, {6, 7, 0}, {7, 4, 0}, {7, 6, 0}, {2, 1, 2}, 
{5, 1, 0}, {6, 2, 0}, {7, 2, 0}, {7, 3, 0}, {4, 4, 2}, {0, 6, 2}, {1, 2, 0}, {6, 7, 0}, {7, 0, 0}, {5, 0, 2}, 
{6, 1, 0}, {7, 5, 0}, {4, 5, 2}, {6, 6, 0}, {6, 0, 2}, {7, 1, 0}, {0, 5, 2}, {1, 3, 0}, {5, 6, 0}, {6, 4, 0}, 
{6, 5, 2}, {3, 0, 3}, {4, 6, 0}, {4, 7, 0}, {6, 3, 0}, {7, 3, 0}, {0, 4, 2}, {5, 5, 0}, {1, 4, 2}, {2, 0, 0}, 
{2, 3, 0}, {6, 7, 0}, {2, 2, 2}, {3, 1, 0}, {6, 2, 0}, {7, 0, 0}, {7, 4, 0}, {7, 6, 0}, {5, 4, 2}, {0, 3, 2},   // +176.767s 176768
{1, 5, 0}, {2, 1, 0}, {6, 6, 0}, {3, 2, 2}, {7, 1, 0}, {1, 7, 2}, {6, 1, 0}, {0, 2, 2}, {5, 3, 0}, {6, 5, 0}, 
{7, 5, 0}, {1, 0, 2}, {7, 3, 0}, {3, 3, 2}, {6, 0, 0}, {0, 1, 2}, {5, 2, 0}, {6, 4, 0}, {6, 5, 0}, {1, 1, 2}, 
{4, 7, 0}, {7, 0, 0}, {3, 4, 3}, {6, 3, 0}, {1, 2, 2}, {2, 0, 0}, {2, 7, 0}, {6, 6, 0}, {7, 1, 0}, {0, 0, 2}, 
{2, 2, 0}, {2, 3, 0}, {5, 1, 0}, {7, 4, 0}, {7, 6, 0}, {1, 3, 2}, {2, 1, 0}, {6, 2, 0}, {6, 7, 0}, {3, 5, 2}, 
{7, 3, 0}, {0, 0, 2}, {5, 0, 0}, {6, 1, 0}, {1, 4, 2}, {6, 5, 0}, {7, 5, 0}, {0, 1, 2}, {3, 6, 0}, {1, 5, 2},   // +177.300s 177296
{6, 0, 0}, {7, 0, 0}, {5, 6, 2}, {5, 7, 0}, {6, 4, 0}, {0, 2, 2}, {6, 6, 0}, {1, 0, 2}, {3, 7, 0}, {4, 6, 0}, 
{5, 5, 0}, {6, 3, 0}, {7, 1, 0}, {0, 3, 3}, {2, 0, 0}, {5, 4, 2}, {6, 7, 0}, {1, 1, 2}, {2, 2, 0}, {2, 3, 0}, 
{4, 0, 0}, {6, 2, 0}, {7, 3, 0}, {7, 4, 0}, {7, 6, 0}, {0, 4, 2}, {2, 1, 0}, {5, 3, 0}, {6, 7, 2}, {0, 5, 2}, 
{1, 2, 0}, {1, 6, 0}, {7, 0, 0}, {4, 1, 2}, {5, 2, 0}, {6, 1, 0}, {6, 6, 0}, {7, 1, 2}, {7, 5, 0}, {0, 6, 2}, 
{5, 1, 0}, {1, 3, 2}, {4, 2, 0}, {6, 0, 0}, {6, 5, 0}, {6, 4, 2}, {7, 3, 0}, {0, 0, 3}, {5, 0, 0}, {1, 4, 2},   // +177.833s 177840
{2, 0, 0}, {4, 6, 0}, {4, 7, 0}, {6, 3, 0}, {6, 7, 0}, {7, 2, 0}, {4, 3, 2}, {7, 0, 0}, {0, 1, 2}, {2, 2, 0}, 
{5, 6, 0}, {6, 2, 0}, {2, 1, 2}, {2, 3, 0}, {6, 6, 0}, {7, 4, 0}, {7, 6, 0}, {1, 5, 2}, {2, 4, 0}, {0, 2, 2}, 
{5, 5, 0}, {6, 1, 0}, {7, 1, 0}, {7, 5, 0}, {2, 5, 2}, {1, 0, 2}, {6, 0, 0}, {6, 5, 0}, {6, 4, 2}, {7, 3, 0}, 
{0, 3, 2}, {2, 6, 0}, {5, 4, 0}, {1, 1, 2}, {6, 3, 0}, {6, 5, 0}, {7, 0, 0}, {2, 0, 2}, {4, 4, 0}, {0, 4, 3}, 
{1, 2, 0}, {4, 6, 0}, {5, 3, 0}, {6, 6, 0}, {4, 5, 2}, {7, 1, 0}, {1, 7, 2}, {6, 2, 0}, {1, 3, 2}, {2, 1, 0},   // +178.333s 178336
{2, 2, 0}, {2, 3, 0}, {3, 0, 0}, {6, 7, 0}, {7, 4, 0}, {7, 6, 0}, {0, 5, 2}, {5, 2, 0}, {7, 3, 0}, {1, 4, 2}, 
{6, 1, 0}, {3, 1, 2}, {6, 5, 0}, {0, 6, 2}, {5, 1, 0}, {7, 0, 0}, {7, 5, 0}, {1, 5, 2}, {3, 2, 0}, {6, 0, 2}, 
{6, 6, 0}, {0, 6, 2}, {1, 0, 0}, {2, 7, 0}, {6, 4, 0}, {3, 3, 3}, {5, 0, 0}, {7, 1, 0}, {2, 0, 2}, {4, 7, 0}, 
{6, 3, 0}, {0, 5, 2}, {1, 1, 0}, {6, 7, 0}, {2, 1, 2}, {3, 4, 0}, {5, 6, 0}, {6, 2, 0}, {7, 3, 0}, {0, 4, 2}, 
{2, 2, 0}, {2, 3, 0}, {6, 7, 0}, {1, 2, 2}, {3, 5, 0}, {5, 5, 0}, {6, 1, 0}, {7, 0, 0}, {7, 4, 0}, {7, 6, 0},   // +178.767s 178768
{5, 7, 2}, {7, 5, 0}, {0, 3, 2}, {6, 6, 0}, {7, 1, 0}, {3, 6, 2}, {5, 4, 0}, {6, 0, 0}, {0, 2, 2}, {1, 3, 0}, 
{6, 4, 0}, {6, 5, 0}, {3, 7, 2}, {5, 3, 0}, {7, 3, 0}, {0, 1, 2}, {6, 3, 0}, {4, 0, 3}, {6, 7, 0}, {1, 4, 2}, 
{1, 6, 0}, {2, 0, 0}, {5, 2, 0}, {0, 0, 2}, {4, 6, 0}, {4, 7, 0}, {6, 2, 0}, {7, 0, 0}, {4, 1, 2}, {1, 5, 2}, 
{2, 1, 0}, {2, 3, 0}, {5, 1, 0}, {6, 6, 0}, {7, 4, 0}, {7, 6, 0}, {0, 6, 2}, {2, 2, 0}, {7, 1, 0}, {4, 2, 2}, 
{5, 0, 0}, {6, 1, 0}, {1, 0, 2}, {6, 5, 0}, {7, 5, 0}, {0, 5, 2}, {4, 3, 0}, {7, 3, 0}, {5, 6, 2}, {6, 0, 0},   // +179.300s 179296
{7, 2, 0}, {1, 1, 2}, {6, 4, 0}, {6, 5, 0}, {2, 4, 2}, {7, 0, 0}, {0, 4, 2}, {5, 5, 0}, {6, 3, 0}, {1, 2, 3}, 
{2, 0, 0}, {4, 7, 0}, {6, 6, 0}, {7, 1, 0}, {2, 5, 2}, {0, 3, 2}, {1, 3, 0}, {2, 1, 0}, {6, 2, 0}, {6, 7, 0}, 
{2, 2, 2}, {5, 4, 0}, {7, 3, 0}, {2, 3, 2}, {6, 1, 0}, {7, 4, 0}, {7, 6, 0}, {1, 4, 2}, {2, 6, 0}, {6, 5, 0}, 
{7, 5, 0}, {0, 2, 2}, {5, 3, 0}, {6, 0, 0}, {1, 5, 2}, {6, 4, 0}, {7, 0, 0}, {4, 4, 2}, {6, 6, 0}, {0, 1, 2}, 
{5, 2, 0}, {6, 3, 0}, {1, 0, 2}, {1, 7, 0}, {7, 1, 0}, {2, 0, 2}, {4, 5, 3}, {6, 7, 0}, {0, 0, 2}, {1, 1, 0},   // +179.867s 179872
{4, 6, 0}, {5, 1, 0}, {6, 2, 0}, {7, 3, 0}, {2, 1, 2}, {0, 0, 2}, {6, 7, 0}, {7, 0, 0}, {1, 2, 2}, {2, 2, 0}, 
{2, 3, 0}, {3, 0, 0}, {5, 0, 0}, {6, 1, 0}, {7, 4, 0}, {7, 6, 0}, {6, 6, 2}, {0, 1, 2}, {2, 7, 0}, {7, 1, 0}, 
{7, 5, 0}, {1, 3, 2}, {3, 1, 0}, {5, 6, 0}, {0, 2, 2}, {6, 0, 0}, {6, 5, 0}, {6, 4, 2}, {7, 3, 0}, {3, 2, 2}, 
{5, 5, 0}, {0, 3, 3}, {1, 4, 0}, {2, 0, 0}, {6, 3, 0}, {6, 7, 0}, {5, 4, 2}, {7, 0, 0}, {0, 4, 2}, {5, 7, 0}, 
{6, 2, 0}, {1, 5, 2}, {2, 1, 0}, {3, 3, 0}, {4, 6, 0}, {4, 7, 0}, {6, 6, 0}, {5, 3, 2}, {0, 5, 2}, {2, 2, 0},   // +180.367s 180368
{2, 3, 0}, {6, 1, 0}, {7, 1, 0}, {7, 5, 0}, {3, 4, 2}, {5, 2, 0}, {7, 4, 0}, {7, 6, 0}, {0, 6, 2}, {1, 0, 0}, 
{6, 0, 0}, {6, 5, 0}, {6, 4, 2}, {7, 3, 0}, {3, 5, 2}, {5, 1, 0}, {1, 1, 2}, {1, 6, 0}, {6, 5, 0}, {7, 0, 0}, 
{0, 0, 2}, {2, 0, 0}, {5, 0, 0}, {6, 3, 0}, {1, 2, 3}, {6, 6, 0}, {3, 6, 2}, {7, 1, 0}, {0, 1, 2}, {4, 6, 0}, 
{5, 6, 0}, {6, 2, 0}, {6, 7, 0}, {1, 3, 2}, {2, 1, 0}, {3, 7, 2}, {7, 3, 0}, {1, 4, 2}, {2, 3, 0}, {6, 1, 0}, 
{6, 5, 0}, {7, 4, 0}, {7, 6, 0}, {0, 2, 2}, {2, 2, 0}, {5, 5, 0}, {7, 2, 0}, {7, 5, 0}, {7, 0, 2}, {1, 5, 2},   // +180.867s 180864
{4, 0, 0}, {6, 0, 0}, {0, 3, 2}, {5, 4, 0}, {6, 4, 0}, {6, 6, 0}, {1, 0, 2}, {7, 1, 0}, {4, 1, 2}, {6, 3, 0}, 
{0, 4, 2}, {2, 0, 0}, {5, 3, 0}, {6, 7, 0}, {1, 1, 3}, {7, 3, 0}, {2, 1, 2}, {6, 2, 0}, {4, 2, 2}, {4, 7, 0}, 
{6, 7, 0}, {0, 5, 2}, {1, 2, 0}, {5, 2, 0}, {6, 1, 0}, {7, 0, 0}, {2, 2, 2}, {7, 5, 0}, {2, 3, 2}, {6, 6, 0}, 
{7, 1, 0}, {7, 4, 0}, {7, 6, 0}, {0, 6, 2}, {1, 3, 0}, {4, 3, 0}, {5, 1, 0}, {6, 0, 0}, {1, 7, 2}, {6, 4, 0}, 
{6, 5, 0}, {2, 4, 2}, {7, 3, 0}, {0, 6, 2}, {6, 3, 0}, {1, 4, 2}, {2, 0, 0}, {2, 5, 0}, {5, 0, 0}, {6, 7, 0},   // +181.367s 181360
{0, 5, 5}, {6, 2, 0}, {7, 0, 0}, {2, 6, 2}, {5, 6, 0}, {0, 4, 2}, {1, 5, 0}, {2, 1, 0}, {2, 7, 0}, {4, 6, 0}, 
{4, 7, 0}, {6, 6, 0}, {4, 4, 2}, {5, 5, 0}, {7, 1, 0}, {6, 1, 2}, {0, 3, 2}, {1, 0, 0}, {2, 2, 0}, {2, 3, 0}, 
{5, 4, 0}, {6, 5, 0}, {7, 4, 0}, {7, 5, 0}, {7, 6, 0}, {4, 5, 2}, {7, 3, 0}, {0, 2, 2}, {6, 0, 0}, {1, 1, 2}, 
{3, 0, 0}, {5, 3, 0}, {6, 4, 0}, {6, 5, 0}, {7, 0, 2}, {0, 1, 2}, {5, 7, 0}, {6, 3, 0}, {6, 6, 0}, {1, 2, 2}, 
{2, 0, 0}, {3, 1, 0}, {5, 2, 0}, {7, 1, 0}, {0, 0, 3}, {6, 2, 0}, {1, 3, 2}, {2, 1, 0}, {5, 1, 0}, {6, 7, 0},   // +181.867s 181872
{3, 2, 2}, {4, 7, 0}, {7, 3, 0}, {0, 6, 2}, {6, 1, 0}, {7, 5, 0}, {1, 4, 2}, {3, 3, 0}, {5, 0, 0}, {6, 5, 0}*/
};

//------------------------------------------------------------------------------
int numReplays(void)
{
    return sizeof(kLampReplay)/sizeof(kLampReplay[0]);
}

//------------------------------------------------------------------------------
byte replay(byte col)
{
    static byte replayLamps[NUM_COL] = {0};
    static uint32_t lastUpdTtag = 0;
    static uint32_t replayPos = 0;
    //static uint32_t numReplays = (sizeof(kLampReplay) / sizeof(AG_LAMP_SWITCH_t));
    static uint16_t currData = 0;
    AG_LAMP_SWITCH_t *pEv = (AG_LAMP_SWITCH_t*)&currData;
    int nr = numReplays();

    // update the lamp matrix
    uint32_t replayTtag = (sTtag >> 6); //(sTtag / (REPLAY_TTAG_SCALE / TTAG_INT_A));
    if (lastUpdTtag == 0)
    {
        lastUpdTtag = replayTtag;
    }
    uint32_t dTtag = (replayTtag - lastUpdTtag);
    if (dTtag >= pEv->dttag)
    {
        // handle all events of this ttag
        currData = pgm_read_word_near(kLampReplay + replayPos);
        do
        {
            replayLamps[pEv->col] ^= (1 << pEv->row);
            replayPos++;
            if (replayPos > nr)
            {
                // start over again
                replayPos = 0;
                lastUpdTtag = 0;
                memset(replayLamps, 0, sizeof(replayLamps));
            }
            currData = pgm_read_word_near(kLampReplay + replayPos);
        } while (pEv->dttag == 0);
        lastUpdTtag = replayTtag;
    }

    // return the current row from the replay lamp matrix
    return replayLamps[col];
}

#endif // REPLAY_ENABLED
