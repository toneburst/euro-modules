//   ######   #######  ########  ##    ## ########  ####  ######   ##     ## ########
//  ##    ## ##     ## ##     ##  ##  ##  ##     ##  ##  ##    ##  ##     ##    ##
//  ##       ##     ## ##     ##   ####   ##     ##  ##  ##        ##     ##    ##
//  ##       ##     ## ########     ##    ########   ##  ##   #### #########    ##
//  ##       ##     ## ##           ##    ##   ##    ##  ##    ##  ##     ##    ##
//  ##    ## ##     ## ##           ##    ##    ##   ##  ##    ##  ##     ##    ##
//   ######   #######  ##           ##    ##     ## ####  ######   ##     ##    ##

// Formant Synthesiser
// original code by Peter Knight
// https://code.google.com/p/tinkerit/wiki/Cantarino
// Modified by Jean-Luc Deladrière for the Talko modular Synth
// Further modified by toneburst

// Sound = vowels
// Pitch = pitch
// Speed = not used yet
// Bend =  not used yet

//   ######   ##        #######  ########     ###    ##
//  ##    ##  ##       ##     ## ##     ##   ## ##   ##
//  ##        ##       ##     ## ##     ##  ##   ##  ##
//  ##   #### ##       ##     ## ########  ##     ## ##
//  ##    ##  ##       ##     ## ##     ## ######### ##
//  ##    ##  ##       ##     ## ##     ## ##     ## ##
//   ######   ########  #######  ########  ##     ## ########

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <tables.h>

// Formant phases
uint16_t pitchPhase, form1Phase,form2Phase,form3Phase;
// Formant phase-increments (freq)
uint16_t pitchPhaseInc,form1PhaseInc,form2PhaseInc,form3PhaseInc;
// Formant amplitudes
uint8_t form1Amp,form2Amp,form3Amp;
// Noise-modulation amount
uint8_t noiseMod = 10;

// Default frame-time (not currently used)
int frameTime = 15; // 15ms

// Scale formant frequencies
int formantScale;

//  ##        #######   #######  ########
//  ##       ##     ## ##     ## ##     ##
//  ##       ##     ## ##     ## ##     ##
//  ##       ##     ## ##     ## ########
//  ##       ##     ## ##     ## ##
//  ##       ##     ## ##     ## ##
//  ########  #######   #######  ##


void loop() {

    formantScale = 54;
    const uint8_t *framePos = frameList;

    // INFINITE LOOP
    while(1) {
        //  digitalWrite(LED_PIN,digitalRead(2)); dot led with gate
        int n;
        uint8_t startFormant,staticFrames,tweenFrames;
        uint16_t startPitch,nextPitch;
        uint8_t nextFormant;
        int16_t startForm1PhaseInc,startForm2PhaseInc,startForm3PhaseInc;
        uint8_t startForm1Amp,startForm2Amp,startForm3Amp;
        uint8_t startMod;
        const uint8_t *formantPos;

        // Read next framelist item
        startFormant = pgm_read_byte(framePos++);
        staticFrames = pgm_read_byte(framePos++);

        // if (!staticFrames) break; // End of phrase
        if (digitalRead(2) == 0) break; // End of phrase

        tweenFrames = pgm_read_byte(framePos++);

        startPitch = pitchTable[pgm_read_byte(framePos++)];
        //nextFormant = pgm_read_byte(framePos);
        nextPitch = pitchTable[pgm_read_byte(framePos+3)];

        //Serial.println(startFormant);

        pitchPhaseInc = startPitch;

        do {
            startFormant = vowels[map(analogRead(1), 0,1024,0,18)];
            //startFormant= map(analogRead(1), 0,1020,0,77);
            //Serial.println(startFormant);
            pitchPhaseInc = pitchTable[map(analogRead(2), 0,1024,0,64)];
            formantPos = formantTable + startFormant * FORMANT_SZ;
            form1PhaseInc = startForm1PhaseInc = pgm_read_byte(formantPos++)*formantScale;
            form2PhaseInc = startForm2PhaseInc = pgm_read_byte(formantPos++)*formantScale;
            form3PhaseInc = startForm3PhaseInc = pgm_read_byte(formantPos++)*formantScale;
            form1Amp = startForm1Amp = pgm_read_byte(formantPos++);
            form2Amp = startForm2Amp = pgm_read_byte(formantPos++);
            form3Amp = startForm3Amp = pgm_read_byte(formantPos++);
            noiseMod = startMod = pgm_read_byte(formantPos++);
            // for (;staticFrames--;)
            // {
        } while(digitalRead(2));

        // tweenFrames currently not doing anything, as far as I can tell...
        /*tweenFrames=map( analogRead(4), 0,1024,0,10);
        tweenFrames=10;
        if (tweenFrames) {
            const uint8_t* formantPos;
            int16_t deltaForm1PhaseInc,deltaForm2PhaseInc,deltaForm3PhaseInc;
            int8_t deltaForm1Amp,deltaForm2Amp,deltaForm3Amp;
            int8_t deltaMod;
            uint8_t nextMod;
            int16_t deltaPitch;
            // tweenFrames--;
            formantPos = formantTable + nextFormant * FORMANT_SZ;
            deltaForm1PhaseInc = pgm_read_byte(formantPos++)*formantScale - startForm1PhaseInc;
            deltaForm2PhaseInc = pgm_read_byte(formantPos++)*formantScale - startForm2PhaseInc;
            deltaForm3PhaseInc = pgm_read_byte(formantPos++)*formantScale - startForm3PhaseInc;
            deltaForm1Amp = pgm_read_byte(formantPos++) - startForm1Amp;
            deltaForm2Amp = pgm_read_byte(formantPos++) - startForm2Amp;
            deltaForm3Amp = pgm_read_byte(formantPos++) - startForm3Amp;

            deltaPitch = nextPitch - startPitch;
            deltaMod = nextMod - startMod;

            for (int i=1; i<=tweenFrames; i++) {
                form1PhaseInc = startForm1PhaseInc + (i*deltaForm1PhaseInc)/tweenFrames;
                form2PhaseInc = startForm2PhaseInc + (i*deltaForm2PhaseInc)/tweenFrames;
                form3PhaseInc = startForm3PhaseInc + (i*deltaForm3PhaseInc)/tweenFrames;
                form1Amp = startForm1Amp + (i*deltaForm1Amp)/tweenFrames;
                form2Amp = startForm2Amp + (i*deltaForm2Amp)/tweenFrames;
                form3Amp = startForm3Amp + (i*deltaForm3Amp)/tweenFrames;
                pitchPhaseInc = startPitch + (i*deltaPitch)/tweenFrames;
                noiseMod = startMod + (i*deltaMod)/tweenFrames;
                // frameTime = map( analogRead(3), 0,1024,60,2 ) ;
                //delay(frameTime);
            }
        }*/
    }
}

SIGNAL(PWM_INTERRUPT) {
    int8_t value;

    // Noise
    static int8_t noise;
    int16_t phaseNoise = noise * noiseMod;
    noise += noise<<2; noise++;

    // Formants
    form1Phase += form1PhaseInc;
    value = pgm_read_byte(sinCalc+(((form1Phase>>8) & 0xf0) | form1Amp));
    form2Phase += form2PhaseInc;
    value += pgm_read_byte(sinCalc+(((form2Phase>>8) & 0xf0) | form2Amp));
    form3Phase += form3PhaseInc;
    value += pgm_read_byte(sqrCalc+(((form3Phase>>8) & 0xf0) | form3Amp));

    // Final sample value
    value = (value * (0xff^(pitchPhase>>8)))>>8;
    pitchPhase += pitchPhaseInc;

    // Reset phase-increment
    if ((pitchPhase+phaseNoise) < pitchPhaseInc) {
        form1Phase = 0;
        form2Phase = 0;
        form3Phase = 0;
    }

    // Write sample value to PWM output
    PWM_VALUE = value + 0x80;
}

//  ########   #######  ##     ## ######## #### ##    ## ########  ######
//  ##     ## ##     ## ##     ##    ##     ##  ###   ## ##       ##    ##
//  ##     ## ##     ## ##     ##    ##     ##  ####  ## ##       ##
//  ########  ##     ## ##     ##    ##     ##  ## ## ## ######    ######
//  ##   ##   ##     ## ##     ##    ##     ##  ##  #### ##             ##
//  ##    ##  ##     ## ##     ##    ##     ##  ##   ### ##       ##    ##
//  ##     ##  #######   #######     ##    #### ##    ## ########  ######

// Initialise audio
void audioOn() {
    #if defined(__AVR_ATmega8__)
    // ATmega8 has different registers
    TCCR2 = _BV(WGM20) | _BV(COM21) | _BV(CS20);
    TIMSK = _BV(TOIE2);
    #elif defined(__AVR_ATmega1280__)
    TCCR3A = _BV(COM3C1) | _BV(WGM30);
    TCCR3B = _BV(CS30);
    TIMSK3 = _BV(TOIE3);
    #else
    // Set up PWM to 31.25kHz, phase accurate
    TCCR2A = _BV(COM2B1) | _BV(WGM20);
    TCCR2B = _BV(CS20);
    TIMSK2 = _BV(TOIE2);
    #endif
}

// Write number (0-9) to display
void display(int n)
{	digitalWrite(LedD, ((n >> 3) & 1) ? HIGH : LOW);
    digitalWrite(LedC, ((n >> 2) & 1) ? HIGH : LOW);
    digitalWrite(LedB, ((n >> 1) & 1) ? HIGH : LOW);
    digitalWrite(LedA, (n & 1) ? HIGH : LOW);
}
