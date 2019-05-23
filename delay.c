#include "msp.h"
#include "my_msp.h"
#include "delay.h"

// Initialize DCO and other clock values
void init_dco(void){
    CS->KEY = CS_KEY_VAL;  // Unlock key
    CS->CTL0 = 0;  // Reset
    CS->CTL0 = CS_CTL0_DCORSEL_3; // 12 MHz
    CS->CTL1 = CS_CTL1_SELA_2 |  // Set ACLK to REFCLK
               CS_CTL1_DIVA__16 |
               CS_CTL1_SELS_3 |  // Set SMCLK to DCO
               CS_CTL1_DIVS__16 | // Divide SMCLK
               CS_CTL1_SELM_3;  // Set MCLK to DCO
    CS->KEY = LOCK_CS_KEY;  // Lock key
}

// Apply safe settings for 48MHz operation
static inline void safe_48(){
    /* Transition to VCORE Level 1: AM0_LDO --> AM1_LDO */
    while ((PCM->CTL1 & PCM_CTL1_PMR_BUSY));
    PCM->CTL0 = PCM_CTL0_KEY_VAL | PCM_CTL0_AMR_1;
    while ((PCM->CTL1 & PCM_CTL1_PMR_BUSY));

    /* Configure Flash wait-state to 1 for both banks 0 & 1 */
    FLCTL->BANK0_RDCTL = (FLCTL->BANK0_RDCTL &
     ~(FLCTL_BANK0_RDCTL_WAIT_MASK)) | FLCTL_BANK0_RDCTL_WAIT_1;
    FLCTL->BANK1_RDCTL = (FLCTL->BANK0_RDCTL &
     ~(FLCTL_BANK1_RDCTL_WAIT_MASK)) | FLCTL_BANK1_RDCTL_WAIT_1;
}

// Set DCO to provide frequency
void set_dco(unsigned int freq){
    CS->KEY = CS_KEY_VAL;  // Unlock key
    CS->CTL0 = 0;  // Reset
    switch(freq){
    case(FREQ_1_5_MHZ):
        CS->CTL0 = CS_CTL0_DCORSEL_0;
        break;
    case(FREQ_3_MHZ):
        CS->CTL0 = CS_CTL0_DCORSEL_1;
        break;
    case(FREQ_6_MHZ):
        CS->CTL0 = CS_CTL0_DCORSEL_2;
        break;
    case(FREQ_12_MHZ):
        CS->CTL0 = CS_CTL0_DCORSEL_3;
        break;
    case(FREQ_24_MHZ):
        CS->CTL0 = CS_CTL0_DCORSEL_4;
        break;
    case(FREQ_48_MHZ):
        safe_48();  // Apply settings for safe 48MHz operation
        CS->CTL0 = CS_CTL0_DCORSEL_5;
        break;
    default:
        // Default to 1.5MHz
        CS->CTL0 = CS_CTL0_DCORSEL_0;
    }
    CS->KEY = LOCK_CS_KEY;
}

// Loop counts for 1ms dealy at given frequencies
// Includes fine tuned inline asm NOPs
static inline int one_msec(const unsigned int freq){
    switch(freq){
    case(FREQ_48_MHZ):
        return 6019;
    case(FREQ_24_MHZ):
        NOP
        NOP
        NOP
        return 3435;
    case(FREQ_12_MHZ):
        NOP
        NOP
        NOP
        NOP
        NOP
        NOP
        return 1714;
    case(FREQ_6_MHZ):
        NOP
        NOP
        NOP
        NOP
        NOP
        NOP
        return 853;
    case(FREQ_3_MHZ):
        NOP
        NOP
        NOP
        return 423;
    default:
        // Default to 1.5MHz
        return 181;
    }
}

// Delay for 1ms at the given frequency
inline delay_one_ms(const unsigned int freq){
     unsigned int j = one_msec(freq);
     while(j-- > 0);
}

// Delay for the given number of ms at the given frequency
inline void delay_ms(unsigned int msec, const unsigned int freq){
    while (msec-- > 0){
        delay_one_ms(freq);
    }
}

inline void delay_ms_auto(unsigned int msec){
    // Option 2 
    // unsigned int freq = CS->CTL0;
    // Option 1
    // unisgned int freq;
    // switch(CS->CTL0){
    //  case(CS_CTL0_DCORSEL_0):
    //      freq =FREQ_1_5_MHZ;
    //      break;
    //  case(CS_CTL0_DCORSEL_1):
    //      freq =FREQ_3_MHZ;
    //      break;
    //  case(CS_CTL0_DCORSEL_2):
    //      freq =FREQ_6_MHZ;
    //      break;
    //  case(CS_CTL0_DCORSEL_3):
    //      freq =FREQ_12_MHZ;
    //      break;
    //  case(CS_CTL0_DCORSEL_4):
    //     freq =FREQ_24_MHZ;
    //      break;
    //  case(CS_CTL0_DCORSEL_5):
    //     freq =FREQ_48_MHZ;
    //      break;
    //  default:
    //      // Default to 1.5MHz
    //      freq = FREQ_1_5_MHZ;
    // Option 3
    unsigned int freq = CS->CTL0;
    delay_ms(msec, freq);
}

// Fine tuned 1 us delay only to be used at 48MHz
// Not calibrated for other frequencies
inline void delay_one_us_at_48(){
    NOP
    NOP
    NOP
    NOP
    NOP
    NOP
    NOP
    NOP
    NOP
    NOP
    NOP
    NOP
    NOP
    NOP
    NOP
    NOP
    NOP
    NOP
    NOP
    NOP
    NOP
}

// Delay for the given number of us 
inline void delay_us(unsigned int usec, const unsigned int freq){
    while(usec-- > 0){
        delay_one_us_at_48();
    }
}
