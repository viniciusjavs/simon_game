#include "tm4c123gh6pm.h"
#include <algorithm>
#include <array>
#include <random>

// Functions prototypes
void PortAInit();
void PortEInit();
void PortFInit();
void PLL_Init();
void SysTick_Init();
void SysTick_Wait1ms(unsigned long);
unsigned long SysTick_Get_Seed();
void SysTick_Get_Sound(unsigned long);
void Blink_Led(volatile unsigned long *, unsigned long);

// Colors
enum Color { R, Y, G, B, sz };

// Sounds
constexpr unsigned long A5 = 90909;   // Red 880 Hz
constexpr unsigned long CS5 = 144309; // Yellow 554.365 Hz
constexpr unsigned long E4 = 242698;  // Green 329.628 Hz
constexpr unsigned long E5 = 121349;  // Blue 659.255 Hz

int main(void) {
  // Setup Device
  PLL_Init();
  SysTick_Init();
  PortAInit();
  PortEInit();
  PortFInit();

  // Game variables
  constexpr unsigned short max = 39; // max sequence
  unsigned int count = 1, cur = 0;
  bool game_over = false, next = true;

  // Get a seed
  std::minstd_rand engine(SysTick_Get_Seed());
  // Choose a random number between 0 and Color length - 1
  std::uniform_int_distribution rand(0, Color::sz - 1);
  // Fill array with random data
  const auto seq = [&] {
    std::array<int, max> seq;
    std::generate(seq.begin(), seq.end(), [&]() { return rand(engine); });
    return seq;
  }();

  // Interfaces
  // Port A
  // output: PA2
  // Port E
  // inputs: PE0, PE2, PE4
  // outputs: PE1 PE3 PE5
  // Port F
  // input: PF0
  // output: PF2

  while (!game_over) {
    if (next) {
      SysTick_Wait1ms(500);
      for (unsigned int i = 0; i < count; ++i) {
        switch (seq[i]) {
        case Color::R:
          SysTick_Get_Sound(A5);
          Blink_Led(&GPIO_PORTE_DATA_R, 0x02);
          break;
        case Color::Y:
          SysTick_Get_Sound(CS5);
          Blink_Led(&GPIO_PORTE_DATA_R, 0x08);
          break;
        case Color::G:
          SysTick_Get_Sound(E4);
          Blink_Led(&GPIO_PORTE_DATA_R, 0x20);
          break;
        case Color::B:
          SysTick_Get_Sound(E5);
          Blink_Led(&GPIO_PORTF_DATA_R, 0x04);
          break;
        default:
          return 1;
        }
      }
      next = false;
    }

    // Read PE0 (Red SW)
    if (GPIO_PORTE_DATA_R & 0x01) {
      if (Color::R == seq[cur]) {
        SysTick_Get_Sound(A5);
        Blink_Led(&GPIO_PORTE_DATA_R, 0x02);
        ++cur;
      } else {
        game_over = true;
        break;
      }
    }

    // Read PE2 (Yellow SW)
    else if (GPIO_PORTE_DATA_R & 0x04) {
      if (Color::Y == seq[cur]) {
        SysTick_Get_Sound(CS5);
        Blink_Led(&GPIO_PORTE_DATA_R, 0x08);
        ++cur;
      } else {
        game_over = true;
        break;
      }
    }

    // Read PE4 (Green SW)
    else if (GPIO_PORTE_DATA_R & 0x10) {
      if (Color::G == seq[cur]) {
        SysTick_Get_Sound(E4);
        Blink_Led(&GPIO_PORTE_DATA_R, 0x20);
        ++cur;
      } else {
        game_over = true;
        break;
      }
    }

    // Read PF0 (onboard SW1). Reverse logic
    else if (!(GPIO_PORTF_DATA_R & 0x01)) {
      if (Color::B == seq[cur]) {
        SysTick_Get_Sound(E5);
        Blink_Led(&GPIO_PORTF_DATA_R, 0x04);
        ++cur;
      } else {
        game_over = true;
        break;
      }
    }

    // seq reached
    if (count == cur) {
      if (count++ == seq.size()) { // end game
        game_over = true;
        break;
      }
      cur = 0;
      next = true;
    }
  }
}

// Setup port A
void PortAInit(void) {
  SYSCTL_RCGC2_R |= 0x01;           // activate port A
  GPIO_PORTA_AMSEL_R &= ~0x04;      // no analog PA2
  GPIO_PORTA_PCTL_R &= ~0x00000F00; // regular function PA2
  GPIO_PORTA_DIR_R |= 0x04;         // PA2 output
  GPIO_PORTA_DR8R_R |= 0x04;        // can drive up to 8mA out
  GPIO_PORTA_AFSEL_R &= ~0x04;      // disable alt funct on PA2
  GPIO_PORTA_DEN_R |= 0x04;         // enable digital I/O on PA2
}

// Setup port E
void PortEInit(void) {
  SYSCTL_RCGC2_R |= 0x10;           // Port E clock
  GPIO_PORTE_DIR_R |= 0x2A;         // PE1, PE3, PE5 output
  GPIO_PORTE_DIR_R &= ~0x15;        // PE0, PE2, PE4 input
  GPIO_PORTE_AFSEL_R &= ~0x3F;      // not alternative
  GPIO_PORTE_AMSEL_R &= ~0x3F;      // no analog
  GPIO_PORTE_PCTL_R &= ~0x00FFFFFF; // bits for PE5,PE4,PE3,PE2,PE1,PE0
  GPIO_PORTE_DEN_R |= 0x3F;         // enable PE5,PE4,PE3,PE2,PE1,PE0
  GPIO_PORTE_PUR_R |= 0;            // disable PUR
}

// Setup port F
void PortFInit(void) {
  SYSCTL_RCGC2_R |= 0x00000020;   // F clock
  GPIO_PORTF_LOCK_R = 0x4C4F434B; // Unlock Port F
  GPIO_PORTF_CR_R = 0x1F;         // Allow changes to PF4-0
  GPIO_PORTF_AMSEL_R = 0x00;      // Disable analog functions
  GPIO_PORTF_PCTL_R = 0x00000000; // GPIO clear bit PCTL
  GPIO_PORTF_DIR_R |= 0x04;       // PF2 output
  GPIO_PORTF_DIR_R &= ~0x01;      // PF0 input
  GPIO_PORTF_AFSEL_R = 0x00;      // Clear alt functions
  GPIO_PORTF_PUR_R |= 0x01;       // Enable pull-up resistor on PF0
  GPIO_PORTF_DEN_R |= 0x05;       // Enable digital pin PF2, PF0
}

// Using the 400MHz PLL we get the bus frequency by dividing
// 400MHz / (SYSDIV2+1). Therefore, 400MHz/(4+1) = 80 MHz bus frequency
#define SYSDIV2 4
// Gets clock from PLL
void PLL_Init(void) {
  SYSCTL_RCC2_R |= SYSCTL_RCC2_USERCC2;    // Use RCC2 for advanced features
  SYSCTL_RCC2_R |= SYSCTL_RCC2_BYPASS2;    // Bypass PLL during initialization
  SYSCTL_RCC_R &= ~SYSCTL_RCC_XTAL_M;      // Clear XTAL field
  SYSCTL_RCC_R += SYSCTL_RCC_XTAL_16MHZ;   // Use 16Mhz crystal
  SYSCTL_RCC2_R &= ~SYSCTL_RCC2_OSCSRC2_M; // Clear oscillator source field
  SYSCTL_RCC2_R += SYSCTL_RCC2_OSCSRC2_MO; // Use main oscillator source field
  SYSCTL_RCC2_R &= ~SYSCTL_RCC2_PWRDN2;    // Activate PLL by clearing PWRDN

  // As explained above we use 400MHz PLL and specified SYSDIV2 for 80MHz clock
  SYSCTL_RCC2_R |= SYSCTL_RCC2_DIV400; // Use 400MHz PLL
  // Clear system clock divider field then configure 80Mhz clock
  SYSCTL_RCC2_R = (SYSCTL_RCC2_R & ~0x1FC00000) + (SYSDIV2 << 22);
  // Wait for the PLL to lock by polling PLLLRIS
  while ((SYSCTL_RIS_R & SYSCTL_RIS_PLLLRIS) == 0) {
  };
  SYSCTL_RCC2_R &= ~SYSCTL_RCC2_BYPASS2; // Enable PLL by clearing BYPASS
}

void SysTick_Init(void) {
  NVIC_ST_CTRL_R = 0;    // Disable SysTick during setup
  NVIC_ST_CURRENT_R = 0; // Any write clears current
  // Enable SysTick with core clock, i.e. NVIC_ST_CTRL_R = 0x00000005
  NVIC_ST_CTRL_R = NVIC_ST_CTRL_ENABLE + NVIC_ST_CTRL_CLK_SRC;
}

// Uses SysTick to count down 1ms*(passed value)
void SysTick_Wait1ms(unsigned long ms) {
  // Clock is set to 80MHz, each Systick takes 1/80MHz = 12.5ns
  // To get 1ms count down delay, take (1ms)/(12.5ns)
  unsigned long delay = 80000;

  for (unsigned long i = 0; i < ms; ++i) {
    NVIC_ST_RELOAD_R = delay - 1; // Reload value is the number of counts to
                                  // wait
    NVIC_ST_CURRENT_R = 0;        // Clears current
    // Bit 16 of STCTRL is set to 1 if SysTick timer counts down to zero
    while ((NVIC_ST_CTRL_R & 0x00010000) == 0) {
    }
  }
}

// Uses SysTick to retrieve a seed based on user interaction
unsigned long SysTick_Get_Seed() {
  // Clock is set to 80MHz, each Systick takes 1/80MHz = 12.5ns
  // To get 1ms count down delay, take (1ms)/(12.5ns)
  unsigned long delay = 80000;

  while (true) {
    NVIC_ST_RELOAD_R = delay - 1; // Reload value is the number of counts to
                                  // wait
    NVIC_ST_CURRENT_R = 0;        // Clears current
    // Bit 16 of STCTRL is set to 1 if SysTick timer counts down to zero
    while ((NVIC_ST_CTRL_R & 0x00010000) == 0) {
      // at least one input pressed (PE0, PE2, PE4 or PF1)
      if (GPIO_PORTE_DATA_R & 0x15 || !(GPIO_PORTF_DATA_R & 0x01)) {
        return NVIC_ST_CURRENT_R;
      }
    }
  }
}

// Uses SysTick to play a squarewave sound
void SysTick_Get_Sound(unsigned long period) {
  // Clock is set to 80MHz, each Systick takes 1/80MHz = 12.5ns
  // To get 1/Hz s count down delay, take (1/Hz s)/(12.5ns)
  for (unsigned long i = 0, j = 2 * 80; i < j; ++i) {
    // Reload value is the number of counts to wait
    NVIC_ST_RELOAD_R = period - 1;
    NVIC_ST_CURRENT_R = 0; // Clears current
    // Bit 16 of STCTRL is set to 1 if SysTick timer counts down to zero
    while ((NVIC_ST_CTRL_R & 0x00010000) == 0) {
    }
    GPIO_PORTA_DATA_R ^= 0x04; // toggle PA2;
  }
}

void Blink_Led(volatile unsigned long *port, unsigned long bit) {
  for (int i = 0, j = 8; i < j; ++i) {
    *port ^= bit;
    SysTick_Wait1ms(100);
  }
}
