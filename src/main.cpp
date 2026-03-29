//此代码实现一个计时器功能，利用PC口和PD口通过矩阵扫描的方式控制35个LED，这些LED首先全亮，然后每15分钟熄灭一个，
//直到8小时45分钟（35*15min=525min=8.75h)后全部熄灭，15分钟的标志位会定期存入EEPROM中，因此即使中途断电，
//重新上电后也能继续计数
//短按：开始计时，长按: 15分钟标志位归零

//This code implements a timer function that uses the PC and PD ports to control 35 LEDs via matrix scanning. 
//The LEDs are initially all lit, then one is switched off every 15 minutes, until all are off after 8 hours 
//and 45 minutes (35 × 15 min = 525 min = 8.75 h). The 15-minute flag is periodically saved to EEPROM, so even 
//if the power is cut off midway, the count will resume upon power-up. Short press: Start timer; Long press: 
//Reset the 15-minute flag

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/eeprom.h>
#include <math.h>

//external crystal: 8MHz
//#define F_CPU 8000000UL

const uint32_t AddrEEPROM = 0x0040;

volatile uint16_t t1_counter = 0; //counter for 1sec of Timer1
volatile uint8_t t0_counter = 0; //counter of Timer0
volatile uint8_t blink_flag = 0; //for blinking of the LEDs
volatile uint8_t _15min_flag = 0; //counter for every 15min

//The ":15min_flag" can be read or written from/into EEPROM
uint8_t EEPROM_read (uint32_t address)
{
     while(EECR & (1<<EEPE));
     EEAR = address; 
     EECR |= (1<<EERE);
     return EEDR;
}
void EEPROM_write(uint32_t Address, uint8_t Data)
{
     while(EECR & (1<<EEPE));
     EEAR = Address;EEDR = Data;
     EECR |= (1<<EEMPE); 
     EECR |= (1<<EEPE);
}

//Define the display contents as 2D-arrays. 1 for on, 0 for off
//Since RAM size is limited, these arrays are stored in flash via the keyword "PROGMEM"
const uint8_t ledMatrixAllOn[5][7] PROGMEM = 
{
    {1, 1, 1, 1, 1, 1, 1},{1, 1, 1, 1, 1, 1, 1},{1, 1, 1, 1, 1, 1, 1},{1, 1, 1, 1, 1, 1, 1},{1, 1, 1, 1, 1, 1, 1}
};

const uint8_t ledMatrixAllOff[5][7] PROGMEM = 
{
    {0, 0, 0, 0, 0, 0, 0},{0, 0, 0, 0, 0, 0, 0},{0, 0, 0, 0, 0, 0, 0},{0, 0, 0, 0, 0, 0, 0},{0, 0, 0, 0, 0, 0, 0}
};
const uint8_t ledMatrixLetterLA5[5][7] PROGMEM = //The last letter A starts to go out
{
    {1, 1, 1, 1, 1, 0, 1},{1, 1, 1, 1, 1, 1, 1},{1, 1, 1, 1, 1, 1, 1},{1, 1, 1, 1, 1, 1, 1},{1, 1, 1, 1, 1, 1, 1}
};

const uint8_t ledMatrixLetterLA4[5][7] PROGMEM = 
{
    {1, 1, 1, 0, 1, 0, 1},{1, 1, 1, 1, 1, 1, 1},{1, 1, 1, 1, 1, 1, 1},{1, 1, 1, 1, 1, 1, 1},{1, 1, 1, 1, 1, 1, 1}
};
const uint8_t ledMatrixLetterLA3[5][7] PROGMEM = 
{
    {1, 1, 1, 0, 0, 0, 1},{1, 1, 1, 1, 1, 1, 1},{1, 1, 1, 1, 1, 1, 1},{1, 1, 1, 1, 1, 1, 1},{1, 1, 1, 1, 1, 1, 1}
};
const uint8_t ledMatrixLetterLA2[5][7] PROGMEM = 
{
    {1, 1, 1, 0, 0, 0, 0},{1, 1, 1, 1, 1, 1, 1},{1, 1, 1, 1, 1, 1, 1},{1, 1, 1, 1, 1, 1, 1},{1, 1, 1, 1, 1, 1, 1}
};
const uint8_t ledMatrixLetterLA1[5][7] PROGMEM = 
{
    {1, 1, 0, 0, 0, 0, 0},{1, 1, 1, 1, 1, 1, 1},{1, 1, 1, 1, 1, 1, 1},{1, 1, 1, 1, 1, 1, 1},{1, 1, 1, 1, 1, 1, 1}
};
const uint8_t ledMatrixLetterLA0[5][7] PROGMEM = //The last letter A is completely out
{
    {1, 0, 0, 0, 0, 0, 0},{1, 1, 1, 1, 1, 1, 1},{1, 1, 1, 1, 1, 1, 1},{1, 1, 1, 1, 1, 1, 1},{1, 1, 1, 1, 1, 1, 1}
};
const uint8_t ledMatrixLetterM6[5][7] PROGMEM = //The letter M starts to go out
{
    {1, 0, 0, 0, 0, 0, 0},{1, 1, 1, 1, 1, 0, 1},{1, 1, 1, 1, 1, 1, 1},{1, 1, 1, 1, 1, 1, 1},{1, 1, 1, 1, 1, 1, 1}
};
const uint8_t ledMatrixLetterM5[5][7] PROGMEM = 
{
    {1, 0, 0, 0, 0, 0, 0},{1, 1, 1, 0, 1, 0, 1},{1, 1, 1, 1, 1, 1, 1},{1, 1, 1, 1, 1, 1, 1},{1, 1, 1, 1, 1, 1, 1}
};
const uint8_t ledMatrixLetterM4[5][7] PROGMEM = 
{
    {1, 0, 0, 0, 0, 0, 0},{1, 1, 1, 0, 0, 0, 1},{1, 1, 1, 1, 1, 1, 1},{1, 1, 1, 1, 1, 1, 1},{1, 1, 1, 1, 1, 1, 1}
};
const uint8_t ledMatrixLetterM3[5][7] PROGMEM = 
{
    {1, 0, 0, 0, 0, 0, 0},{1, 1, 1, 0, 0, 0, 0},{1, 1, 1, 1, 1, 1, 1},{1, 1, 1, 1, 1, 1, 1},{1, 1, 1, 1, 1, 1, 1}
};
const uint8_t ledMatrixLetterM2[5][7] PROGMEM = 
{
    {1, 0, 0, 0, 0, 0, 0},{1, 1, 0, 0, 0, 0, 0},{1, 1, 1, 1, 1, 1, 1},{1, 1, 1, 1, 1, 1, 1},{1, 1, 1, 1, 1, 1, 1}
};
const uint8_t ledMatrixLetterM1[5][7] PROGMEM = 
{
    {0, 0, 0, 0, 0, 0, 0},{1, 1, 0, 0, 0, 0, 0},{1, 1, 1, 1, 1, 1, 1},{1, 1, 1, 1, 1, 1, 1},{1, 1, 1, 1, 1, 1, 1}
};
const uint8_t ledMatrixLetterM0[5][7] PROGMEM = //The letter M is completely out
{
    {0, 0, 0, 0, 0, 0, 0},{1, 0, 0, 0, 0, 0, 0},{1, 1, 1, 1, 1, 1, 1},{1, 1, 1, 1, 1, 1, 1},{1, 1, 1, 1, 1, 1, 1}
};
const uint8_t ledMatrixLetterFA5[5][7] PROGMEM = //The first letter A starts to go out
{
    {0, 0, 0, 0, 0, 0, 0},{1, 0, 0, 0, 0, 0, 0},{1, 1, 1, 1, 1, 1, 0},{1, 1, 1, 1, 1, 1, 1},{1, 1, 1, 1, 1, 1, 1}
};
const uint8_t ledMatrixLetterFA4[5][7] PROGMEM = 
{
    {0, 0, 0, 0, 0, 0, 0},{1, 0, 0, 0, 0, 0, 0},{1, 1, 1, 1, 0, 1, 0},{1, 1, 1, 1, 1, 1, 1},{1, 1, 1, 1, 1, 1, 1}
};
const uint8_t ledMatrixLetterFA3[5][7] PROGMEM = 
{
    {0, 0, 0, 0, 0, 0, 0},{1, 0, 0, 0, 0, 0, 0},{1, 1, 1, 1, 0, 0, 0},{1, 1, 1, 1, 1, 1, 1},{1, 1, 1, 1, 1, 1, 1}
};
const uint8_t ledMatrixLetterFA2[5][7] PROGMEM = 
{
    {0, 0, 0, 0, 0, 0, 0},{0, 0, 0, 0, 0, 0, 0},{1, 1, 1, 1, 0, 0, 0},{1, 1, 1, 1, 1, 1, 1},{1, 1, 1, 1, 1, 1, 1}
};
const uint8_t ledMatrixLetterFA1[5][7] PROGMEM =
{
    {0, 0, 0, 0, 0, 0, 0},{0, 0, 0, 0, 0, 0, 0},{1, 1, 1, 0, 0, 0, 0},{1, 1, 1, 1, 1, 1, 1},{1, 1, 1, 1, 1, 1, 1}
};
const uint8_t ledMatrixLetterFA0[5][7] PROGMEM = //The first letter A is completely out
{
    {0, 0, 0, 0, 0, 0, 0},{0, 0, 0, 0, 0, 0, 0},{1, 1, 0, 0, 0, 0, 0},{1, 1, 1, 1, 1, 1, 1},{1, 1, 1, 1, 1, 1, 1}
};
const uint8_t ledMatrixLetterR4[5][7] PROGMEM = //The letter R starts to go out
{
    {0, 0, 0, 0, 0, 0, 0},{0, 0, 0, 0, 0, 0, 0},{1, 0, 0, 0, 0, 0, 0},{1, 1, 1, 1, 1, 1, 1},{1, 1, 1, 1, 1, 1, 1}
};
const uint8_t ledMatrixLetterR3[5][7] PROGMEM = 
{
    {0, 0, 0, 0, 0, 0, 0},{0, 0, 0, 0, 0, 0, 0},{0, 0, 0, 0, 0, 0, 0},{1, 1, 1, 1, 1, 1, 1},{1, 1, 1, 1, 1, 1, 1}
};
const uint8_t ledMatrixLetterR2[5][7] PROGMEM = 
{
    {0, 0, 0, 0, 0, 0, 0},{0, 0, 0, 0, 0, 0, 0},{0, 0, 0, 0, 0, 0, 0},{1, 1, 1, 1, 1, 1, 0},{1, 1, 1, 1, 1, 1, 1}
};
const uint8_t ledMatrixLetterR1[5][7] PROGMEM = 
{
    {0, 0, 0, 0, 0, 0, 0},{0, 0, 0, 0, 0, 0, 0},{0, 0, 0, 0, 0, 0, 0},{1, 1, 1, 1, 1, 0, 0},{1, 1, 1, 1, 1, 1, 1}
};
const uint8_t ledMatrixLetterR0[5][7] PROGMEM = //The letter R is completely out
{
    {0, 0, 0, 0, 0, 0, 0},{0, 0, 0, 0, 0, 0, 0},{0, 0, 0, 0, 0, 0, 0},{1, 1, 1, 1, 0, 0, 0},{1, 1, 1, 1, 1, 1, 1}
};
const uint8_t ledMatrixLetterT4[5][7] PROGMEM = //The letter T starts to go out
{
    {0, 0, 0, 0, 0, 0, 0},{0, 0, 0, 0, 0, 0, 0},{0, 0, 0, 0, 0, 0, 0},{1, 1, 1, 0, 0, 0, 0},{1, 1, 1, 1, 1, 1, 1}
};
const uint8_t ledMatrixLetterT3[5][7] PROGMEM = 
{
    {0, 0, 0, 0, 0, 0, 0},{0, 0, 0, 0, 0, 0, 0},{0, 0, 0, 0, 0, 0, 0},{1, 1, 0, 0, 0, 0, 0},{1, 1, 1, 1, 1, 1, 1}
};
const uint8_t ledMatrixLetterT2[5][7] PROGMEM = 
{
    {0, 0, 0, 0, 0, 0, 0},{0, 0, 0, 0, 0, 0, 0},{0, 0, 0, 0, 0, 0, 0},{1, 0, 0, 0, 0, 0, 0},{1, 1, 1, 1, 1, 1, 1}
};
const uint8_t ledMatrixLetterT1[5][7] PROGMEM = 
{
    {0, 0, 0, 0, 0, 0, 0},{0, 0, 0, 0, 0, 0, 0},{0, 0, 0, 0, 0, 0, 0},{0, 0, 0, 0, 0, 0, 0},{1, 1, 1, 1, 1, 1, 1}
};
const uint8_t ledMatrixLetterT0[5][7] PROGMEM = //The letter T is completely out
{
    {0, 0, 0, 0, 0, 0, 0},{0, 0, 0, 0, 0, 0, 0},{0, 0, 0, 0, 0, 0, 0},{0, 0, 0, 0, 0, 0, 0},{1, 1, 1, 1, 1, 1, 0}
};
const uint8_t ledMatrixLetterS5[5][7] PROGMEM = //The letter S starts to go out
{
    {0, 0, 0, 0, 0, 0, 0},{0, 0, 0, 0, 0, 0, 0},{0, 0, 0, 0, 0, 0, 0},{0, 0, 0, 0, 0, 0, 0},{1, 1, 1, 1, 1, 0, 0}
};
const uint8_t ledMatrixLetterS4[5][7] PROGMEM = 
{
    {0, 0, 0, 0, 0, 0, 0},{0, 0, 0, 0, 0, 0, 0},{0, 0, 0, 0, 0, 0, 0},{0, 0, 0, 0, 0, 0, 0},{1, 1, 1, 1, 0, 0, 0}
};
const uint8_t ledMatrixLetterS3[5][7] PROGMEM = 
{
    {0, 0, 0, 0, 0, 0, 0},{0, 0, 0, 0, 0, 0, 0},{0, 0, 0, 0, 0, 0, 0},{0, 0, 0, 0, 0, 0, 0},{1, 1, 1, 0, 0, 0, 0}
};
const uint8_t ledMatrixLetterS2[5][7] PROGMEM = 
{
    {0, 0, 0, 0, 0, 0, 0},{0, 0, 0, 0, 0, 0, 0},{0, 0, 0, 0, 0, 0, 0},{0, 0, 0, 0, 0, 0, 0},{1, 1, 0, 0, 0, 0, 0}
};
const uint8_t ledMatrixLetterS1[5][7] PROGMEM = 
{
    {0, 0, 0, 0, 0, 0, 0},{0, 0, 0, 0, 0, 0, 0},{0, 0, 0, 0, 0, 0, 0},{0, 0, 0, 0, 0, 0, 0},{1, 0, 0, 0, 0, 0, 0}
};
const uint8_t ledMatrixLetterS0[5][7] PROGMEM = //The letter S is completely out
{
    {0, 0, 0, 0, 0, 0, 0},{0, 0, 0, 0, 0, 0, 0},{0, 0, 0, 0, 0, 0, 0},{0, 0, 0, 0, 0, 0, 0},{0, 0, 0, 0, 0, 0, 0}
};

void timer0_init() 
{
    //Prescaler 1024：CS02=1, CS01=0, CS00=1
    TCCR0B |= (1 << CS02) | (1 << CS00);
    //Enable Timer0 overflow interrupt
    TIMSK0 |= (1 << TOIE0);
    //Initialize the counter of Timer0
    TCNT0 = 0;
}

void timer1_init() 
{
    //Prescaler 1024: CS12=1, CS11=0, CS10=1
    TCCR1B |= (1 << CS12) | (1 << CS10);
    //Initialize counter of Timer1, for 1s overflow
    TCNT1 = 65536 - 62500;  //count to 7813
    //Disable the overflow interrupt
    TIMSK1 &= ~(1 << TOIE1);
}

//Initialize the GPIOs
void initIO() 
{
    //PC0-PC4 as outputs for the rows
    DDRC |= 0x1F;
    //PD0-PD6 as outputs for the columns
    DDRD |= 0x7F; 
    //PB0 as input for the button
    DDRB &= ~(1 << PB0);

    //Initial: all rows closed (low level) and all columns closed (high level)
    PORTC &= ~0x1F;
    PORTD |= 0x7F;
    PORTB |= (1 << PB0); //Pull-up res
}

ISR(TIMER0_OVF_vect) 
{
    t0_counter++;
    if (t0_counter >= 50) 
    {
        TCNT0 = 256 - 66; //500ms
        t0_counter = 0;
        blink_flag = ~blink_flag;
        if(_15min_flag == 0) //
        {
            if(blink_flag == 0)
            {
                PORTC |= 0x1f; //All LEDs on
                PORTD &= ~0x7f;
            }
            else
            {
                PORTC &= ~0x1f; //All LEDs off
                PORTD |= 0x7f;
            }
        }
    }
}

void SaveTimeToEEPROM(uint8_t time)
{
    EEPROM_write(AddrEEPROM, time);
}

uint8_t ReadSavedTimeFromEEPROM(uint32_t addr)
{
    return EEPROM_read(addr);
}

ISR(TIMER1_OVF_vect) 
{
    TCNT1 = 65536 - 7813; //Reset counter value to reach an 1s-interrupt (F_CPU = 8MHz)
    t1_counter += 1;
    if(t1_counter > 900) //900s = 15min 900
    {
        t1_counter = 0;
        _15min_flag++; //Increase by 1 every 15 minutes
        SaveTimeToEEPROM(_15min_flag);
        if(_15min_flag > 36) 
            _15min_flag = 0;
    }   
}

void CheckKeyPress()
{
    if (!(PINB & (1 << PB0)))
    {
        _delay_ms(20);  //anti-shake
        if (!(PINB & (1 << PB0)))
        {
            uint8_t i = 0;
            while (!(PINB & (1 << PB0)) && i < 100) //detect for 1s（100 * 10ms）
            {
                _delay_ms(10);
                i++;
            }
            if (i >= 100)  //long press
            {
                _15min_flag = 1;
            }
            else  //short press
            {
                _15min_flag = ReadSavedTimeFromEEPROM(AddrEEPROM);
            }
            //waiting for release...
            while (!(PINB & (1 << PB0)));
            _delay_ms(20);
            TIMSK1 |= (1 << TOIE1); //enable the Timer1
        }
    }
    
}

//Update the matrix display via scanning
void updateMatrix(const uint8_t Matrix[5][7]) 
{
    for (uint8_t row = 0; row < 5; row++) 
    {
        //Turn on the current row
        PORTC = (PORTC & ~0x1F) | (1 << row);  //only light up the current row

        //Setup the columns
        for (uint8_t col = 0; col < 7; col++) 
        {
            if (pgm_read_byte(&(Matrix[row][col]))) //Use the function "pgm_read_byte" to access the arrays in flash
            {
                //If the led of this position should be lighted up, set the corresponding column to low
                PORTD &= ~(1 << col);
            } else 
            {
                //If it shouldn't be lighted up, keep the colunm as high level
                PORTD |= (1 << col);
            }
        }
        //Delay for some times
        _delay_ms(2);
        //Turn off all columns
        PORTD |= 0x7F;
    }
}

int main(void) 
{
    initIO();
    timer0_init();
    timer1_init();
    SREG |= 0x80; //Enable global interrupt
    while (1) 
    {
        CheckKeyPress(); //wait for the key input
        switch (_15min_flag)
        {
            case 1: if(blink_flag == 0) updateMatrix(ledMatrixAllOn); else updateMatrix(ledMatrixLetterLA5);break;
            case 2: if(blink_flag == 0) updateMatrix(ledMatrixLetterLA5); else updateMatrix(ledMatrixLetterLA4);break;
            case 3: if(blink_flag == 0) updateMatrix(ledMatrixLetterLA4); else updateMatrix(ledMatrixLetterLA3);break;
            case 4: if(blink_flag == 0) updateMatrix(ledMatrixLetterLA3); else updateMatrix(ledMatrixLetterLA2);break;
            case 5: if(blink_flag == 0) updateMatrix(ledMatrixLetterLA2); else updateMatrix(ledMatrixLetterLA1);break;
            case 6: if(blink_flag == 0) updateMatrix(ledMatrixLetterLA1); else updateMatrix(ledMatrixLetterLA0);break;
            case 7: if(blink_flag == 0) updateMatrix(ledMatrixLetterLA0); else updateMatrix(ledMatrixLetterM6);break;
            case 8: if(blink_flag == 0) updateMatrix(ledMatrixLetterM6); else updateMatrix(ledMatrixLetterM5);break;
            case 9: if(blink_flag == 0) updateMatrix(ledMatrixLetterM5); else updateMatrix(ledMatrixLetterM4);break;
            case 10: if(blink_flag == 0) updateMatrix(ledMatrixLetterM4); else updateMatrix(ledMatrixLetterM3);break;
            case 11: if(blink_flag == 0) updateMatrix(ledMatrixLetterM3); else updateMatrix(ledMatrixLetterM2);break;
            case 12: if(blink_flag == 0) updateMatrix(ledMatrixLetterM2); else updateMatrix(ledMatrixLetterM1);break;
            case 13: if(blink_flag == 0) updateMatrix(ledMatrixLetterM1); else updateMatrix(ledMatrixLetterM0);break;
            case 14: if(blink_flag == 0) updateMatrix(ledMatrixLetterM0); else updateMatrix(ledMatrixLetterFA5);break;
            case 15: if(blink_flag == 0) updateMatrix(ledMatrixLetterFA5); else updateMatrix(ledMatrixLetterFA4);break;
            case 16: if(blink_flag == 0) updateMatrix(ledMatrixLetterFA4); else updateMatrix(ledMatrixLetterFA3);break;
            case 17: if(blink_flag == 0) updateMatrix(ledMatrixLetterFA3); else updateMatrix(ledMatrixLetterFA2);break;
            case 18: if(blink_flag == 0) updateMatrix(ledMatrixLetterFA2); else updateMatrix(ledMatrixLetterFA1);break;
            case 19: if(blink_flag == 0) updateMatrix(ledMatrixLetterFA1); else updateMatrix(ledMatrixLetterFA0);break;
            case 20: if(blink_flag == 0) updateMatrix(ledMatrixLetterFA0); else updateMatrix(ledMatrixLetterR4);break;
            case 21: if(blink_flag == 0) updateMatrix(ledMatrixLetterR4); else updateMatrix(ledMatrixLetterR3);break;
            case 22: if(blink_flag == 0) updateMatrix(ledMatrixLetterR3); else updateMatrix(ledMatrixLetterR2);break;
            case 23: if(blink_flag == 0) updateMatrix(ledMatrixLetterR2); else updateMatrix(ledMatrixLetterR1);break;
            case 24: if(blink_flag == 0) updateMatrix(ledMatrixLetterR1); else updateMatrix(ledMatrixLetterR0);break;
            case 25: if(blink_flag == 0) updateMatrix(ledMatrixLetterR0); else updateMatrix(ledMatrixLetterT4);break;
            case 26: if(blink_flag == 0) updateMatrix(ledMatrixLetterT4); else updateMatrix(ledMatrixLetterT3);break;
            case 27: if(blink_flag == 0) updateMatrix(ledMatrixLetterT3); else updateMatrix(ledMatrixLetterT2);break;
            case 28: if(blink_flag == 0) updateMatrix(ledMatrixLetterT2); else updateMatrix(ledMatrixLetterT1);break;
            case 29: if(blink_flag == 0) updateMatrix(ledMatrixLetterT1); else updateMatrix(ledMatrixLetterT0);break;
            case 30: if(blink_flag == 0) updateMatrix(ledMatrixLetterT0); else updateMatrix(ledMatrixLetterS5);break;
            case 31: if(blink_flag == 0) updateMatrix(ledMatrixLetterS5); else updateMatrix(ledMatrixLetterS4);break;
            case 32: if(blink_flag == 0) updateMatrix(ledMatrixLetterS4); else updateMatrix(ledMatrixLetterS3);break;
            case 33: if(blink_flag == 0) updateMatrix(ledMatrixLetterS3); else updateMatrix(ledMatrixLetterS2);break;
            case 34: if(blink_flag == 0) updateMatrix(ledMatrixLetterS2); else updateMatrix(ledMatrixLetterS1);break;
            case 35: if(blink_flag == 0) updateMatrix(ledMatrixLetterS1); else updateMatrix(ledMatrixLetterS0);break;
        }
    }
    return 0;
}
