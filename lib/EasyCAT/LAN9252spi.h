#ifndef LAN9252_SPI_H
#define LAN9252_SPI_H

#include <stdint.h>
#include <Arduino.h> 
#include <SPI.h> 

#if defined(ARDUINO_ARCH_AVR) 
  #define SpiSpeed         8000000

#elif defined (STM32F4)  

  #define SpiSpeed        42000000

#elif defined (ARDUINO_ARCH_ESP32)  
  #define SpiSpeed        19000000
#else
  #error "Architecture not supported"
#endif

extern SPIClass SPI_H;

void SPISetup(unsigned char scsPin);
void SPIEnd();

void SPIWriteRegisterDirect(unsigned short address, unsigned long dataOut);
unsigned long SPIReadRegisterDirect(unsigned short address, unsigned char Len);

void SPIWriteRegisterIndirect(unsigned long sataOut, unsigned short address, unsigned char Len);
unsigned long SPIReadRegisterIndirect(unsigned short address, unsigned char Len); 

void SPIReadProcRamFifo();
void SPIWriteProcRamFifo();

//-------------------------------------------------------------------------------------------------    
    
    
#if defined(ARDUINO_ARCH_AVR)                     // -- AVR architecture (Uno - Mega) ---------

inline static void SPI_TransferTx(unsigned char Data) {                             \
                                                        SPDR = Data;                  \
                                                        asm volatile("nop");        
                                                        while (!(SPSR & _BV(SPIF))) ; \
                                                        };               
    
inline static void SPI_TransferTxLast(unsigned char Data) {                         \
                                                        SPDR = Data;                  \
                                                        asm volatile("nop");                                                                        
                                                        while (!(SPSR & _BV(SPIF))) ; \
                                                        };         
    
inline static unsigned char SPI_TransferRx(unsigned char Data) {                      \
                                                        SPDR = Data;                  \
                                                        asm volatile("nop");          
                                                        while (!(SPSR & _BV(SPIF))) ; \
                                                        return SPDR; };    
    
#else                                             //-- standard transfer for others architectures

void SPI_TransferTx            (unsigned char Data);    
void SPI_TransferTxLast        (unsigned char Data); 
unsigned char SPI_TransferRx   (unsigned char Data); 

#endif


#endif // LAN9252_SPI_H