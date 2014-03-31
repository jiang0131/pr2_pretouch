; **************************************************************************************
; Pressure sensor hardware 
; **************************************************************************************
; Address of pressure sensor buffer in block ram
; Addr Range: Len : Description
; 0x70-0xFF : 32  : Address range for entire pressure sensor block
; 0x70      : 1   : Control/status register
; 0x71      : 1   : Data register (for indirect write)
; 0x72      : 1   : Index pointer (LSByte)
; 0x73      : 1   ; Index pointer (MSByte)
; 0x74      : 4   : latch timestamp register
CONSTANT PRESSURE_CTRL_REG, 70
; Pressure Control/status register bits
;  Bit 0 : TIMESTAMP_LATCH (write)
;          When written 1, will copy current 32bit timestamp value into timestamp buffer.
;          Timestamp buffer can be read by reading TIMESTAMP_REGX
;  Bit 1 : READY (write) 
;          When written 1 will will rotate pressure buffer, and 
;          also start transfered of data from pressure buffer to ET1100 
;          Write data is being transfered busy flag (bit2) will be set.
;          Flag will be cleaered when pressure data has been completely copied to ET1100.
;  Bit 2 : BUSY (read)
;          This bit will be set when 
;          treat this as busy flag, pressure buffer should not be rotated again when 
;          flag is set.
;  Bit 3 : ENABLE (read)
;          When set to 1, pressure sensor data should be read, 
;          When set to 0, pressure sensor read should be skipped.
;          This bit is controlled by computer software
	CONSTANT PRESSURE_TIMESTAMP_LATCH_FLAG, 1
	CONSTANT PRESSURE_READY_FLAG, 2
	CONSTANT PRESSURE_BUSY_FLAG, 4
	CONSTANT PRESSURE_ENABLE_FLAG, 8
; Data register that allows indirect write to pressure data buffer.
;  when data register is writen, the value is stored in pressure buffer at location 
;  based on index pointer.  After data is written, index pointer is incremented by 1
CONSTANT PRESSURE_DATA_REG, 71
; Index pointer (9bit).  Points to location where data is stored when next write to 
;  PRESSURE_DATA_REG is preformed.  Index pointer can be changed by writting new value 
;  to register.  Currently, index pointer cannot be read.
CONSTANT PRESSURE_INDEX_LOW_REG, 72  ; Lower 8bits of 9bit index pointer
CONSTANT PRESSURE_INDEX_HIGH_REG, 73 ; Upper 1 bit of 9bit index pointer
; 4-byte register where latched timestamp value is stored
CONSTANT PRESSURE_TIMESTAMP_REG_0, 74   ; LSByte of 32bit timestamp 
CONSTANT PRESSURE_TIMESTAMP_REG_1, 75
CONSTANT PRESSURE_TIMESTAMP_REG_2, 76
CONSTANT PRESSURE_TIMESTAMP_REG_3, 77   ; MSByte of 32bit timestamp


;**********************************************************************************
; SPI hardware peripherals
; Addr Range: Len : Description
; 0x00-0x01 : 2   : Address range for entire SPI block
; 0x00      : 1   : Control/status register
; 0x01      : 1   : Data buffer
; 0x02      : 1   : SPI clock divisor
; ADDR : Description
;  0  : SPI Control/Status register
;    bit 0 :
;      busy (read)
;        when read, bit will be 1 while SPI transfer is occuring(busy), and 0 otherwise (idle)
;    bit 1 : 
;      set idle MOSI (write )
;        writing 1 to this bit will make SPI MOSI (master out/slave in) output HIGH when SPI is idle
;    bit 2 : 
;      clear idle MOSI (write)
;        writing 1 to bit will make SPI MOSI (master out_slave in) output LOW when SPI is idle
;    bit 3 : 
;      assert chip-select 1 (write)
;        when written 1, will assert nChipSelect1 (by driving it low)
;    bit 4 : 
;      assert chip-select 2 (write)
;        when written 1, will assert nChipSelect2 (by driving it low)
;    bit 5 : 
;      de-assert chip selects (write)
;         when written 1, will de-assert both nChipSelect1 and nChipSelect2 (by driving them high)
;  1  :  SPI Data Regiseter
;    bit 0:7
;      output data buffer (write) 
;        When written will immediately take write value and start transfer.
;      input data (r)
;        When read, will contain value shifted in from last transfer.  
;  2  :  SPI clock divisor (write)
;      Takes 25Mhz base clock and divides it down to produce SPI clock
;      Formula for SPI clock frequency = 
;         SPI_CLOCK_FREQUENCY = 12.5Mhz / (SPI_CLOCK_DIVISOR+1)
;      Below is a list of divisor values and the SPI clock rates they produce
;         Divisor    SPI clock frequency
;         0          12.5Mhz
;         1          6.25Mhz
;         9          1.25Mhz
;         14         833.3kHz
CONSTANT SPI_CTRL_REG, 0
	CONSTANT SPI_BUSY_FLAG, 1
	CONSTANT SPI_SET_IDLE_MOSI_FLAG, 2
	CONSTANT SPI_CLR_IDLE_MOSI_FLAG, 4
	CONSTANT SPI_ASSERT_CHIPSEL_1_FLAG, 8
	CONSTANT SPI_ASSERT_CHIPSEL_2_FLAG, 10
	CONSTANT SPI_DEASSERT_CHIPSEL_FLAG, 20
CONSTANT SPI_DATA_REG, 1
CONSTANT SPI_CLOCK_REG, 2


; **************************************************************************************
; SPI constants
; **************************************************************************************
CONSTANT SPI_CLOCK_DIVIDER, 1E  ; 0x1E = 30 --> 403.2kHz clock frequency
;CONSTANT SPI_CLOCK_DIVIDER, E  ; 0xE = 14 --> 833.3kHz clock frequency
;CONSTANT SPI_CLOCK_DIVIDER, 5  ; 0x5 = 5 --> 2500kHz clock frequency
;CONSTANT SPI_CLOCK_DIVIDER, 3  ; 0x3 = 3 --> 3125kHz clock frequency (V)
;CONSTANT SPI_CLOCK_DIVIDER, 1  ; 0x1 = 1 --> 6.25Mhz clock frequency
;CONSTANT SPI_CLOCK_DIVIDER, 0  ; 0x0 = 0 --> 12.5Mhz clock frequency


;**********************************************************************************
; TIMER peripherals (Timer_A and Timer_B)
;
; TimerA/B peripherals
; Addr Range: Len : Description
; 0x04-0x08 : Address range for Timer A
; 0x08-0x0C : Address range for Timer B 
;
; Timer peripherals 
; Offset: bits  : Name/Description
; 0x00 : bit7:0 : Compare register
;	   Timer counter is compared to this register every time it timer strobe 
;          occurs, if counter would end up equaling compare register, counter is 
;          reset to 0 instead and overflow flag it set
;          Any 8-bit value can be written to register
;          Reading register will produce garbage result
;          Because of how compare works, changing value in compare register may
;          Cause value in counter to "miss" matching compare value.
;          For proper operation compare should be set once during initialization
;          and not changed after
;          Compare register will NOT be set to known state when device is reset
; 0x01 : bit7:0 : Counter register
;          Increments by 1 every time timer strobe occurs.
;          Will be reset to 0 when timer strobe occurs and next value would match
;          compare register.
;          When read will return current counter value
;          Counter value can be set to arbitrary value by writing register
;          Counter register will NOT be set to known state when device is reset
; 0x02 : bit0 : Overflow bit 
;          When read returns 1 if overflow bit as been set
;          Write 1 to bit to reset overflow bit. 
;
; Timer register will NOT be set to known states when device is reset.  If using 
;  timer, please initialize or clear all registers during initialization.

; Timer A increments every 0.1 millisecond 
CONSTANT TIMER_A_COMPARE_REG, 4
CONSTANT TIMER_A_COUNTER_REG, 5
CONSTANT TIMER_A_OVERFLOW_REG, 6

; Timer B increments every 1 microsecond = 0.001ms
CONSTANT TIMER_B_COMPARE_REG, 8
CONSTANT TIMER_B_COUNTER_REG, 9
CONSTANT TIMER_B_OVERFLOW_REG, A

CONSTANT TIMER_OVERFLOW_FLAG, 1



;**********************************************************************************
; microsecond delay loop uses 2-instructions per loop + 3 instructions 
; for loading s0, function call & return.  processor uses 2 clock cycles per instruction.
; Clock speed of 25Mhz equates to 12.5 MIPS
; For delay of 10us you need delay count of 
;   delay = 10us
;   mips  = 12.5 instrutions / us
;   instuctions_count = 10us * 12.5 instructions / us = 125 instructions
;   loop_count = (125 instructions - 3 instruction call overhead) * 1 loop / 2instructions = 61
CONSTANT DELAY_40US_COUNT, F9
CONSTANT DELAY_36US_COUNT, E0
CONSTANT DELAY_30US_COUNT, BA
CONSTANT DELAY_10US_COUNT, 3D
CONSTANT DELAY_8US_COUNT, 31
CONSTANT DELAY_6US_COUNT, 24
CONSTANT DELAY_5US_COUNT, 1E
CONSTANT DELAY_4US_COUNT, 18
CONSTANT DELAY_3US_COUNT, 12
CONSTANT DELAY_2US_COUNT, B
CONSTANT DELAY_1US_COUNT, 5




;**********************************************************************************
; FIRST INSTSTRUCTION
init:
	; Initialize SPI clock frequency 
	load s1, SPI_CLOCK_DIVIDER
	output s1, SPI_CLOCK_REG	
	
	; Use timerA
	; Setup timerA to expire every 0.1ms (10,000Hz) (V) (ADC prescalar=4)
	;load s0, 1 ; 0.1ms * 1 = 0.1ms    1 = 0x01
	; Setup timerA to expire every 0.5ms (2,000Hz) (V) (ADC prescalar=4)
    ; 4 ch for 1 reading, so the actual reading rate is 2000/4 = 500 Hz
	load s0, 5 ; 0.1ms * 5 = 0.5ms    5 = 0x05
	; Setup timerA to expire every 2.5ms (400Hz) (V) (ADC prescalar=4)
    ; 4 ch for 1 reading, so the actual reading rate is 400/4 = 100 Hz
	;load s0, 19 ; 0.1ms * 25 = 2.5ms    25 = 0x19

	; Setup timerB to expire every 125us (8,000Hz) (V) (ADC prescalar=4)
	; load s0, 7d ; 0.001ms * 125 = 0.125ms   125 = 0x7d
	; Setup timerB to expire every 63us (16,000Hz) (V)
	; load s0, 3f ; 0.001ms * 63 = 0.063ms    = 0x3f
	; Setup timerB to expire every 1us (1,000,000Hz) (X)
	; load s0, b ; 0.001ms * 1 = 0.001ms    = 0x1
	; Setup timerB to expire every 11us (88,200Hz) (X)
	; load s0, d ; 0.001ms * 13 = 0.013ms    = 0x0d
	; Setup timerB to expire every 14us (71,429Hz) (V)
	; load s0, e ; 0.001ms * 14 = 0.014ms    = 0x0e
	; Setup timerB to expire every 28us (~35,715Hz) (V)
	; load s0, e ; 0.001ms * 28 = 0.028ms    = 0x1c
	; Setup timerB to expire every 23us (~44,100Hz) (V)
	; load s0, 17 ; 0.001ms * 23 = 0.023ms    = 0x17
	; Setup timerB to expire every 45us (~22,050Hz) (V)
	; load s0, 2d ; 0.001ms * 45 = 0.045ms    = 0x2d
	; Setup timerB to expire every 15us (66,666Hz) (V)
	; load s0, f ; 0.001ms * 15 = 0.015ms    = 0x0f
	; Setup timerB to expire every 17us (58,800Hz) (V)
	; load s0, 11 ; 0.001ms * 17 = 0.017ms    = 0x11
	
	output s0, TIMER_A_COMPARE_REG
	load s0, 0  
	output s0, TIMER_A_COUNTER_REG   ; reset Timer B counter to 0
	load s0, TIMER_OVERFLOW_FLAG
	output s0, TIMER_A_OVERFLOW_REG  ; clear overflow flag


;**********************************************************************************
; MAIN LOOP
main_loop:

	; reset index pointer to 0
		load s0, 0
		output s0, PRESSURE_INDEX_LOW_REG
		output s0, PRESSURE_INDEX_HIGH_REG	

	; The first value in pressure data is 32bit timestamp
	; The timestamp is automically copied from 32bit hardware 
	; timer that increments every 1us
		; Have hardware automically copy (latch) timer value into buffer 
		;   so it can be read out later
			load   s0, PRESSURE_TIMESTAMP_LATCH_FLAG
			output s0, PRESSURE_CTRL_REG
			load   s0, 0
		; copy timestamp to begining of pressure data buffer
			input  s0, PRESSURE_TIMESTAMP_REG_0
			output s0, PRESSURE_DATA_REG
			input  s0, PRESSURE_TIMESTAMP_REG_1
			output s0, PRESSURE_DATA_REG
			input  s0, PRESSURE_TIMESTAMP_REG_2
			output s0, PRESSURE_DATA_REG
			input  s0, PRESSURE_TIMESTAMP_REG_3
			output s0, PRESSURE_DATA_REG

    ; Flush the data buffer after every 4 sensor reads (4 channels)
	; (4=0x4) * 1 channel = 4

	load s2, 4;  4 cycles
	read_sensor_multiple_times:

        wait_for_timerA_overflow_loop:
            input s0, TIMER_A_OVERFLOW_REG
            test s0, TIMER_OVERFLOW_FLAG
            jump Z, wait_for_timerA_overflow_loop
        ; clear overflow flag
            load s0, TIMER_OVERFLOW_FLAG
            output s0, TIMER_A_OVERFLOW_REG

        ; Read first sensor (IR receiver)
            ; assert nChipSel1
                load s0, SPI_ASSERT_CHIPSEL_1_FLAG
                output s0, SPI_CTRL_REG
            ; use sub-routine to preform actual read of data
            ; the sub-routine will also de-assert chipselect before returning
                call read_sensor

        ; No need to read from the IR emitter
            ; assert nChipSel2
                ;load s0, SPI_ASSERT_CHIPSEL_2_FLAG
                ;output s0, SPI_CTRL_REG
            ; use sub-routine to preform actual read of data
            ; the sub-routine will also de-assert chipselect before returning
					;call read_sensor

	    sub s2, 1
		jump NZ, read_sensor_multiple_times
	
	  ; New pressure data is ready, all we need to do is flip pressure buffer
		; Make sure old pressure buffer was copied out, before flipping pressure buffer 
		pressure_buf_ready_wait_loop:
			input s0, PRESSURE_CTRL_REG
			test s0, PRESSURE_BUSY_FLAG
			jump NZ, pressure_buf_ready_wait_loop
		; Flag pressure data as being ready
			load s0, PRESSURE_READY_FLAG
			output s0, PRESSURE_CTRL_REG

	; MAIN_LOOP END
	jump main_loop


;**********************************************************************************
; Reads data from sensor into current location of output buffer
; Modifies 
;   s0
; Inputs 
;   s2
; Outputs:
;   NONE
; Preconditions:
;   One of the sensor microcontroller's chip selects has been asserted
;   Pressure sensor buffer is "ready"
; Postconditions:
;   Will de-assert both chip selects
read_sensor:
	; wait for a little time for chipselect assert to propogate (2-3us)
		;load s0, DELAY_2US_COUNT
		load s0, DELAY_40US_COUNT
		call delay_us
	
	;  wait for ADC conversion, ADCH->SPDR...
		;load s0, DELAY_2US_COUNT
		load s0, DELAY_40US_COUNT 
		call delay_us

	; data transfer:  
	;   read 1bytes of data from pressure sensor into buffer
	;   assume pressure index pointer is already pointing to proper place
		;load s0, s2   ; send the current s2 value to the slave (4-3-2-1 cycle)
		call spi_xfer_8
	; put recieved data (s0) into pressure buffer
		output s0, PRESSURE_DATA_REG

	; deassert chipselect and wait for bit for de-select to propogate
		load s0, SPI_DEASSERT_CHIPSEL_FLAG
		output s0, SPI_CTRL_REG
		load s0, DELAY_40US_COUNT
		call delay_us

	; read_pressure_sensor is done
	return 


;**********************************************************************************
; Transfers 8bit value over SPI bus, waits for transfer to complete, 
;   and returns recieved data
; Inputs:
;  s0 : Data byte to send
; Outputs:
;  s0 : Data byte that was recieved'
; Modifies:
;  s0
; Preconditions:
;  SPI is idle (not busy)
; Postcondition:
;  SPI is idle
spi_xfer_8:	
	; put 5h3 current s2 value in SPI buffer and start transfer
	output s2, SPI_DATA_REG
	; wait for transfer to complete
	spi_xfer_busy_wait:
		input s0, SPI_CTRL_REG
		test s0, SPI_BUSY_FLAG
		jump NZ, spi_xfer_busy_wait
	; put result of transfer into s0
	input s0, SPI_DATA_REG
	return


;**********************************************************************************
; Delays for a given number of milliseconds by running an idle loop
; Modifies
;   s0
; Inputs 
;   s0 : Number of delay cycles - use DELAY_XXUS_COUNT constants
; Outputs:
;   NONE
delay_us:
	sub s0, 1
	jump NZ, delay_us
	return 


