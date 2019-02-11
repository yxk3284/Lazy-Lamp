            TTL Uart stuff
;****************************************************************
;This is the assembly code for the 
;Name:  Sean Bonaventure
;Date:  11/13/18
;Class:  CMPE-250
;Section:  Tuesday 
;---------------------------------------------------------------
;Keil Template for KL46 Assembly with Keil C startup
;R. W. Melton
;November 13, 2017
;****************************************************************
;Assembler directives
            THUMB
            GBLL  MIXED_ASM_C
MIXED_ASM_C SETL  {TRUE}
            OPT   64  ;Turn on listing macro expansions
;****************************************************************
;Include files
            GET  MKL46Z4.s     ;Included by start.s
            OPT  1   ;Turn on listing
;****************************************************************
;EQUates
;EQUates
;---------------------------------------------------------------
;Characters
CR          EQU  0x0D
LF          EQU  0x0A
NULL        EQU  0x00
;---------------------------------------------------------------
;NVIC_ICER
;31-00:CLRENA=masks for HW IRQ sources;
;             read:   0 = unmasked;   1 = masked
;             write:  0 = no effect;  1 = mask
;12:UART0 IRQ mask
NVIC_ICER_UART0_MASK  EQU  UART0_IRQ_MASK
;---------------------------------------------------------------
;NVIC_ICPR
;31-00:CLRPEND=pending status for HW IRQ sources;
;             read:   0 = not pending;  1 = pending
;             write:  0 = no effect;
;                     1 = change status to not pending
;12:UART0 IRQ pending status
NVIC_ICPR_UART0_MASK  EQU  UART0_IRQ_MASK
;---------------------------------------------------------------
;NVIC_IPR0-NVIC_IPR7
;2-bit priority:  00 = highest; 11 = lowest
UART0_IRQ_PRIORITY    EQU  3
NVIC_IPR_UART0_MASK   EQU (3 << UART0_PRI_POS)
NVIC_IPR_UART0_PRI_3  EQU (UART0_IRQ_PRIORITY << UART0_PRI_POS)
;---------------------------------------------------------------
;NVIC_ISER
;31-00:SETENA=masks for HW IRQ sources;
;             read:   0 = masked;     1 = unmasked
;             write:  0 = no effect;  1 = unmask
;12:UART0 IRQ mask
NVIC_ISER_UART0_MASK  EQU  UART0_IRQ_MASK
;---------------------------------------------------------------
;PORTx_PCRn (Port x pin control register n [for pin n])
;___->10-08:Pin mux control (select 0 to 8)
;Use provided PORT_PCR_MUX_SELECT_2_MASK
;---------------------------------------------------------------
;Port A
PORT_PCR_SET_PTA1_UART0_RX  EQU  (PORT_PCR_ISF_MASK :OR: \
                                  PORT_PCR_MUX_SELECT_2_MASK)
PORT_PCR_SET_PTA2_UART0_TX  EQU  (PORT_PCR_ISF_MASK :OR: \
                                  PORT_PCR_MUX_SELECT_2_MASK)
;---------------------------------------------------------------
;SIM_SCGC4
;1->10:UART0 clock gate control (enabled)
;Use provided SIM_SCGC4_UART0_MASK
;---------------------------------------------------------------
;SIM_SCGC5
;1->09:Port A clock gate control (enabled)
;Use provided SIM_SCGC5_PORTA_MASK
;---------------------------------------------------------------
;SIM_SOPT2
;01=27-26:UART0SRC=UART0 clock source select
;         (PLLFLLSEL determines MCGFLLCLK' or MCGPLLCLK/2)
; 1=   16:PLLFLLSEL=PLL/FLL clock select (MCGPLLCLK/2)
SIM_SOPT2_UART0SRC_MCGPLLCLK  EQU  \
                                 (1 << SIM_SOPT2_UART0SRC_SHIFT)
SIM_SOPT2_UART0_MCGPLLCLK_DIV2 EQU \
    (SIM_SOPT2_UART0SRC_MCGPLLCLK :OR: SIM_SOPT2_PLLFLLSEL_MASK)
;---------------------------------------------------------------
;SIM_SOPT5
; 0->   16:UART0 open drain enable (disabled)
; 0->   02:UART0 receive data select (UART0_RX)
;00->01-00:UART0 transmit data select source (UART0_TX)
SIM_SOPT5_UART0_EXTERN_MASK_CLEAR  EQU  \
                               (SIM_SOPT5_UART0ODE_MASK :OR: \
                                SIM_SOPT5_UART0RXSRC_MASK :OR: \
                                SIM_SOPT5_UART0TXSRC_MASK)
;---------------------------------------------------------------
;UART0_BDH
;    0->  7:LIN break detect IE (disabled)
;    0->  6:RxD input active edge IE (disabled)
;    0->  5:Stop bit number select (1)
;00001->4-0:SBR[12:0] (UART0CLK / [9600 * (OSR + 1)]) 
;UART0CLK is MCGPLLCLK/2
;MCGPLLCLK is 96 MHz
;MCGPLLCLK/2 is 48 MHz
;SBR = 48 MHz / (9600 * 16) = 312.5 --> 312 = 0x138
UART0_BDH_9600  EQU  0x01
;---------------------------------------------------------------
;UART0_BDL
;0x38->7-0:SBR[7:0] (UART0CLK / [9600 * (OSR + 1)])
;UART0CLK is MCGPLLCLK/2
;MCGPLLCLK is 96 MHz
;MCGPLLCLK/2 is 48 MHz
;SBR = 48 MHz / (9600 * 16) = 312.5 --> 312 = 0x138
UART0_BDL_9600  EQU  0x38
;---------------------------------------------------------------
;UART0_C1
;0-->7:LOOPS=loops select (normal)
;0-->6:DOZEEN=doze enable (disabled)
;0-->5:RSRC=receiver source select (internal--no effect LOOPS=0)
;0-->4:M=9- or 8-bit mode select 
;        (1 start, 8 data [lsb first], 1 stop)
;0-->3:WAKE=receiver wakeup method select (idle)
;0-->2:IDLE=idle line type select (idle begins after start bit)
;0-->1:PE=parity enable (disabled)
;0-->0:PT=parity type (even parity--no effect PE=0)
UART0_C1_8N1  EQU  0x00
;---------------------------------------------------------------
;UART0_C2
;0-->7:TIE=transmit IE for TDRE (disabled)
;0-->6:TCIE=transmission complete IE for TC (disabled)
;0-->5:RIE=receiver IE for RDRF (disabled)
;0-->4:ILIE=idle line IE for IDLE (disabled)
;1-->3:TE=transmitter enable (enabled)
;1-->2:RE=receiver enable (enabled)
;0-->1:RWU=receiver wakeup control (normal)
;0-->0:SBK=send break (disabled, normal)
UART0_C2_T_R    EQU  (UART0_C2_TE_MASK :OR: UART0_C2_RE_MASK)
UART0_C2_T_RI   EQU  (UART0_C2_RIE_MASK :OR: UART0_C2_T_R)
UART0_C2_TI_RI  EQU  (UART0_C2_TIE_MASK :OR: UART0_C2_T_RI)
;---------------------------------------------------------------
;UART0_C3
;0-->7:R8T9=9th data bit for receiver (not used M=0)
;           10th data bit for transmitter (not used M10=0)
;0-->6:R9T8=9th data bit for transmitter (not used M=0)
;           10th data bit for receiver (not used M10=0)
;0-->5:TXDIR=UART_TX pin direction in single-wire mode
;            (no effect LOOPS=0)
;0-->4:TXINV=transmit data inversion (not inverted)
;0-->3:ORIE=overrun IE for OR (disabled)
;0-->2:NEIE=noise error IE for NF (disabled)
;0-->1:FEIE=framing error IE for FE (disabled)
;0-->0:PEIE=parity error IE for PF (disabled)
UART0_C3_NO_TXINV  EQU  0x00
;---------------------------------------------------------------
;UART0_C4
;    0-->  7:MAEN1=match address mode enable 1 (disabled)
;    0-->  6:MAEN2=match address mode enable 2 (disabled)
;    0-->  5:M10=10-bit mode select (not selected)
;01111-->4-0:OSR=over sampling ratio (16)
;               = 1 + OSR for 3 <= OSR <= 31
;               = 16 for 0 <= OSR <= 2 (invalid values)
UART0_C4_OSR_16           EQU  0x0F
UART0_C4_NO_MATCH_OSR_16  EQU  UART0_C4_OSR_16
;---------------------------------------------------------------
;UART0_C5
;  0-->  7:TDMAE=transmitter DMA enable (disabled)
;  0-->  6:Reserved; read-only; always 0
;  0-->  5:RDMAE=receiver full DMA enable (disabled)
;000-->4-2:Reserved; read-only; always 0
;  0-->  1:BOTHEDGE=both edge sampling (rising edge only)
;  0-->  0:RESYNCDIS=resynchronization disable (enabled)
UART0_C5_NO_DMA_SSR_SYNC  EQU  0x00
;---------------------------------------------------------------
;UART0_S1
;0-->7:TDRE=transmit data register empty flag; read-only
;0-->6:TC=transmission complete flag; read-only
;0-->5:RDRF=receive data register full flag; read-only
;1-->4:IDLE=idle line flag; write 1 to clear (clear)
;1-->3:OR=receiver overrun flag; write 1 to clear (clear)
;1-->2:NF=noise flag; write 1 to clear (clear)
;1-->1:FE=framing error flag; write 1 to clear (clear)
;1-->0:PF=parity error flag; write 1 to clear (clear)
UART0_S1_CLEAR_FLAGS  EQU  (UART0_S1_IDLE_MASK :OR: \
                            UART0_S1_OR_MASK :OR: \
                            UART0_S1_NF_MASK :OR: \
                            UART0_S1_FE_MASK :OR: \
                            UART0_S1_PF_MASK)
;---------------------------------------------------------------
;UART0_S2
;1-->7:LBKDIF=LIN break detect interrupt flag (clear)
;             write 1 to clear
;1-->6:RXEDGIF=RxD pin active edge interrupt flag (clear)
;              write 1 to clear
;0-->5:(reserved); read-only; always 0
;0-->4:RXINV=receive data inversion (disabled)
;0-->3:RWUID=receive wake-up idle detect
;0-->2:BRK13=break character generation length (10)
;0-->1:LBKDE=LIN break detect enable (disabled)
;0-->0:RAF=receiver active flag; read-only
UART0_S2_NO_RXINV_BRK10_NO_LBKDETECT_CLEAR_FLAGS  EQU  \
        (UART0_S2_LBKDIF_MASK :OR: UART0_S2_RXEDGIF_MASK)
;---------------------------------------------------------------
DAC0_STEPS	EQU		4096
SERVO_POSITIONS EQU	5

TPM_CnV_PWM_DUTY_2ms EQU 5750
TPM_CnV_PWM_DUTY_1ms EQU 1750 
	
PWM_2ms 	EQU 	TPM_CnV_PWM_DUTY_2ms
PWM_1ms 	EQU 	TPM_CnV_PWM_DUTY_1ms
	
MaxSize		EQU		79
QueueISize	EQU		80
QueueRcSize	EQU		18
IN_PTR		EQU		0
OUT_PTR		EQU		4
BUF_STRT	EQU		8
BUF_PAST	EQU		12
BUF_SIZE	EQU		16
NUM_ENQD	EQU		17

;****************************************************************
;MACROs
			MACRO
			SETC
			PUSH	{R0, R1}
			MRS 	R0, APSR	;Following lines set C flag with out changing other flags
			MOVS 	R1, #0x20	
			LSLS 	R1, R1, #24
			ORRS 	R0, R0, R1
			MSR 	APSR, R0
			POP		{R0, R1}
			MEND
			
			MACRO
			CLEARC
			PUSH 	{R0, R1} 
			MRS   	R0,APSR
			MOVS  	R1,#0x20
			LSLS  	R1,R1,#24
			BICS  	R0,R0,R1
			MSR   	APSR,R0
			POP		{R0, R1}
			MEND
			
			MACRO
			NEWLINE		
			PUSH	{R0, R1}
			MRS		R1, APSR
			MOVS	R0,#13
			BL		PutChar
			MOVS	R0,#10
			BL		PutChar
			MSR		APSR, R1
			POP		{R0, R1}
			MEND
;****************************************************************
;Program
;C source will contain main ()
;Only subroutines and ISRs in this assembly source
            AREA    MyCode,CODE,READONLY
;>>>>> begin subroutine code <<<<<
			EXPORT	Init_UART0_IRQ
			EXPORT	PutChar
			EXPORT  GetChar
			EXPORT  PutStringSB
			EXPORT  GetStringSB
			EXPORT	PutNumUB
			EXPORT	PutNumHex
			EXPORT	UART0_IRQHandler
			EXPORT  GetCharNoBlock
UART0_IRQHandler	PROC	{R0-R14}
			;Handles the interupt for the UART. General algorithm: Checks to see if TIE is set. If its not then
			;We know its not an transmit interupt. If it is we then check if TDRE is set. If so we can send a character. If
			;not then disable TIE. If TIE is not sent then we know its a RDRF interupt. So we read the data from the data
			;register then add it to the get queue
			CPSID   I
			PUSH	{R4,R5,R6, LR}
			LDR		R4,=UART0_BASE
			LDRB	R5,[R4,#UART0_C2_OFFSET]		;Load the C2 register values into R5
			LDRB	R2,[R4,#UART0_S1_OFFSET]
			MOVS	R3,#UART0_C2_TIE_MASK			;Get the mask to check if TIE is set
			TST		R5,R3							;And them together to see if TIE is set
			BEQ		CHECKRx							;If the result is 0 TIE is not set, so check if RIE is set
			MOVS	R6,#UART0_S1_TDRE_MASK			;Get the mask to check is TDRE is set
			TST		R5,R6							;And them to check
			BEQ		CHECKRx							;If the result is 0 TDRE is not set, then check receive interupt
													;If it is set then we know the interupt was caused by being able to 
													;Send a character, so we try and send a character by dequeue, then saving
			LDR		R1,=putQueueRC
			BL		DEQUEUE
			BCS		DISABLETIE						;If the C flag is set, nothing is in the queue, so disable TIE
			STRB	R0,[R4,#UART0_D_OFFSET]			;If it isn't set save the character to the data register to send
			B		ENDUARTISR						;Then go to the end of ISR
DISABLETIE	MOVS	R3,#UART0_C2_T_RI				;To turn off TIE we need to and it with the mask for regular C2 and RIE on
			ANDS	R5,R5,R3						;And them to turn of TIE, save in R5
			STRB	R5,[R4,#UART0_C2_OFFSET]		;Save it to C2
			B		ENDUARTISR						;End the program
			
CHECKRx		MOVS	R3,#UART0_S1_RDRF_MASK			;Get the mask to check if 
			TST		R2,R3							;Check to see if RIE is set
			BEQ		ENDUARTISR						;If not end ISR
			LDRB	R0,[R4,#UART0_D_OFFSET]
			LDR		R1,=getQueueRC
			BL		ENQUEUE
ENDUARTISR	CPSIE   I
			POP		{R4,R5,R6, PC}			
			ENDP
				
Init_UART0_IRQ		PROC 	{R0-R14}
			PUSH	{R0-R2, LR}

			;Initialize queues
			;R0:Address of buffer
			;R1:Addresss of record
			;R2:Size
			LDR		R0,=putQueue
			LDR		R1,=putQueueRC
			MOVS	R2,#QueueISize
			BL		InitQueue
			LDR		R0,=getQueue
			LDR		R1,=getQueueRC
			BL		InitQueue
			;Select MCGPLLCLK / 2 as UART0 clock source
			LDR 	R0,=SIM_SOPT2					
			LDR 	R1,=SIM_SOPT2_UART0SRC_MASK
			LDR 	R2,[R0,#0]
			BICS 	R2,R2,R1
			LDR 	R1,=SIM_SOPT2_UART0_MCGPLLCLK_DIV2
			ORRS 	R2,R2,R1
			STR 	R2,[R0,#0]
			;Enable external connection for UART0
			LDR 	R0,=SIM_SOPT5
			LDR 	R1,= SIM_SOPT5_UART0_EXTERN_MASK_CLEAR
			LDR 	R2,[R0,#0]
			BICS 	R2,R2,R1
			STR 	R2,[R0,#0]
			;Enable clock for UART0 module
			LDR 	R0,=SIM_SCGC4
			LDR 	R1,= SIM_SCGC4_UART0_MASK
			LDR 	R2,[R0,#0]
			ORRS 	R2,R2,R1
			STR 	R2,[R0,#0]
			;Enable clock for Port A module
			LDR 	R0,=SIM_SCGC5
			LDR 	R1,= SIM_SCGC5_PORTA_MASK
			LDR 	R2,[R0,#0]
			ORRS 	R2,R2,R1
			STR 	R2,[R0,#0]
			;Connect PORT A Pin 1 (PTA1) to UART0 Rx (J1 Pin 02)
			LDR 	R0,=PORTA_PCR1
			LDR 	R1,=PORT_PCR_SET_PTA1_UART0_RX
			STR 	R1,[R0,#0]
			;Connect PORT A Pin 2 (PTA2) to UART0 Tx (J1 Pin 04)
			LDR 	R0,=PORTA_PCR2
			LDR 	R1,=PORT_PCR_SET_PTA2_UART0_TX
			STR 	R1,[R0,#0]
			
			;Now we are going to set up interrupts with NVIC
			LDR		R0,=UART0_IPR
			LDR		R2,=NVIC_IPR_UART0_MASK
			LDR		R3,[R0,#0]
			ORRS	R3,R3,R1
			STR		R3,[R0,#0]
			
			LDR     R0,=NVIC_ICPR 
			LDR		R1,=NVIC_ICPR_UART0_MASK
			STR		R1,[R0,#0]
			
			LDR		R0,=NVIC_ISER
			LDR		R1,=NVIC_ISER_UART0_MASK
			STR		R1,[R0,#0]
			
			;Setting up the actual UART		
			LDR		R0,=UART0_BASE	;Load the base address of the UART so we can just add to is to get registsers
			
			;Setting up baud rate
			MOVS	R1,#UART0_BDH_9600
			STRB	R1,[R0,#UART0_BDH_OFFSET]
			
			;Setting up baud rate
			MOVS	R1,#UART0_BDL_9600
			STRB	R1,[R0,#UART0_BDL_OFFSET]
			
			;Setting control register 1
			MOVS	R1, #UART0_C1_8N1 ;Use 0x00 because none of the bits are supposed to be set
			STRB	R1,[R0,#UART0_C1_OFFSET]
			
			;Setting control register 2
			MOVS	R1, #UART0_C2_T_RI			;Enable the transmitters and turn on the interrupt for receive
			STRB	R1,[R0,#UART0_C2_OFFSET]
			
			;Setting control register 3
			MOVS	R1, #UART0_C3_NO_TXINV ;Use 0x00 because none of the bits are supposed to be set
			STRB	R1,[R0,#UART0_C3_OFFSET]
			
			;Setting up control register 4
			MOVS	R1, #UART0_C4_NO_MATCH_OSR_16	;Use 0x0C because the first 4 bits need to be set for OSR,i.e: 0000 1111 = 0x0F
			STRB	R1,[R0,#UART0_C4_OFFSET] 
			
			;Setting up control register 5
			MOVS	R1, #UART0_C5_NO_DMA_SSR_SYNC	;Use 0x00 because none of the bits are supposed to be set
			STRB	R1,[R0,#UART0_C5_OFFSET]
			
			;Clearing the S1 flags
			MOVS	R1, #UART0_S1_CLEAR_FLAGS	;Use 0x0C because the last 5 bits need to be set in order to clear the flags,i.e: 0001 1111 = 0x1F
			STRB	R1,[R0,#UART0_S1_OFFSET]
			
			;Clearing the S1 flags
			MOVS	R1, #UART0_S2_NO_RXINV_BRK10_NO_LBKDETECT_CLEAR_FLAGS
			STRB	R1,[R0,#UART0_S1_OFFSET]
			
			

			POP		{R0-R2, PC}
			BX		LR
			ENDP
				
PutChar		PROC 	{R1-R14}
			;PutChar using interrupts. Put char will add the character to the put Queue. Then it sets the TIE to enable interrupts for TDRE
			;Inputs:
			;R0: The character to print
			;Outputs:
			;None
			PUSH	{R1-R3, LR}
			CPSID   I							;mask them
			MRS		R1, APSR
PUTENQ		LDR		R1,=putQueueRC				;Get the address of the queue record
			BL		ENQUEUE						;Attempt to put it in the Q
			CPSIE   I							;Unmask them
			BCS		PUTENQ						;If it fails to put it in, keep trying until it can
			LDR		R1,=UART0_BASE				;Get the base address for setting interrupt
			LDRB	R2,[R1,#UART0_C2_OFFSET]	;Get the C2 value
			MOVS	R3,#UART0_C2_TI_RI			;This mask makes sure TIE and RIE are set
			ORRS	R2,R3,R2					;Or the mask and resgister that way TIE and RIE are set
			STRB	R2,[R1,#UART0_C2_OFFSET]	;Save it back to the control register
			MSR		APSR, R1
			POP		{R1-R3, PC}
			ENDP
				
GetChar		PROC	{R1-R14}
			;Gets a character from the get queue. 
			;Inputs:
			;None
			;Outputs:
			;R0: The character gotten
			PUSH	{R1, LR}
			MRS		R1, APSR
			LDR		R1,=getQueueRC
GETQUEUE	CPSID   I
			BL		DEQUEUE
			CPSIE   I
			BCS		GETQUEUE
			MSR		APSR, R1
			POP		{R1, PC}
			ENDP

GetCharNoBlock PROC	{R1-R14}
			;This subroutine acts like a get char that isnt a blocking call
			;It simply dequeues the first item from the receive buffer. If nothing is there
			;Returns null character
			;Outputs:
			;R0: The Character returned
			PUSH	{R1-R2, LR}
			MRS		R2, APSR
			LDR		R1,=getQueueRC
			CPSID   I
			BL		DEQUEUE				;Dequeue the character
			CPSIE   I
			BCC		GOTCHAR				;If succesful done
			MOVS	R0,#0				;If not move the null character in
GOTCHAR		MSR		APSR, R2
			POP		{R1-R2, PC}
			ENDP
				
DIVU		PROC {R2-R14}
	;Divides the contents of R1 by R0. Sets C flag if R0 is 0
	;Parameters: 
	;R0: Divisor
	;R1: Divident
	;Outputs: 
	;R0: Quotient
	;R1: Remainder
			PUSH	{R3} ;Push R3 because Im going to use it as a counter
			MOVS	R3, #0
			CMP		R0, #0 ;See if we are dividing by 0
			BEQ		DVDZERO
WHILE		CMP		R1, R0
			BLO		ENDWHILE
			SUBS 	R1, R1, R0
			ADDS	R3, R3, #1
			B 		WHILE
ENDWHILE	MOVS 	R0, R3
			B DVDEND
DVDZERO		PUSH	{R0, R1}
			MRS 	R0, APSR	;Following lines set C flag with out changing other flags
			MOVS 	R1, #0x20	
			LSLS 	R1, R1, #24
			ORRS 	R0, R0, R1
			MSR 	APSR, R0
			POP		{R0, R1}
DVDEND		POP		{R3}
			BX		LR
			ENDP
				
GetStringSB	PROC	{R0-R14}
			;Prints a string from the UART and saves it to a string variable
			;Inputs:
			;R1: Size of buffer
			;Outputs:
			;Filled string variable
			PUSH	{R0, R2, R3, LR}
			LDR		R2,=String1		;Pointer to String1. Diretions say use R0 but that is used in get/put char so we chagning it
			ADDS	R3,R2,R1		;Index of head of string + buffer size is the last possible index of the string
WHILEGET	BL		GetChar			;Get the character
			CMP		R0, #13			;See if the character is the enter character
			BEQ		ENDWHILEGET		;If so then branch out
			CMP		R0,#8			;See if the backspace character was sent
			BEQ		Backspace		;If so go to backspace section
			CMP		R0, #0x1F		;See if it is a control character by doing value-0x1f
			BLO		WHILEGET		;If the value is negative then it is within the range, so skip it
			CMP		R0,	#0x7F		;If the char is another control skip it
			BEQ		WHILEGET
			CMP		R3, R2			;(String + Buffer) - String
			BLO		WHILEGET		;If the memory address of the String pointer is greater or equal to branch out
			BL		PutChar			;Put the character out so the user can see
			STRB	R0,[R2,#0]		;If not store the character into the String
			ADDS	R2,#1			;Add one to the pointer to get next address
			B		WHILEGET		;Branch back up
ENDWHILEGET	MOVS	R0, #0			;Move 0, the null terminator into the register
			BL		PutChar
			STRB	R0,[R2, #0]		;Put it at the last address of the string
			MOVS	R0, #13			;Move 13, the enter character into the register. Doing this to move the cursor back all the left
			BL		PutChar			;Output it to the UART
			MOVS	R0, #10			;Move the newline character into the register so we can make a newline on the terminal
			BL		PutChar			;Put it to the UART
			B		EndSubGet
			
Backspace	LDR		R4,=String1		;Load address of string 1 so we can see if we are at it
			CMP		R2,R4			;See if we are at the mem address
			BEQ		WHILEGET		;Branch back so we dont go into non ram memory
			SUBS	R2, #1			;Subtract 1 from pointer so it overrides element
			MOVS	R0,#8			;Move backspace ascii in
			BL		PutChar			;Output so cursor goes back
			MOVS	R0, #' '			;Put space in
			BL		PutChar			;Output space
			MOVS	R0,#8			;Move backspace in
			BL		PutChar			;Output backspace
			B		WHILEGET		;Go back to while get
EndSubGet	POP		{R0,R2, R3, PC}
			ENDP
				
				
PutStringSB	PROC	{R2-R14}	
			;Prints a string to the UART
			;Inputs:
			;R0: Pointer to string head
			;R1: Size of buffer
			;Outputs:
			;None
			PUSH	{R2-R5, LR}		;We push LR because using BL in the subroutine resets it. So we need to save BL
			MOVS	R2, R0			;Moving R0, the string pointer, to R2 because R0 is used by putchar
			ADDS	R3,R2,R1		;Find the end address of the string
WHILEPUT	CMP		R3, R2			;(String + Buffer) - String
			BLS		ENDWHILEPUT		;If the memory address of the String pointer is greater or equal to  the end address branch out
			LDRB	R0,[R2, #0]		;Load the character into R0
			CMP		R0, #0			;Check to see if the character is the null terminator
			BEQ		ENDWHILEPUT		;If so branch out
			BL		PutChar			;Output it
			ADDS	R2, #1			;Add one to the string pointer
			B		WHILEPUT		;Loop back to top
ENDWHILEPUT	MOVS	R0, #0			;Move the null character to the registers putchar uses
			BL		PutChar			;Output
			POP		{R2-R5, PC}		;Pop everything, including PC. This will bounce us back to the right spot
			ENDP

PutNumU		PROC	{R0-R14}
			;Takes a hex number and prints the decimal equivlent to the UART
			;Basically this method is going to iterivly divide by 10 and put the 
			;remainder (the digit) into an array. Then we iterate backwards through the arrray to 
			;print each output
			;Inputs
			;R0: The number to be printed
			;Outputs:
			;None
			PUSH	{R0-R4, LR}
			CMP		R0,#0
			BNE		NOTZERO
			MOVS	R0,#'0'
			BL		PutChar
			B		ENDPUT
NOTZERO		MOVS	R2,#0			;Counter to see how many items in stack
			MOVS	R1, R0			;Move the dividend to the proper register
NUMWHILE	MOVS	R0, #0xA		;Move 10 to the divisor
			CMP		R1, R0			;If R1 is less than 10 end loop
			BLO		ENDNMWHILE
			BL		DIVU			;Divide R1, the current dividend, by 10
			PUSH	{R1}
			ADDS	R2, #1			;Add 1 to the counter
			MOVS	R1, R0			;Put the quotient into the dividend spot
			B		NUMWHILE		;Go back to top
ENDNMWHILE	PUSH	{R1}
			MOVS	R1,#0
PRINTWHILE	POP		{R0}
			ADDS	R0,#48	
			BL		PutChar
			SUBS	R2, #1
			BLO		ENDPUT
			B		PRINTWHILE
LOGNUM		MOVS	R1,#1			;Set R1 to a one
			ADDS	R0,#48			;Add 48 to the number to get the proper ascii code. ex, if the num was 
									;5 then 5+48=53, and 53 is the ascii code for 5
			BL		PutChar			;Put the character
			SUBS	R3, #1			;Subtract 1 from the pointer
			B		PRINTWHILE
ENDPUT		POP		{R0-R4, PC}
			ENDP
				
InitQueue	PROC	{R0-R14}
	;Creates a Queue record at the memory address in R1, for the queue data at
	;R0 with the size in R2
	;Input:
	;R0:Address of buffer
	;R1:Addresss of record
	;R2:Size
	;Output:None
			PUSH	{R0-R7}
			STR		R0,[R1,#IN_PTR] 		;Save the starting address of the buffer at IN_PTR
			STR		R0,[R1,#OUT_PTR]		;Save the start at OUT_PTR
			STR		R0,[R1,#BUF_STRT]		;Store begining address in BUFF_START
			ADDS	R3, R0, R2		;Find the ending address of the buffer
			STR		R3,[R1,#BUF_PAST]		;Store end addresss in BUF_PAST
			STRB	R2,[R1,#BUF_SIZE] 	;Store size of queue(R2)
			MOVS	R0,#0
			STRB	R0,[R1,#NUM_ENQD]		;Store the size in the queue, which is 0 at the start, in NUM_ENQD
			POP		{R0-R7}
			BX		LR
			ENDP
				
ENQUEUE		PROC	{R0-R13}
			;Adds character to queue. Record address is R1, character is at R0
			;If queue is full sets C flag
			;Input: 
			;R1: Queue record address
			;R0: Character
			;Ouput:
			;PSR: Sets C if full
			PUSH	{R0-R7}
			LDRB	R2, [R1,#BUF_SIZE]		;Load buffer size
			LDRB	R3, [R1,#NUM_ENQD]		;Load number in the queue
			CMP		R3, R2					;Check to see if number in queue is equal to or greater than size
			BLO		STRTQUEUE				
			SETC							;If so, set C, go to end
			B		ENDENQUEUE
STRTQUEUE	LDR		R2,[R1, #IN_PTR]		;Get address of the in_ptr
			STRB	R0,[R2,#0]				;Store value to the queue
			ADDS	R2,#1					;Add 1 to in_ptr
			ADDS	R3,#1					;Add 1 to number of items in the queue
			STRB	R3,[R1,#NUM_ENQD]		;Store the new number enqd
			LDR		R3,[R1,#BUF_PAST]		;Get the value of Buff past
			STR		R2,[R1,#IN_PTR]	
			CMP		R3, R2					;See if the buf_past is greater or equal to in ptr
			BHI		enqClr				;If so then we are done
			LDR		R2,[R1,#BUF_STRT]		;If not put the in_ptr value to the start of the buffer
			STR		R2,[R1,#IN_PTR]	
enqClr		NOP
			CLEARC							;Clear the C flag
ENDENQUEUE	POP		{R0-R7}
			BX		LR
			ENDP
				
DEQUEUE		PROC	{R1, R13}
			;Will pop an item out of the queue. Sets C flag is operation fails, clears if succedes
			;Inputs
			;R1: Address of the record structure
			;Outputs:
			;R0: The character popped out
			PUSH	{R1-R7}
			LDRB	R2,[R1,#NUM_ENQD]		;Load the number of items in the queue
			CMP		R2, #0					;Compare it to 0
			BEQ		DEQUEUEFAIL				;If its 0 go to the branch that sets c and stuff
			LDRB	R3,[R1,#BUF_SIZE]		;Get the buffer size
			CMP		R2, R3					;See if the number in the queue is higher or the same as buffer size
			BHI		DEQUEUEFAIL				;If so go to the failure branch
			LDR		R3,[R1,#OUT_PTR]		;Get the pointer to the value in the queue
			LDRB	R0,[R3,#0]				;Load the value to be popped into R0
			ADDS	R3, #1					;Increment the out pointer by 1
			SUBS	R2, #1					;decrement number enqueued by 1
			STRB	R2,[R1,#NUM_ENQD]		;Store the new number enqueded
			LDR		R2,[R1,#BUF_PAST]		;Get the value of the first memory addresss past the end
			CLEARC							;Clear the C flag
			CMP		R3, R2					;See if the new out pointer is past
			BLO		deqClr				;If it is lower, go to the area of code that finished dequeue
			LDR		R2,[R1,#BUF_STRT]		;If it's higher then get the address of the start of the buffer
			MOVS	R3, R2					;And set the out pointer to the start of the buffer
deqClr		STR		R3,[R1,#OUT_PTR]		;Save the new outpointer
			CLEARC							;Clear the C flag
			B		ENDDEQUEUE
DEQUEUEFAIL	NOP
			SETC
ENDDEQUEUE	POP		{R1-R7}
			BX		LR
			ENDP

PutNumHex	PROC	{R0-R14}
			;Prints hex number contained in R0
			;Input: 
			;R0: The number to print
			;General strategy: use mask to isolate msb byte, shift right to align them to lsb, then find 
			;ASCII value. I'm doing this instead of just masking LSB becuuse I dont want to use the stack
			PUSH	{R0-R7, LR}
			MOVS	R3, R0			;Put the hex value in R3 because R0 is used by put char
			MOVS	R4, #0			;Set loop counter to 0
WHILEHEX	CMP		R4, #8			;Check to see if it ran 8 iterations
			BHS		ENDHEX			;Skip to end of the loop
			MOVS	R1, #0xF		;Set the mask to make a nibble all ones
			LSLS	R1, R1, #28		;Shift the nibble to the MSB
			MOVS	R0, R3			;Put the value into R2
			ANDS	R0, R0, R1		;And it to get isolate the MSB
			LSRS	R0, R0, #28		;Shift the MSB right to get the actual value
			CMP		R0, #9			;See if its less than 9. If so add 48 to print an integer
			BHI		HEXVAL			
			ADDS	R0, #48			;Adding 48
			BL		PutChar			;Print the character
			LSLS	R3, #4			;Left shift the whole thing
			ADDS	R4, #1			;Increment loop counter
			B		WHILEHEX		;Go back up to top
HEXVAL		ADDS	R0, #55			;Add 65 to get hex values A-F
			BL		PutChar
			LSLS	R3, #4			;Left shift the whole thing
			ADDS	R4, #1			;Increment loop counter
			B		WHILEHEX		;Go back up to top
ENDHEX		POP		{R0-R7, PC}
			ENDP
				
PutNumUB	PROC	{R0-R14}
			;Prints the least significant bit of a register
			;Input:
			;R0: The value to be printed
			PUSH	{R0-R7, LR}
			MOVS	R1, #0xFF
			ANDS	R0, R0, R1
			BL		PutNumU
			POP		{R0-R7, PC}
			ENDP
				
PrintQueue	PROC	{R0-R14}
			PUSH	{R0-R4}
			
			POP		{R0-R4}
			ENDP				
;>>>>>   end subroutine code <<<<<
            ALIGN
;**********************************************************************
;Constants
            AREA    MyConst,DATA,READONLY
;>>>>> begin constants here <<<<<
;ROM lookup table of digital values for conversion to analog
			EXPORT DAC0_table_0 ;make available to C program 
DAC0_table_0
DAC0_table
			DCW ((DAC0_STEPS - 1) / (SERVO_POSITIONS * 2))
			DCW (((DAC0_STEPS - 1) * 3) / (SERVO_POSITIONS * 2))
			DCW (((DAC0_STEPS - 1) * 5) / (SERVO_POSITIONS * 2))
			DCW (((DAC0_STEPS - 1) * 7) / (SERVO_POSITIONS * 2))
			DCW (((DAC0_STEPS - 1) * 9) / (SERVO_POSITIONS * 2))


			EXPORT PWM_duty_table_0 ;include if accessed from C
PWM_duty_table
PWM_duty_table_0 ;include if accessed from C
;Servo positions from 1 (leftmost) to 5 (rightmost)
			DCW PWM_2ms ;-50% of range
			DCW ((3*(PWM_2ms-PWM_1ms)/4)+PWM_1ms) ;-25% of range
			DCW (((PWM_2ms-PWM_1ms)/2)+PWM_1ms) ; 0% of range
			DCW (((PWM_2ms-PWM_1ms)/4)+PWM_1ms) ;+25% of range
			DCW PWM_1ms ;+50% of range

;>>>>>   end constants here <<<<<
;**********************************************************************
;Variables
            AREA    MyData,DATA,READWRITE
;>>>>> begin variables here <<<<<
putQueueRC	SPACE	QueueRcSize
			ALIGN
getQueueRC  SPACE	QueueRcSize
			ALIGN
putQueue	SPACE	QueueISize
getQueue	SPACE	QueueISize	
String1		SPACE	MaxSize
;>>>>>   end variables here <<<<<
            END