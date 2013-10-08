;------------------------------------------------------------------------------
;
; Title:      ds30 Loader for PIC24 and dsPIC
;
; File description: user init and exit code
;
; Copyright:    Copyright © 2011, Mikael Gustafsson
;
;------------------------------------------------------------------------------


;------------------------------------------------------------------------------
;    This file is part of ds30 Loader.
;
;    ds30 Loader is free software: you can redistribute it and/or modify
;    it under the terms of the GNU General Public License as published by
;    the Free Software Foundation.
;
;    ds30 Loader is distributed in the hope that it will be useful,
;    but WITHOUT ANY WARRANTY; without even the implied warranty of
;    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
;    GNU General Public License for more details.
;
;    You should have received a copy of the GNU General Public License
;    along with ds30 Loader. If not, see <http://www.gnu.org/licenses/>.
;------------------------------------------------------------------------------


;------------------------------------------------------------------------------
; UserInit()
; Description: this is executed immediately on start-up before the boot loader code
;------------------------------------------------------------------------------
    .macro UserInit

		; Turn LED on, D3 on Explorer 16 board
		bclr	TRISA, #TRISA4
		bset	LATA, #LATA4
		
		; UART2 rx, map RP19 to uart
		bset	RPINR19, #U2RXR0		;xxx
		bclr	RPINR19, #U2RXR1		;xxx
		bset	RPINR19, #U2RXR2		;xxx
		bclr	RPINR19, #U2RXR3		;xxx
		bclr	RPINR19, #U2RXR4		;xxx
        bset    TRISA, #TRISA0

		; UART tx, map uart to pin, RP25
		bset	RPOR3, #RP6R0			;xxx
		bclr	RPOR3, #RP6R1			;xxx
		bset	RPOR3, #RP6R2			;xxx
		bclr	RPOR3, #RP6R3			;xxx
		bclr	RPOR3, #RP6R4			;xxx
		
		; Make AN12/RP19 digital
		bset	AD1PCFG, #PCFG0
        bset	AD1PCFG, #PCFG1
		
		; Set FRC to 8MHz
		bclr	CLKDIV, #RCDIV0	

    ; End of macro
    .endm


;------------------------------------------------------------------------------
; UserExit()
; Description: this is executed right before the user application is loadaed
;------------------------------------------------------------------------------
    .macro UserExit

		; Turn LED off, D3 on Explorer 16 board
		bclr	LATA, #LATA4
		
    ; End of macro
    .endm


;------------------------------------------------------------------------------
; End of file
;------------------------------------------------------------------------------
