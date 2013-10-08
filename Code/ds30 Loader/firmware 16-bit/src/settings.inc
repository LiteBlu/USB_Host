;------------------------------------------------------------------------------
;
; Title:      ds30 loader
;
; File description: user settings and configuration bits for PIC24 and dsPIC
;
; Copyright:    Copyright 2009-2012 Mikael Gustafsson
;
;------------------------------------------------------------------------------

;-----------------------------------------------------------------------------
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
; Device
;------------------------------------------------------------------------------
    .equ __24FJ64GB002,     1

;------------------------------------------------------------------------------
; Includes
;------------------------------------------------------------------------------
    .include  "devices.inc"

;------------------------------------------------------------------------------
; User preferences
;------------------------------------------------------------------------------
    .equiv  FCY,        	16000000 ;xxx speed of internal cycle clock[Hz], used to calculate uart brg and delay
    .equiv  BLINIT,       	3000    ;xxx hello receive timeout [ms]
    .equiv  HELLOTRIES,    	2    	;xxx number of non hello characters received before branching to the user application
    .equiv  BLTIME,       	3000    ;xxx data receive timeout [ms]

;------------------------------------------------------------------------------
; UART settings
;------------------------------------------------------------------------------
  	.equiv USE_UART2,      	1    	;xxx uncomment to use uart2
    .equiv BAUDRATE,    	115200  ;xxx baudrate
    .equiv USE_BRGH,       	1     	;xxx uncomment to use high baud rates, READ ERRATA FIRST, not available for PIC24F


;------------------------------------------------------------------------------
; Advanced settings
;------------------------------------------------------------------------------
  .equiv KICK_WD,        	1     	;xxx uncomment to kick the wd in the rx loop, ONLY enable if watchdog is enabled
  .equiv WRITE_VER,      	1     	;xxx do flash write verification
  .equiv EWRITE_VER,     	1     	;xxx do eeprom write verification
  .equiv PROT_GOTO,      	1     	;protect goto at 0x00
  .equiv PROT_BL,        	1     	;protect bootloader

  .equiv BLPLP,        		2     	;bootloader placement, pages from end
  .equiv BLSIZEP,     	 	1		;bootloader size [pages], used by bootloader protection

;------------------------------------------------------------------------------
; Configuration bits,
;------------------------------------------------------------------------------
  ;config __CONFIG1, JTAGEN_OFF & GCP_OFF & GWRP_OFF & ICS_PGx1 & FWDTEN_ON & WINDIS_OFF & WDTPS_PS256 & FWPSA_PR128
  ;config __CONFIG2, IESO_OFF & FNOSC_FRCPLL & FCKSM_CSDCMD & OSCIOFNC_OFF & POSCMOD_XT & PLL96MHZ_ON & PLLDIV_DIV2

   config __CONFIG1, WDTPS_PS1 & FWPSA_PR32 & WINDIS_OFF & FWDTEN_OFF & ICS_PGx1 & GWRP_OFF & GCP_OFF & JTAGEN_OFF
   config __CONFIG2, POSCMOD_HS & I2C1SEL_PRI & IOL1WAY_OFF & OSCIOFNC_ON & FCKSM_CSECME & FNOSC_PRIPLL & PLL96MHZ_ON & PLLDIV_DIV2 & IESO_ON
   config __CONFIG3, WPFP_WPFP0 & SOSCSEL_IO & WUTSEL_LEG & WPDIS_WPDIS & WPCFG_WPCFGDIS & WPEND_WPENDMEM
   config __CONFIG4, DSWDTPS_DSWDTPS3 & DSWDTOSC_LPRC & RTCOSC_LPRC & DSBOREN_OFF & DSWDTEN_OFF

;------------------------------------------------------------------------------
; End of file
;------------------------------------------------------------------------------
