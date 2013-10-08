#
# Generated Makefile - do not edit!
#
# Edit the Makefile in the project folder instead (../Makefile). Each target
# has a -pre and a -post target defined where you can add customized code.
#
# This makefile implements configuration specific macros and targets.


# Include project Makefile
ifeq "${IGNORE_LOCAL}" "TRUE"
# do not include local makefile. User is passing all local related variables already
else
include Makefile
# Include makefile containing local settings
ifeq "$(wildcard nbproject/Makefile-local-PIC24FJ64GB004_PIM.mk)" "nbproject/Makefile-local-PIC24FJ64GB004_PIM.mk"
include nbproject/Makefile-local-PIC24FJ64GB004_PIM.mk
endif
endif

# Environment
MKDIR=gnumkdir -p
RM=rm -f 
MV=mv 
CP=cp 

# Macros
CND_CONF=PIC24FJ64GB004_PIM
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
IMAGE_TYPE=debug
OUTPUT_SUFFIX=elf
DEBUGGABLE_SUFFIX=elf
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/MPLAB.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
else
IMAGE_TYPE=production
OUTPUT_SUFFIX=hex
DEBUGGABLE_SUFFIX=elf
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/MPLAB.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
endif

# Object Directory
OBJECTDIR=build/${CND_CONF}/${IMAGE_TYPE}

# Distribution Directory
DISTDIR=dist/${CND_CONF}/${IMAGE_TYPE}

# Object Files Quoted if spaced
OBJECTFILES_QUOTED_IF_SPACED=${OBJECTDIR}/_ext/491339551/FSIO.o ${OBJECTDIR}/_ext/926206843/usb_host.o ${OBJECTDIR}/_ext/921800108/usb_host_msd.o ${OBJECTDIR}/_ext/921800108/usb_host_msd_scsi.o ${OBJECTDIR}/_ext/2048740170/uart2.o ${OBJECTDIR}/_ext/1472/usb_config.o ${OBJECTDIR}/_ext/1472/usb_data_logger.o
POSSIBLE_DEPFILES=${OBJECTDIR}/_ext/491339551/FSIO.o.d ${OBJECTDIR}/_ext/926206843/usb_host.o.d ${OBJECTDIR}/_ext/921800108/usb_host_msd.o.d ${OBJECTDIR}/_ext/921800108/usb_host_msd_scsi.o.d ${OBJECTDIR}/_ext/2048740170/uart2.o.d ${OBJECTDIR}/_ext/1472/usb_config.o.d ${OBJECTDIR}/_ext/1472/usb_data_logger.o.d

# Object Files
OBJECTFILES=${OBJECTDIR}/_ext/491339551/FSIO.o ${OBJECTDIR}/_ext/926206843/usb_host.o ${OBJECTDIR}/_ext/921800108/usb_host_msd.o ${OBJECTDIR}/_ext/921800108/usb_host_msd_scsi.o ${OBJECTDIR}/_ext/2048740170/uart2.o ${OBJECTDIR}/_ext/1472/usb_config.o ${OBJECTDIR}/_ext/1472/usb_data_logger.o


CFLAGS=
ASFLAGS=
LDLIBSOPTIONS=

############# Tool locations ##########################################
# If you copy a project from one host to another, the path where the  #
# compiler is installed may be different.                             #
# If you open this project with MPLAB X in the new host, this         #
# makefile will be regenerated and the paths will be corrected.       #
#######################################################################
# fixDeps replaces a bunch of sed/cat/printf statements that slow down the build
FIXDEPS=fixDeps

.build-conf:  ${BUILD_SUBPROJECTS}
	${MAKE} ${MAKE_OPTIONS} -f nbproject/Makefile-PIC24FJ64GB004_PIM.mk dist/${CND_CONF}/${IMAGE_TYPE}/MPLAB.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}

MP_PROCESSOR_OPTION=24FJ64GB002
MP_LINKER_FILE_OPTION=,-Tp24FJ64GB002.gld
# ------------------------------------------------------------------------------------
# Rules for buildStep: assemble
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: assembleWithPreprocess
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: compile
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
${OBJECTDIR}/_ext/491339551/FSIO.o: ../../../../Microchip/MDD\ File\ System/FSIO.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/491339551 
	@${RM} ${OBJECTDIR}/_ext/491339551/FSIO.o.d 
	@${RM} ${OBJECTDIR}/_ext/491339551/FSIO.o.ok ${OBJECTDIR}/_ext/491339551/FSIO.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/491339551/FSIO.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -fno-short-double -I".." -I"../../../../Microchip/Include" -Os -MMD -MF "${OBJECTDIR}/_ext/491339551/FSIO.o.d" -o ${OBJECTDIR}/_ext/491339551/FSIO.o "../../../../Microchip/MDD File System/FSIO.c"    
	
${OBJECTDIR}/_ext/926206843/usb_host.o: ../../../../Microchip/USB/usb_host.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/926206843 
	@${RM} ${OBJECTDIR}/_ext/926206843/usb_host.o.d 
	@${RM} ${OBJECTDIR}/_ext/926206843/usb_host.o.ok ${OBJECTDIR}/_ext/926206843/usb_host.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/926206843/usb_host.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -fno-short-double -I".." -I"../../../../Microchip/Include" -Os -MMD -MF "${OBJECTDIR}/_ext/926206843/usb_host.o.d" -o ${OBJECTDIR}/_ext/926206843/usb_host.o ../../../../Microchip/USB/usb_host.c    
	
${OBJECTDIR}/_ext/921800108/usb_host_msd.o: ../../../../Microchip/USB/MSD\ Host\ Driver/usb_host_msd.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/921800108 
	@${RM} ${OBJECTDIR}/_ext/921800108/usb_host_msd.o.d 
	@${RM} ${OBJECTDIR}/_ext/921800108/usb_host_msd.o.ok ${OBJECTDIR}/_ext/921800108/usb_host_msd.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/921800108/usb_host_msd.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -fno-short-double -I".." -I"../../../../Microchip/Include" -Os -MMD -MF "${OBJECTDIR}/_ext/921800108/usb_host_msd.o.d" -o ${OBJECTDIR}/_ext/921800108/usb_host_msd.o "../../../../Microchip/USB/MSD Host Driver/usb_host_msd.c"    
	
${OBJECTDIR}/_ext/921800108/usb_host_msd_scsi.o: ../../../../Microchip/USB/MSD\ Host\ Driver/usb_host_msd_scsi.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/921800108 
	@${RM} ${OBJECTDIR}/_ext/921800108/usb_host_msd_scsi.o.d 
	@${RM} ${OBJECTDIR}/_ext/921800108/usb_host_msd_scsi.o.ok ${OBJECTDIR}/_ext/921800108/usb_host_msd_scsi.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/921800108/usb_host_msd_scsi.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -fno-short-double -I".." -I"../../../../Microchip/Include" -Os -MMD -MF "${OBJECTDIR}/_ext/921800108/usb_host_msd_scsi.o.d" -o ${OBJECTDIR}/_ext/921800108/usb_host_msd_scsi.o "../../../../Microchip/USB/MSD Host Driver/usb_host_msd_scsi.c"    
	
${OBJECTDIR}/_ext/2048740170/uart2.o: ../../../../Microchip/Common/uart2.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/2048740170 
	@${RM} ${OBJECTDIR}/_ext/2048740170/uart2.o.d 
	@${RM} ${OBJECTDIR}/_ext/2048740170/uart2.o.ok ${OBJECTDIR}/_ext/2048740170/uart2.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/2048740170/uart2.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -fno-short-double -I".." -I"../../../../Microchip/Include" -Os -MMD -MF "${OBJECTDIR}/_ext/2048740170/uart2.o.d" -o ${OBJECTDIR}/_ext/2048740170/uart2.o ../../../../Microchip/Common/uart2.c    
	
${OBJECTDIR}/_ext/1472/usb_config.o: ../usb_config.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1472 
	@${RM} ${OBJECTDIR}/_ext/1472/usb_config.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/usb_config.o.ok ${OBJECTDIR}/_ext/1472/usb_config.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/usb_config.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -fno-short-double -I".." -I"../../../../Microchip/Include" -Os -MMD -MF "${OBJECTDIR}/_ext/1472/usb_config.o.d" -o ${OBJECTDIR}/_ext/1472/usb_config.o ../usb_config.c    
	
${OBJECTDIR}/_ext/1472/usb_data_logger.o: ../usb_data_logger.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1472 
	@${RM} ${OBJECTDIR}/_ext/1472/usb_data_logger.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/usb_data_logger.o.ok ${OBJECTDIR}/_ext/1472/usb_data_logger.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/usb_data_logger.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -fno-short-double -I".." -I"../../../../Microchip/Include" -Os -MMD -MF "${OBJECTDIR}/_ext/1472/usb_data_logger.o.d" -o ${OBJECTDIR}/_ext/1472/usb_data_logger.o ../usb_data_logger.c    
	
else
${OBJECTDIR}/_ext/491339551/FSIO.o: ../../../../Microchip/MDD\ File\ System/FSIO.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/491339551 
	@${RM} ${OBJECTDIR}/_ext/491339551/FSIO.o.d 
	@${RM} ${OBJECTDIR}/_ext/491339551/FSIO.o.ok ${OBJECTDIR}/_ext/491339551/FSIO.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/491339551/FSIO.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -fno-short-double -I".." -I"../../../../Microchip/Include" -Os -MMD -MF "${OBJECTDIR}/_ext/491339551/FSIO.o.d" -o ${OBJECTDIR}/_ext/491339551/FSIO.o "../../../../Microchip/MDD File System/FSIO.c"    
	
${OBJECTDIR}/_ext/926206843/usb_host.o: ../../../../Microchip/USB/usb_host.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/926206843 
	@${RM} ${OBJECTDIR}/_ext/926206843/usb_host.o.d 
	@${RM} ${OBJECTDIR}/_ext/926206843/usb_host.o.ok ${OBJECTDIR}/_ext/926206843/usb_host.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/926206843/usb_host.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -fno-short-double -I".." -I"../../../../Microchip/Include" -Os -MMD -MF "${OBJECTDIR}/_ext/926206843/usb_host.o.d" -o ${OBJECTDIR}/_ext/926206843/usb_host.o ../../../../Microchip/USB/usb_host.c    
	
${OBJECTDIR}/_ext/921800108/usb_host_msd.o: ../../../../Microchip/USB/MSD\ Host\ Driver/usb_host_msd.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/921800108 
	@${RM} ${OBJECTDIR}/_ext/921800108/usb_host_msd.o.d 
	@${RM} ${OBJECTDIR}/_ext/921800108/usb_host_msd.o.ok ${OBJECTDIR}/_ext/921800108/usb_host_msd.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/921800108/usb_host_msd.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -fno-short-double -I".." -I"../../../../Microchip/Include" -Os -MMD -MF "${OBJECTDIR}/_ext/921800108/usb_host_msd.o.d" -o ${OBJECTDIR}/_ext/921800108/usb_host_msd.o "../../../../Microchip/USB/MSD Host Driver/usb_host_msd.c"    
	
${OBJECTDIR}/_ext/921800108/usb_host_msd_scsi.o: ../../../../Microchip/USB/MSD\ Host\ Driver/usb_host_msd_scsi.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/921800108 
	@${RM} ${OBJECTDIR}/_ext/921800108/usb_host_msd_scsi.o.d 
	@${RM} ${OBJECTDIR}/_ext/921800108/usb_host_msd_scsi.o.ok ${OBJECTDIR}/_ext/921800108/usb_host_msd_scsi.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/921800108/usb_host_msd_scsi.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -fno-short-double -I".." -I"../../../../Microchip/Include" -Os -MMD -MF "${OBJECTDIR}/_ext/921800108/usb_host_msd_scsi.o.d" -o ${OBJECTDIR}/_ext/921800108/usb_host_msd_scsi.o "../../../../Microchip/USB/MSD Host Driver/usb_host_msd_scsi.c"    
	
${OBJECTDIR}/_ext/2048740170/uart2.o: ../../../../Microchip/Common/uart2.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/2048740170 
	@${RM} ${OBJECTDIR}/_ext/2048740170/uart2.o.d 
	@${RM} ${OBJECTDIR}/_ext/2048740170/uart2.o.ok ${OBJECTDIR}/_ext/2048740170/uart2.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/2048740170/uart2.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -fno-short-double -I".." -I"../../../../Microchip/Include" -Os -MMD -MF "${OBJECTDIR}/_ext/2048740170/uart2.o.d" -o ${OBJECTDIR}/_ext/2048740170/uart2.o ../../../../Microchip/Common/uart2.c    
	
${OBJECTDIR}/_ext/1472/usb_config.o: ../usb_config.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1472 
	@${RM} ${OBJECTDIR}/_ext/1472/usb_config.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/usb_config.o.ok ${OBJECTDIR}/_ext/1472/usb_config.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/usb_config.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -fno-short-double -I".." -I"../../../../Microchip/Include" -Os -MMD -MF "${OBJECTDIR}/_ext/1472/usb_config.o.d" -o ${OBJECTDIR}/_ext/1472/usb_config.o ../usb_config.c    
	
${OBJECTDIR}/_ext/1472/usb_data_logger.o: ../usb_data_logger.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1472 
	@${RM} ${OBJECTDIR}/_ext/1472/usb_data_logger.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/usb_data_logger.o.ok ${OBJECTDIR}/_ext/1472/usb_data_logger.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/usb_data_logger.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -fno-short-double -I".." -I"../../../../Microchip/Include" -Os -MMD -MF "${OBJECTDIR}/_ext/1472/usb_data_logger.o.d" -o ${OBJECTDIR}/_ext/1472/usb_data_logger.o ../usb_data_logger.c    
	
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: link
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
dist/${CND_CONF}/${IMAGE_TYPE}/MPLAB.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk    
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_CC} $(MP_EXTRA_LD_PRE)  -omf=elf -mcpu=$(MP_PROCESSOR_OPTION)  -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -o dist/${CND_CONF}/${IMAGE_TYPE}/MPLAB.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX} ${OBJECTFILES_QUOTED_IF_SPACED}         -Wl,--defsym=__MPLAB_BUILD=1,--heap=2000,--no-check-sections$(MP_EXTRA_LD_POST)$(MP_LINKER_FILE_OPTION),--defsym=__ICD2RAM=1,--defsym=__MPLAB_DEBUG=1,--defsym=__DEBUG=1,--defsym=__MPLAB_DEBUGGER_PK3=1
else
dist/${CND_CONF}/${IMAGE_TYPE}/MPLAB.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk   
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_CC} $(MP_EXTRA_LD_PRE)  -omf=elf -mcpu=$(MP_PROCESSOR_OPTION)  -o dist/${CND_CONF}/${IMAGE_TYPE}/MPLAB.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX} ${OBJECTFILES_QUOTED_IF_SPACED}         -Wl,--defsym=__MPLAB_BUILD=1,--heap=2000,--no-check-sections$(MP_EXTRA_LD_POST)$(MP_LINKER_FILE_OPTION)
	${MP_CC_DIR}\\pic30-bin2hex dist/${CND_CONF}/${IMAGE_TYPE}/MPLAB.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX} -omf=elf
endif


# Subprojects
.build-subprojects:


# Subprojects
.clean-subprojects:

# Clean Targets
.clean-conf: ${CLEAN_SUBPROJECTS}
	${RM} -r build/PIC24FJ64GB004_PIM
	${RM} -r dist/PIC24FJ64GB004_PIM

# Enable dependency checking
.dep.inc: .depcheck-impl

DEPFILES=$(shell mplabwildcard ${POSSIBLE_DEPFILES})
ifneq (${DEPFILES},)
include ${DEPFILES}
endif
