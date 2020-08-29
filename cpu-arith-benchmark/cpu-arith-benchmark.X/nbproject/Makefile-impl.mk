#
# Generated Makefile - do not edit!
#
# Edit the Makefile in the project folder instead (../Makefile). Each target
# has a pre- and a post- target defined where you can add customization code.
#
# This makefile implements macros and targets common to all configurations.
#
# NOCDDL


# Building and Cleaning subprojects are done by default, but can be controlled with the SUB
# macro. If SUB=no, subprojects will not be built or cleaned. The following macro
# statements set BUILD_SUB-CONF and CLEAN_SUB-CONF to .build-reqprojects-conf
# and .clean-reqprojects-conf unless SUB has the value 'no'
SUB_no=NO
SUBPROJECTS=${SUB_${SUB}}
BUILD_SUBPROJECTS_=.build-subprojects
BUILD_SUBPROJECTS_NO=
BUILD_SUBPROJECTS=${BUILD_SUBPROJECTS_${SUBPROJECTS}}
CLEAN_SUBPROJECTS_=.clean-subprojects
CLEAN_SUBPROJECTS_NO=
CLEAN_SUBPROJECTS=${CLEAN_SUBPROJECTS_${SUBPROJECTS}}


# Project Name
PROJECTNAME=cpu-arith-benchmark.X

# Active Configuration
DEFAULTCONF=PIC32MZ1024EFH064_MINI32_BB
CONF=${DEFAULTCONF}

# All Configurations
ALLCONFS=PIC16F15356 PIC16LF15356_BB PIC16F15376_CNANO PIC24FJ256GA702 dsPIC33EP256GP502 PIC32MX170F256B PIC32MZ1024EFH064 PIC32MZ1024EFH064_MINI32_BB PIC32MZ2048EFM144_CURIOSITY2 


# build
.build-impl: .build-pre
	${MAKE} -f nbproject/Makefile-${CONF}.mk SUBPROJECTS=${SUBPROJECTS} .build-conf


# clean
.clean-impl: .clean-pre
	${MAKE} -f nbproject/Makefile-${CONF}.mk SUBPROJECTS=${SUBPROJECTS} .clean-conf

# clobber
.clobber-impl: .clobber-pre .depcheck-impl
	    ${MAKE} SUBPROJECTS=${SUBPROJECTS} CONF=PIC16F15356 clean
	    ${MAKE} SUBPROJECTS=${SUBPROJECTS} CONF=PIC16LF15356_BB clean
	    ${MAKE} SUBPROJECTS=${SUBPROJECTS} CONF=PIC16F15376_CNANO clean
	    ${MAKE} SUBPROJECTS=${SUBPROJECTS} CONF=PIC24FJ256GA702 clean
	    ${MAKE} SUBPROJECTS=${SUBPROJECTS} CONF=dsPIC33EP256GP502 clean
	    ${MAKE} SUBPROJECTS=${SUBPROJECTS} CONF=PIC32MX170F256B clean
	    ${MAKE} SUBPROJECTS=${SUBPROJECTS} CONF=PIC32MZ1024EFH064 clean
	    ${MAKE} SUBPROJECTS=${SUBPROJECTS} CONF=PIC32MZ1024EFH064_MINI32_BB clean
	    ${MAKE} SUBPROJECTS=${SUBPROJECTS} CONF=PIC32MZ2048EFM144_CURIOSITY2 clean



# all
.all-impl: .all-pre .depcheck-impl
	    ${MAKE} SUBPROJECTS=${SUBPROJECTS} CONF=PIC16F15356 build
	    ${MAKE} SUBPROJECTS=${SUBPROJECTS} CONF=PIC16LF15356_BB build
	    ${MAKE} SUBPROJECTS=${SUBPROJECTS} CONF=PIC16F15376_CNANO build
	    ${MAKE} SUBPROJECTS=${SUBPROJECTS} CONF=PIC24FJ256GA702 build
	    ${MAKE} SUBPROJECTS=${SUBPROJECTS} CONF=dsPIC33EP256GP502 build
	    ${MAKE} SUBPROJECTS=${SUBPROJECTS} CONF=PIC32MX170F256B build
	    ${MAKE} SUBPROJECTS=${SUBPROJECTS} CONF=PIC32MZ1024EFH064 build
	    ${MAKE} SUBPROJECTS=${SUBPROJECTS} CONF=PIC32MZ1024EFH064_MINI32_BB build
	    ${MAKE} SUBPROJECTS=${SUBPROJECTS} CONF=PIC32MZ2048EFM144_CURIOSITY2 build



# dependency checking support
.depcheck-impl:
#	@echo "# This code depends on make tool being used" >.dep.inc
#	@if [ -n "${MAKE_VERSION}" ]; then \
#	    echo "DEPFILES=\$$(wildcard \$$(addsuffix .d, \$${OBJECTFILES}))" >>.dep.inc; \
#	    echo "ifneq (\$${DEPFILES},)" >>.dep.inc; \
#	    echo "include \$${DEPFILES}" >>.dep.inc; \
#	    echo "endif" >>.dep.inc; \
#	else \
#	    echo ".KEEP_STATE:" >>.dep.inc; \
#	    echo ".KEEP_STATE_FILE:.make.state.\$${CONF}" >>.dep.inc; \
#	fi
