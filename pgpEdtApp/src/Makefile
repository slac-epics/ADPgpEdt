TOP=../..
include $(TOP)/configure/CONFIG
#----------------------------------------
# ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

LIBRARY_IOC = pgpEdt

DBD        += pgpEdtSupport.dbd

LIB_SRCS   += pgpEdt.cpp
LIB_SRCS   += pgpEdtSerial.cpp

USR_INCLUDES += -I/reg/neh/home/dhzhang/FPGA/PgpCardG3/software_new/include

include $(ADCORE)/ADApp/commonDriverMakefile

#=============================

include $(TOP)/configure/RULES
#----------------------------------------
# ADD RULES AFTER THIS LINE

