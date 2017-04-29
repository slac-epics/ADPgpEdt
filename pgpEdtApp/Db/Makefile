TOP=../..
include $(TOP)/configure/CONFIG
#----------------------------------------
# ADD MACRO DEFINITIONS AFTER THIS LINE

#----------------------------------------------------
# Optimization of db files using dbst (DEFAULT: NO)
# DB_OPT = YES

#----------------------------------------------------
# Create and install (or just install)
# databases, templates, substitutions like this

# Drivers
DB_INSTALLS += pgpEdt.template

# Install archive files
ARCHIVE += pgpEdt.tpl-arch

# Install autosave files
REQ     += pgpEdt_settings.req

#----------------------------------------------------
# If <anyname>.db template is not named <anyname>*.template add
# <anyname>_template = <templatename>

include $(TOP)/configure/RULES
#----------------------------------------
# ADD RULES AFTER THIS LINE

