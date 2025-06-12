@ECHO OFF
CLS

ECHO ******************************************************************************
ECHO *                                                                            *
ECHO *                                  IFG GmbH                                  *
ECHO *                    Automated PLC-Software Update Installer                 *
ECHO *                                                                            *
::   *   Version   : 1.0                                                          *
::   *   written by: Frank Tretner                                                *
::   *                                                                            *
::   *   Version   : 1.1                                                          *
::   *   changed for WinSCP (passive ftp) by: Markus Oesker                       *
::   *                                                                            *
ECHO ******************************************************************************
ECHO.

set IP_ADDRESS="192.168.1.3"

ECHO.
ECHO.                                             
ECHO  Before start of update please:
ECHO  1. Stop running equipment!!!
ECHO  2. Stop  ALL running external software connected to the PLC.
ECHO  3. Close ALL web visualisations running on the PLC.
ECHO  4. Make sure, nobody else is connected to the PLC!
ECHO  5. Make sure, nobody else is trying to connect to the PLC!
ECHO.
ECHO  Press any key to continue.                                
PAUSE > NUL
ECHO.                                      
ECHO.                                      
ECHO *** Update in progress... ***
ECHO.                                      
MD PLC_OLD 2> NUL
DIR PLC_OLD > NUL
IF ERRORLEVEL 1 GOTO ERROR_LABEL1
WinSCP.COM -passive=on /log="plc_update.log" /command "open ftp://user:user@%IP_ADDRESS%" "cd PLC" "option batch continue" "echo" "ECHO *** Saving old files to PLC_OLD/: ***" "get -delete -transfer=binary DEFAULT.CHK DEFAULT.PRG PLC_OLD\\" "option batch abort" "echo" "ECHO *** Uploading new files: ***" "put -transfer=binary DEFAULT.CHK DEFAULT.PRG /PLC/" "exit"
IF ERRORLEVEL 1 GOTO ERROR_LABEL2

ECHO.
ECHO.
ECHO *** Job done ***
ECHO.
ECHO.
ECHO *** Please restart your PLC to take an effect ***
ECHO.
ECHO.

GOTO END_OF_FILE


: ERROR_LABEL1

ECHO Could not create or access folder PLC_OLD for a backup.
GOTO END_OF_FILE

: ERROR_LABEL2

ECHO Sorry. An error ocurred.
GOTO END_OF_FILE

:END_OF_FILE

ECHO Press any key to quit
PAUSE > NUL
