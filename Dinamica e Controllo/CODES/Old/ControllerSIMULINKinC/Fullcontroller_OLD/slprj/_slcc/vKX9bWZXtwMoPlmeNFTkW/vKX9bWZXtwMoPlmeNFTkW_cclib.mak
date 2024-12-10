#------------------------ Tool Specifications & Options ----------------------

COMPILER  =  lcc

CC        =  "C:\Program Files\MATLAB\R2022b\sys\lcc64\lcc64\bin\lcc64.exe"
LD        =  "C:\Program Files\MATLAB\R2022b\sys\lcc64\lcc64\bin\lcclnk64.exe"
LIBCMD    =  "C:\Program Files\MATLAB\R2022b\sys\lcc64\lcc64\bin\lcclib64.exe"
CFLAGS    =  -dll -noregistrylookup  -c -Zp8 -DLCC_WIN64 -DMATLAB_MEX_FILE -nodeclspec
LDFLAGS   =  -s -dll -entry LibMain vKX9bWZXtwMoPlmeNFTkW_cclib.def -L"C:\Program Files\MATLAB\R2022b\sys\lcc64\lcc64\lib64"

OBJECTS = \
	   controller_full.obj \
	   slcc_interface_vKX9bWZXtwMoPlmeNFTkW.obj \
	   lccstub.obj \

STATICLIBS = \

#------------------------------ Include/Lib Path ------------------------------

INCLUDE_PATH = \
     -I"c:\program files\matlab\r2022b\extern\include" \
     -I"c:\program files\matlab\r2022b\simulink\include" \
     -I"c:\users\pbrus\onedrive - automation engineering association\falco\dinamica e controllo\codes\controllersimulinkinc\fullcontroller_old\slprj\_slcc\vkx9bwzxtwmoplmenftkw" \
     -I"c:\users\pbrus\onedrive - automation engineering association\falco\dinamica e controllo\codes\controllersimulinkinc\fullcontroller_old" \
     -I"c:\program files\matlab\r2022b\sys\lcc64\lcc64\include64" \
     -I"c:\program files\matlab\r2022b\sys\lcc64\lcc64\mex" \

#--------------------------------- Rules --------------------------------------

vKX9bWZXtwMoPlmeNFTkW_cclib.dll : $(MAKEFILE) $(OBJECTS)
	$(LD) $(LDFLAGS) /OUT:vKX9bWZXtwMoPlmeNFTkW_cclib.dll $(OBJECTS)  $(STATICLIBS) "C:\Program Files\MATLAB\R2022b\extern\lib\win64\microsoft\libmex.lib" "C:\Program Files\MATLAB\R2022b\extern\lib\win64\microsoft\libmx.lib"
controller_full.obj :	c:\users\pbrus\ONEDRI~2\falco\DINAMI~1\codes\CONTRO~2\FULLCO~1\CONTRO~1.C
	$(CC) $(CFLAGS) $(INCLUDE_PATH) "c:\users\pbrus\onedrive - automation engineering association\falco\dinamica e controllo\codes\controllersimulinkinc\fullcontroller_old\controller_full.c"
slcc_interface_vKX9bWZXtwMoPlmeNFTkW.obj :	C:\Users\pbrus\ONEDRI~2\FALCO\DINAMI~1\CODES\CONTRO~2\FULLCO~1\slprj\_slcc\VKX9BW~1\SLCC_I~1.C
	$(CC) $(CFLAGS) $(INCLUDE_PATH) "C:\Users\pbrus\OneDrive - Automation Engineering Association\FALCO\Dinamica e Controllo\CODES\ControllerSIMULINKinC\Fullcontroller_OLD\slprj\_slcc\vKX9bWZXtwMoPlmeNFTkW\slcc_interface_vKX9bWZXtwMoPlmeNFTkW.c"
lccstub.obj :	C:\PROGRA~1\MATLAB\R2022b\sys\lcc64\lcc64\mex\lccstub.c
	$(CC) $(CFLAGS) $(INCLUDE_PATH) "C:\Program Files\MATLAB\R2022b\sys\lcc64\lcc64\mex\lccstub.c"
