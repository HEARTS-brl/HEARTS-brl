import datetime as dt
import os.path
import time as T
#
# DU - Derek's Utilities
#
# 11 Aug 2016 - Changes for Mk 4 release being Sun 9th Octobe at Cottage:
# Add temp sensor reading code
# Add LED Blinking code
# remove \n from "CONTINUED " log msg as misleading in log file!!
########################################################################
# 06 Dec 2016
# 1 - For GPIO_Mk_5 this module is now renamed to DU_v5.py but imported 
#     as "DU" - hence backward compatible if needed.
#
# 2 - Temperature readings from gettemp(ts):
#     -99.01 means that w1_slave file was not found for sensor
#     was -99.99 in previous versions. 
#     New test added for decoding temp sensor CRC data elementwhich is
#     1st line in w1_slave file.
#     -99.02 means that "YES" was not set & hence a CRC error in sensor

# Class Start ##########################################################
class c_datetime(object):
  ######################################################################
  def __init__(self):
    self.starttime = dt.datetime.now()
    
  ######################################################################    
  def logdatetime(self): 
    o_now = dt.datetime.now()
    o_now =str(o_now.isoformat()[:-5])
    return o_now.replace("T"," ")
  
 #######################################################################   
  def elapsedtime(self):  
    now  = dt.datetime.now()
    gap  = (now - self.starttime)
    return str(gap)[:-5]
  # end of class c_datetime(object):######################################


# Class Start ########################################################## 
class c_logger(object):
  ######################################################################
  def __init__(self,dirname,filename):
     self.logdirname  = dirname
     self.logfilename = filename
     self.o_DT = c_datetime()
     self.logfullpath = os.path.join(self.logdirname, self.logfilename)

     if os.path.isfile(self.logfullpath):
          # Append existing file
          self.write("*****_CONTINUED after a Power Cut or User ReBoot_*****")
          self.write("Log_Appended: " + self.logfullpath)
     else:
          # New file will be created
          self.write("******__START__*****")  
          self.write("Log_Created : " + self.logfullpath)
  ######################################################################
  def printme(self):
     print(self.logfullpath+"\n")

  ######################################################################
  def write(self,msg):
    ldatetime = self.o_DT.logdatetime()
    fh = open(self.logfullpath,"a")
    fh.write(ldatetime+ " " + msg  + "\n")
    fh.close()  

  def GetLogDirName(self):
    return self.logdirname
		
  # end of class c_logger(object):########################################    


# Class Start ########################################################## 
class c_comms(object):
######################################################################
   def __init__(self,o_LOG):
     self.o_LOG = o_LOG
     
   def  cmdline(self, command):
     from subprocess import PIPE, Popen
     process = Popen(
       args = command,
       stdout = PIPE,
       shell = True )
     print("pre textstr" )
     textstr = str(process.communicate()[0])
     print("post textstr")
     # string returned starts -b'- and ends with a single -'- (ignore hyphens)
     L_text = textstr[2:-1].split("\\n")
     listlen = L_text.__len__()
     if listlen > 1 :
       self.o_LOG.write("***INFORMATION***_ouput_from_cmdline_for: " + command)
       for item in L_text:         
         self.o_LOG.write(item)
     else:
         self.o_LOG.write("***ERROR***_cmdline_for: " + command)
         self.o_LOG.write(L_text[0])
     return L_text

   def checkextnetwork(self, pingtarget, pingcnt):
     if pingcnt <= 0:
       pingcnt = 4
     print("call to cmdline")
     L_result = self.cmdline("ping " + pingtarget + " -c " + str(pingcnt))
     print("post cmdline call")  
     # count all succesful pings
     icnt = 0
     for line in   L_result:
        if "bytes from" in line:
           icnt = icnt+ 1
           
     # ping count should equal required number of actual pings
     B_Networkstatus = pingcnt == icnt
     self.o_LOG.write("EXTERNAL NETWORK found  is : " + str(B_Networkstatus))
     return B_Networkstatus
   
# end of class c_comms(object):########################################    


def gettemp(ts):

    # NB *** one wire network has been enabled on BCM pin 17 (Pi pin  11) ***
	# Temp sensor type DS18B20
	
	if ts == "Internal" :	
		tempfile = "/sys/bus/w1/devices/28-0000073d9052/w1_slave"		
		#tempfile = "/sys/bus/w1/devices/28-01161311d3ee/w1_slave"		
	elif  ts == "External": 
		tempfile = "/sys/bus/w1/devices/28-80000028a8a9/w1_slave" #External Temp sensor 1
		#tempfile = "/sys/bus/w1/devices/28-8000002896eb/w1_slave" #External Temp sensor 2

	# Check presence of sensor	w1_slave file	
	if os.path.isfile(tempfile):
		nofile = False
	else:
		nofile = True	
		
	if nofile:
		# as temp sensor not detected just set obvious false value
		temperature = "%6.2f" % (-99.01)
	else:
		# temp sensor w1_slave file is present so process value
		fh      = open(tempfile)
		thetext = fh.read().split("\n")
		fh.close()
			
		crc      = thetext[0].rsplit(" ",1)[1]   # 1st line of w1_slave file		
		tempdata = thetext[1].rsplit("t=",1)[1]  # 2nd line of w1_slave file
		
		if  crc != 'YES': 
			temperature = "%6.2f" % (-99.02)
		else:	
			temperature = float(tempdata)
			temperature = "%6.2f" % (temperature / 1000)
	
	return temperature
	
def blink(GPIO, blinktime, pin):
		GPIO.setup(pin,  GPIO.OUT)
		GPIO.output(pin, GPIO.HIGH)
		T.sleep(blinktime)
		GPIO.output(pin, GPIO.LOW)
		return
		